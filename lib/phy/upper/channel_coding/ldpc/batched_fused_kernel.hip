// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

// Batched layered normalised min-sum LDPC kernel.
//
// Launch geometry (called once per layer per iteration):
//   Grid  : dim3(1, lifting_size)  blockIdx.y = z-index (0 .. Z-1)
//   Block : dim3(BATCH_SIZE)       threadIdx.x = codeword index (0 .. B-1)
//
// Memory layouts
//   soft_bits  : [node_id][Z][Batch]  = node_id * Z * B + z * B + b
//   check_to_var: [layer][edge][Z][Batch]
//                = (layer * MAX_NODES_PER_LAYER + edge) * Z * B + z * B + b
//
// Each thread (b, z) handles one codeword for one lifted bit position.
// All z-threads for the same codeword work independently (no LDS needed).

#include <hip/hip_runtime.h>

// Must match ldpc_decoder_impl.h
#define BATCH_SIZE         256
// Upper bound on check-node degree across BG1 and BG2
#define MAX_NODES_PER_LAYER 19

#include <algorithm>
#include <cstdio>
#include <cstdlib>

#ifndef __clamp_i8
__device__ static inline int8_t __clamp_i8(int val, int min_val, int max_val) {
    return (int8_t)(val < min_val ? min_val : (val > max_val ? max_val : val));
}
#endif

// Internal kernel: one layer, all codewords, one z-slice per block-row
__global__ void ldpc_fused_layer_kernel(
    int8_t* __restrict__        soft_bits,
    int8_t* __restrict__        check_to_var,
    const uint16_t* __restrict__ d_shifts,      // [layer][edge]
    const uint8_t*  __restrict__ d_adj,         // [layer][edge], 255 = no edge
    const uint8_t               nof_edges,      // degree of this check node
    const int                   layer_idx,      // which check node we are processing
    const uint16_t              Z,              // lifting size
    const float                 scaling_factor)
{
    const int b     = threadIdx.x;   // codeword index
    const int z_idx = blockIdx.y;    // bit position within the lifted group

    if (b >= BATCH_SIZE) return;

    // PHASE 1: compute extrinsic v2c for every edge and gather min-sum
    //          statistics entirely in registers (no LDS required).
    
    int8_t  local_v2c[MAX_NODES_PER_LAYER];
    int8_t  min1      = 127;
    int8_t  min2      = 127;
    uint8_t min_idx   = 0;
    uint8_t sign_xor  = 0;   // XOR of all v2c sign bits

    for (uint8_t i = 0; i < nof_edges; ++i) {
        const uint8_t  node_id = d_adj   [layer_idx * MAX_NODES_PER_LAYER + i];
        const uint16_t shift   = d_shifts [layer_idx * MAX_NODES_PER_LAYER + i];

        // Cyclic shift applied to the z dimension
        const int rotated_z = (z_idx + (int)shift) % (int)Z;

        // Address in soft_bits (layout [node][Z][Batch])
        const int sb_addr  = (int)node_id * (int)Z * BATCH_SIZE
                           + rotated_z * BATCH_SIZE + b;

        // Address in check_to_var (layout [layer][edge][Z][Batch])
        const int c2v_addr = ((layer_idx * MAX_NODES_PER_LAYER + i) * (int)Z + z_idx)
                           * BATCH_SIZE + b;

        // Extrinsic v2c = posterior LLR minus current c2v extrinsic
        const int8_t sb    = soft_bits    [sb_addr];
        const int8_t c2v   = check_to_var [c2v_addr];
        const int8_t v2c   = (int8_t)__clamp_i8((int)sb - (int)c2v, -127, 127);

        local_v2c[i] = v2c;

        const int8_t  abs_v2c = (v2c < 0) ? -v2c : v2c;
        sign_xor ^= (uint8_t)(v2c < 0 ? 1 : 0);

        if (abs_v2c < min1) { min2 = min1; min1 = abs_v2c; min_idx = i; }
        else if (abs_v2c < min2) { min2 = abs_v2c; }
    }

    // PHASE 2: update c2v messages and apply the layered soft-bit update.
    //          Uses the cached local_v2c[] — no re-read of soft_bits.

    const uint8_t sign_prod = sign_xor;   // parity of all signs

    for (uint8_t i = 0; i < nof_edges; ++i) {
        const uint8_t  node_id = d_adj   [layer_idx * MAX_NODES_PER_LAYER + i];
        const uint16_t shift   = d_shifts [layer_idx * MAX_NODES_PER_LAYER + i];
        const int rotated_z    = (z_idx + (int)shift) % (int)Z;

        const int sb_addr  = (int)node_id * (int)Z * BATCH_SIZE
                           + rotated_z * BATCH_SIZE + b;
        const int c2v_addr = ((layer_idx * MAX_NODES_PER_LAYER + i) * (int)Z + z_idx)
                           * BATCH_SIZE + b;

        // Leave-one-out min-sum
        const int8_t chosen_min = (i == min_idx) ? min2 : min1;
        // Scaled min-sum (clamped to stay in int8 range)
        const int8_t scaled = (int8_t)__clamp_i8((int)(chosen_min * scaling_factor), 0, 127);

        // Extrinsic sign = global sign product divided by this edge's sign
        const uint8_t cur_sign  = (local_v2c[i] < 0) ? 1u : 0u;
        const uint8_t final_sign = sign_prod ^ cur_sign;
        const int8_t  new_c2v   = final_sign ? -scaled : scaled;

        // Store updated c2v
        check_to_var[c2v_addr] = new_c2v;

        // Layered soft-bit update:
        //   soft_bits_new = v2c + new_c2v
        // v2c already has old c2v subtracted (Phase 1), so this is equivalent to
        //   soft_bits_new = soft_bits_old - old_c2v + new_c2v
        const int8_t updated_sb = (int8_t)__clamp_i8((int)local_v2c[i] + (int)new_c2v, -127, 127);
        soft_bits[sb_addr] = updated_sb;
    }
}

// C linkage wrapper – called from ldpc_decoder_impl.cpp
extern "C" {

void ldpc_process_layer_fused_batch(
    hipStream_t     stream,
    int8_t*         d_soft_bits,
    int8_t*         d_c2v,
    const uint16_t* d_shifts,
    const uint8_t*  d_adj,
    uint8_t         nof_edges,       // degree of this check node
    int             layer_idx,       // which check node
    int             lifting_size,    // Z
    float           scaling_factor)
{
    // One block-row per z, one thread per codeword
    dim3 grid(1, lifting_size);
    dim3 block(BATCH_SIZE);

    hipLaunchKernelGGL(
        ldpc_fused_layer_kernel,
        grid, block,
        0, stream,
        d_soft_bits, d_c2v,
        d_shifts, d_adj,
        nof_edges,
        layer_idx,
        (uint16_t)lifting_size,
        scaling_factor);
}

} // extern "C"
