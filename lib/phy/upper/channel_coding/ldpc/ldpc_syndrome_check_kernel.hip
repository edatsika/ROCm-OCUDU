// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

// Syndrome check kernel for early-stop in batched LDPC decoding.
//
// Launch geometry:
//   Grid  : dim3(1)
//   Block : dim3(BATCH_SIZE)   threadIdx.x = codeword index
//
// Each thread checks all parity equations for its codeword.
// On success it sets d_valid_flags[b] = 1 (one byte per codeword,
// avoids the bitmask overflow that occurs when BATCH_SIZE > 32).

#include <hip/hip_runtime.h>

// Must match ldpc_decoder_impl.h and batched_fused_kernel.hip
#define BATCH_SIZE          256
#define MAX_NODES_PER_LAYER 19

__global__ void ldpc_syndrome_check_kernel(
    const int8_t*   __restrict__ d_soft_bits,     // [node][Z][Batch]
    const uint8_t*  __restrict__ d_adj,           // [layer][edge], 255 = no edge
    const uint16_t* __restrict__ d_rev_shifts,    // [layer][edge]
    uint8_t*        __restrict__ d_valid_flags,   // [Batch] = 1 if codeword is valid
    uint32_t                     nof_layers,
    uint32_t                     Z,
    uint32_t                     batch_size)
{
    const uint32_t b = threadIdx.x;
    if (b >= batch_size) return;

    bool all_passed = true;

    for (uint32_t l = 0; l < nof_layers && all_passed; ++l) {
        // Check the Z parity equations that correspond to this lifted row.
        // For each z position, XOR the hard decisions of all connected variable
        // nodes (after applying their cyclic shift).
        for (uint32_t z = 0; z < Z && all_passed; ++z) {
            uint32_t row_parity = 0;

            for (uint32_t n = 0; n < MAX_NODES_PER_LAYER; ++n) {
                const uint8_t node_id = d_adj[l * MAX_NODES_PER_LAYER + n];
                if (node_id == 255u) break;  // sentinel: end of row

                const uint16_t shift     = d_rev_shifts[l * MAX_NODES_PER_LAYER + n];
                const uint32_t shifted_z = (z + shift) % Z;

                // soft_bits layout: [node][Z][Batch]
                const size_t offset = (size_t)node_id * Z * batch_size
                                    + shifted_z * batch_size + b;

                // Hard decision: LLR < 0  →  bit = 1
                row_parity ^= (d_soft_bits[offset] < 0) ? 1u : 0u;
            }

            if (row_parity != 0u) {
                all_passed = false;
            }
        }
    }

    // One write per thread – no atomic needed (each thread owns its flag byte)
    d_valid_flags[b] = all_passed ? 1u : 0u;
}

// C linkage wrapper
extern "C" {

void ldpc_syndrome_check_batch(
    hipStream_t     stream,
    const int8_t*   d_soft_bits,
    const uint8_t*  d_adj,
    const uint16_t* d_rev_shifts,
    uint8_t*        d_valid_flags,
    uint32_t        nof_layers,
    uint32_t        lifting_size,
    uint32_t        batch_size)
{
    dim3 grid(1);
    dim3 block(BATCH_SIZE);

    hipLaunchKernelGGL(
        ldpc_syndrome_check_kernel,
        grid, block,
        0, stream,
        d_soft_bits, d_adj, d_rev_shifts,
        d_valid_flags,
        nof_layers, lifting_size, batch_size);
}

} // extern "C"
