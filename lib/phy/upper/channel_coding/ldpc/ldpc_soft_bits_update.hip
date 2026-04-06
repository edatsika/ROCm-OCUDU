#include <hip/hip_runtime.h>
#include <stdint.h>

__global__ void ldpc_soft_bits_update_kernel(
    float* __restrict__ d_soft_bits,    // Global LLRs
    float* __restrict__ d_v2c_copy,     
    float* __restrict__ d_c2v_messages, // New C2V msgs (from previous kernel)
    const uint16_t* __restrict__ d_adj_matrix,
    const uint16_t* __restrict__ d_shifts,
    const uint8_t* __restrict__ d_row_offsets,
    const uint8_t* __restrict__ d_row_lengths,
    int lifting_size,
    int check_node)
{
    int tid = hipThreadIdx_x;

    if (tid < lifting_size) {
        int start = d_row_offsets[check_node];
        int num = d_row_lengths[check_node];

        for (int i = 0; i < num; ++i) {
            uint16_t i_node = d_adj_matrix[start + i];
            uint16_t shift = d_shifts[start + i];

            // Circular shift indexing
            int rotated_tid = (tid + shift) % lifting_size;
            
            int soft_idx = i_node * lifting_size + tid;
            int msg_idx  = (start + i) * lifting_size + rotated_tid;

            // soft_bits = v2c_copy + c2v_new
            // Use v2c_copy stored in first kernel
            d_soft_bits[soft_idx] = d_v2c_copy[msg_idx] + d_c2v_messages[msg_idx];
        }
    }
}

// Wrapper called by CPU
extern "C" {
void ldpc_soft_bits_update(
    float* d_soft_bits, float* d_v2c_copy, float* d_c2v,
    const uint16_t* d_adj, const uint16_t* d_shifts,
    const uint8_t* d_offsets, const uint8_t* d_lengths,
    int lifting_size, int check_node) 
{
    dim3 block(lifting_size); 
    dim3 grid(1);

    hipLaunchKernelGGL(ldpc_soft_bits_update_kernel, grid, block, 0, 0, 
        d_soft_bits, d_v2c_copy, d_c2v, d_adj, d_shifts, d_offsets, d_lengths, 
        lifting_size, check_node);
}
}
