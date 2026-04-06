#include <hip/hip_runtime.h>
#include <stdint.h>

__global__ void ldpc_v2c_subtraction_kernel(
    float* __restrict__ d_soft_bits,      
    float* __restrict__ d_c2v_messages,   
    float* __restrict__ d_v2c_messages,   
    float* __restrict__ d_v2c_copy,       
    const uint16_t* __restrict__ d_adj_matrix, 
    const uint8_t* __restrict__ d_row_offsets, 
    const uint8_t* __restrict__ d_row_lengths, 
    int lifting_size,                          
    int check_node,                            
    bool is_initialized)                       
{
    int tid = hipThreadIdx_x; 
    if (tid < lifting_size) {
        int start_idx = d_row_offsets[check_node];
        int num_nodes = d_row_lengths[check_node];

        for (int i = 0; i < num_nodes; ++i) {
            uint16_t i_node = d_adj_matrix[start_idx + i];
            int soft_idx = i_node * lifting_size + tid;
            int msg_idx = (start_idx + i) * lifting_size + tid;

            float soft_val = d_soft_bits[soft_idx];
            float v2c_val;

            if (is_initialized) {
                v2c_val = soft_val - d_c2v_messages[msg_idx];
            } else {
                v2c_val = soft_val;
            }

            d_v2c_messages[msg_idx] = v2c_val;
            d_v2c_copy[msg_idx] = v2c_val; // Needed for final soft update
        }
    }
}

// Wrapper in CPU "seen" by .cpp and linker
extern "C" {
void ldpc_v2c_subtraction(
    float* d_soft_bits, float* d_c2v, float* d_v2c, float* d_v2c_copy,
    const uint16_t* d_adj, const uint8_t* d_offsets, const uint8_t* d_lengths,
    int lifting_size, int check_node, bool is_initialized) 
{
    // 1 thread per LS)
    // If LS <= 1024, use 1 block?
    dim3 block(lifting_size);
    dim3 grid(1);
    hipLaunchKernelGGL(ldpc_v2c_subtraction_kernel, grid, block, 0, 0, 
        d_soft_bits, d_c2v, d_v2c, d_v2c_copy, 
        d_adj, d_offsets, d_lengths, 
        lifting_size, check_node, is_initialized);
}
}


