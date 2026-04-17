#include <hip/hip_runtime.h>

// Scale factor (e.g., 0.75f), in ocudu is it LUT?
#define ALPHA 0.75f 
#define LLR_MAX 127.0f // According of type of data


__global__ void ldpc_layer_process_fused_kernel(
    float* d_soft_bits, float* d_c2v,
    const uint16_t* d_adj, const uint16_t* d_shifts,
    const uint8_t* d_offsets, const uint8_t* d_lengths,
    int lifting_size, int check_node, bool is_initialized) 
{
    int tid = hipThreadIdx_x; // 0 to LS-1
    if (tid >= lifting_size) return;

    int start_idx = d_offsets[check_node];
    int num_nodes = d_lengths[check_node];

    // In register, 32 for max degree of BG1 is 30
    float local_v2c[32]; 
    
    float min1 = 1e10f;
    float min2 = 1e10f;
    int min_idx = -1;
    float sign_prod = 1.0f;

    // Fetch from global memory
    for (int i = 0; i < num_nodes; ++i) {
        uint16_t i_node = d_adj[start_idx + i];
        uint16_t shift  = d_shifts[start_idx + i];
        
        // Correct indexing
        int rotated_tid = (tid + shift) % lifting_size;
        int soft_idx = i_node * lifting_size + tid; 
        int msg_idx  = (start_idx + i) * lifting_size + rotated_tid;

        float soft_val = d_soft_bits[soft_idx];
        float v2c_val = is_initialized ? (soft_val - d_c2v[msg_idx]) : soft_val;
        
        local_v2c[i] = v2c_val; 

        // Min-Sum logic
        float abs_val = fabsf(v2c_val);
        sign_prod *= (v2c_val >= 0.0f) ? 1.0f : -1.0f;

        if (abs_val < min1) {
            min2 = min1; min1 = abs_val; min_idx = i;
        } else if (abs_val < min2) {
            min2 = abs_val;
        }
    }

    // Calc and write back to global memory
    float scaled_min1 = min1 * 0.75f; 
    float scaled_min2 = min2 * 0.75f;

    for (int i = 0; i < num_nodes; ++i) {
        uint16_t i_node = d_adj[start_idx + i];
        uint16_t shift  = d_shifts[start_idx + i];
        int rotated_tid = (tid + shift) % lifting_size;
        
        int soft_idx = i_node * lifting_size + tid;
        int msg_idx  = (start_idx + i) * lifting_size + rotated_tid;

        float res_abs = (i == min_idx) ? scaled_min2 : scaled_min1;
        float current_sign = (local_v2c[i] >= 0.0f) ? 1.0f : -1.0f;
        float new_c2v = (sign_prod * current_sign) * res_abs;

        d_c2v[msg_idx] = new_c2v;
        d_soft_bits[soft_idx] = local_v2c[i] + new_c2v;
    }
}


extern "C" {
void ldpc_process_layer_fused(
    float* d_soft_bits, float* d_c2v,
    const uint16_t* d_adj, const uint16_t* d_shifts,
    const uint8_t* d_offsets, const uint8_t* d_lengths,
    int lifting_size, int check_node, bool is_initialized) 
{
    hipLaunchKernelGGL(ldpc_layer_process_fused_kernel, 
                       dim3(1), dim3(lifting_size), 0, 0, 
                       d_soft_bits, d_c2v, d_adj, d_shifts, d_offsets, d_lengths, 
                       lifting_size, check_node, is_initialized);
}
}
