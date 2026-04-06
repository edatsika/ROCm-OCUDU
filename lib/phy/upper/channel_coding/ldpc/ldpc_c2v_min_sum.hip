#include <hip/hip_runtime.h>

__global__ void ldpc_c2v_min_sum_kernel(
    float* __restrict__ d_v2c,
    float* __restrict__ d_c2v,
    const uint16_t* __restrict__ d_adj,
    const uint16_t* __restrict__ d_shifts,
    const uint8_t* __restrict__ d_offsets,
    const uint8_t* __restrict__ d_lengths,
    int lifting_size,
    int check_node,
    int max_msg_idx) 
{
    int tid = hipThreadIdx_x;
    if (tid >= lifting_size || lifting_size == 0) return;

    int start_idx = d_offsets[check_node];
    int num_nodes = d_lengths[check_node];

    // Registers for mins and sign
    float min1 = 127.0f;
    float min2 = 127.0f;
    int min_idx = -1;
    float sign_prod = 1.0f;

    // Registers to avoid reading from VRAM in Step 2
    // MAX_CHECK_NODE_DEGREE = 27 for BG1
    float local_vals[32]; 
    int local_indices[32];

    // STEP 1: Read and find Mins
    for (int i = 0; i < num_nodes && i < 32; ++i) {
        int edge_ptr = start_idx + i;
        uint16_t shift = d_shifts[edge_ptr];
        int rotated_tid = (tid + shift) % lifting_size;
        int msg_idx = edge_ptr * lifting_size + rotated_tid;

        if (msg_idx < max_msg_idx) {
            float val = d_v2c[msg_idx];
            local_vals[i] = val; // Store for Step 2
            local_indices[i] = msg_idx;

            float abs_val = fabsf(val);
            float s = (val >= 0.0f) ? 1.0f : -1.0f;
            sign_prod *= s;

            if (abs_val < min1) {
                min2 = min1;
                min1 = abs_val;
                min_idx = i;
            } else if (abs_val < min2) {
                min2 = abs_val;
            }
        }
    }

    // Step 2: Compute & Write Back
    float scaled_min1 = min1 * 0.75f;
    float scaled_min2 = min2 * 0.75f;

    for (int i = 0; i < num_nodes && i < 32; ++i) {
        float val = local_vals[i];
        float res_abs = (i == min_idx) ? scaled_min2 : scaled_min1;
        float current_sign = (val >= 0.0f) ? 1.0f : -1.0f;
        d_c2v[local_indices[i]] = (sign_prod * current_sign) * res_abs;
    }
}

extern "C" void ldpc_c2v_min_sum(
    float* d_v2c, 
    float* d_c2v, 
    const uint16_t* d_adj, 
    const uint16_t* d_shifts,
    const uint8_t* d_offsets, 
    const uint8_t* d_lengths,
    int lifting_size, 
    int check_node, 
    int total_edges) 
{
    if (lifting_size <= 0) return;

    int max_msg_idx = total_edges * lifting_size;

    hipLaunchKernelGGL(ldpc_c2v_min_sum_kernel, 
                       dim3(1), 
                       dim3(lifting_size), 
                       0, 0, 
                       d_v2c, d_c2v, d_adj, d_shifts, d_offsets, d_lengths, 
                       lifting_size, check_node, max_msg_idx);
}

