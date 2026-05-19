// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be
// subject to additional licensing requirements.

/// \file
/// \brief LDPC decoder – declaration of the HIP/GPU batched implementation.
///
/// Follows exactly the same subclass pattern as ldpc_decoder_avx512:
///   - Inherits ldpc_decoder_impl (the CPU base class)
///   - Overrides specific_init()  (no CPU state needed for GPU path)
///   - Overrides decode()         (GPU single-codeword via decode_batch)
///   - Adds    decode_batch()     (main GPU interface, BATCH_SIZE codewords)
///   - Provides unreachable stubs for the CPU pure-virtual hooks so the
///     class is concrete (they abort if somehow called)
///
/// The GPU buffers and HIP resources are private to this subclass and are
/// completely separate from the CPU-path buffers in ldpc_decoder_impl.

#pragma once

#include "ldpc_decoder_impl.h"
#include <hip/hip_runtime.h>
#include <cstdint>
#include <vector>

// Convenience macro used only within this translation unit
#define CHECK_HIP(cmd)                                                                            \
  {                                                                                               \
    hipError_t _e = (cmd);                                                                        \
    if (_e != hipSuccess) {                                                                       \
      fprintf(stderr, "HIP error: %s at %s:%d\n", hipGetErrorString(_e), __FILE__, __LINE__);    \
      exit(EXIT_FAILURE);                                                                         \
    }                                                                                             \
  }

namespace ocudu {


// Kernel C wrappers (defined in batched_fused_kernel.hip and ldpc_syndrome_check_kernel.hip)
#ifdef __cplusplus
extern "C" {
#endif

/// Layered min-sum kernel – one check node, all BATCH_SIZE codewords.
void ldpc_process_layer_fused_batch(
    hipStream_t     stream,
    int8_t*         d_soft_bits,    ///< [node][Z][Batch]
    int8_t*         d_c2v,          ///< [layer][edge][Z][Batch]
    const uint16_t* d_shifts,       ///< [layer][edge]
    const uint8_t*  d_adj,          ///< [layer][edge], 255 = sentinel
    uint8_t         nof_edges,      ///< degree of this check node
    int             layer_idx,
    int             lifting_size,
    float           scaling_factor);

/// Syndrome check – sets d_valid_flags[b]=1 when codeword b is valid.
void ldpc_syndrome_check_batch(
    hipStream_t     stream,
    const int8_t*   d_soft_bits,
    const uint8_t*  d_adj,
    const uint16_t* d_rev_shifts,
    uint8_t*        d_valid_flags,
    uint32_t        nof_layers,
    uint32_t        lifting_size,
    uint32_t        batch_size);

#ifdef __cplusplus
}
#endif

// ldpc_decoder_hip
class ldpc_decoder_hip : public ldpc_decoder_impl
{
public:
  // GPU constants – must match the #defines in both .hip files
  static constexpr uint32_t BATCH_SIZE          = 256;
  static constexpr uint32_t MAX_NODES_PER_LAYER = 19;

  // Constructor / destructor
  ldpc_decoder_hip(bool cfg_force_decoding, bool cfg_early_stop_syndrome);
  ~ldpc_decoder_hip() override;

  /// Single-codeword GPU decode (wraps decode_batch with a padded batch).
  /// Overrides ldpc_decoder_impl::decode() so the CPU layered path is never
  /// entered for this subclass.
  std::optional<unsigned> decode(bit_buffer&                      output,
                                 span<const log_likelihood_ratio> input,
                                 crc_calculator*                  crc,
                                 const configuration&             cfg) override;

  /// Batched GPU decode – the primary interface for this subclass.
  /// Decodes up to BATCH_SIZE codewords in a single GPU pass.
  /// @param outputs  one span<int8_t> per codeword – written with decoded LLRs
  /// @param inputs   one span<const int8_t> per codeword – received channel LLRs
  void decode_batch(std::vector<span<int8_t>>&             outputs,
                    const std::vector<span<const int8_t>>& inputs);
 
  void prepare_gpu(const configuration& cfg) {
    // Call init of base (ldpc_decoder_impl) 
    // Fill: this->lifting_size, bg_K, bg_M, current_graph
    this->ldpc_decoder_impl::init(cfg); 

    // Update local current_cfg of ldpc_decoder_hip
    this->current_cfg = cfg;

    // Set GPU dimensions
    this->nof_layers_gpu = (cfg.base_graph == ldpc_base_graph_type::BG1) ? 46 : 42;
    this->nof_nodes_gpu  = (cfg.base_graph == ldpc_base_graph_type::BG1) ? 68 : 52;
    this->K_nodes_gpu    = (cfg.base_graph == ldpc_base_graph_type::BG1) ? 22 : 10;

printf("[DEBUG GPU] Base Graph: %s\n", (cfg.base_graph == ldpc_base_graph_type::BG1 ? "BG1" : "BG2"));
printf("[DEBUG GPU] nof_nodes_gpu (Allocated): %u\n", this->nof_nodes_gpu);
printf("[DEBUG GPU] nof_layers_gpu: %u\n", this->nof_layers_gpu);
printf("[DEBUG GPU] Lifting Size (Z): %u\n", this->lifting_size);
printf("[DEBUG GPU] Batch Size: %d\n", BATCH_SIZE);

size_t total_elements = (size_t)nof_nodes_gpu * lifting_size * BATCH_SIZE;
printf("[DEBUG GPU] Total Allocation (Soft Bits): %zu elements\n", total_elements);


    this->upload_bg_to_device(); 
}

private:
  // specific_init() – called once per BG/LS change at the end of the
  // base-class init()  Nothing to do for the GPU path.

  void specific_init() override {}
  void init(const configuration& cfg) override;

  // CPU pure-virtual stubs – unreachable in the GPU path because decode()
  // is fully overridden above.  They abort immediately if somehow reached
  // (e.g. accidental direct call to the base-class decode()).
  void analyze_var_to_check_msgs(span<log_likelihood_ratio>,
                                 span<log_likelihood_ratio>,
                                 span<uint8_t>,
                                 span<uint8_t>,
                                 span<const log_likelihood_ratio>,
                                 unsigned) override
  { ocudu_assert(false, "analyze_var_to_check_msgs called on GPU decoder"); }

  void scale(span<log_likelihood_ratio>,
             span<const log_likelihood_ratio>) override
  { ocudu_assert(false, "scale called on GPU decoder"); }

  void compute_check_to_var_msgs(span<log_likelihood_ratio>,
                                 span<const log_likelihood_ratio>,
                                 span<const log_likelihood_ratio>,
                                 span<const log_likelihood_ratio>,
                                 span<const log_likelihood_ratio>,
                                 span<const uint8_t>,
                                 span<const uint8_t>,
                                 unsigned,
                                 unsigned) override
  { ocudu_assert(false, "compute_check_to_var_msgs called on GPU decoder"); }

  void compute_soft_bits(span<log_likelihood_ratio>,
                         span<const log_likelihood_ratio>,
                         span<const log_likelihood_ratio>) override
  { ocudu_assert(false, "compute_soft_bits called on GPU decoder"); }

  void compute_var_to_check_msgs(span<log_likelihood_ratio>,
                                 span<const log_likelihood_ratio>,
                                 span<const log_likelihood_ratio>) override
  { ocudu_assert(false, "compute_var_to_check_msgs called on GPU decoder"); }

  // GPU H-matrix upload and data layout helpers
  void upload_bg_to_device();
  void interleave  (const std::vector<span<const int8_t>>& inputs);
  void deinterleave(std::vector<span<int8_t>>& outputs);

  // GPU configuration 
  configuration current_cfg{};
  uint32_t nof_layers_gpu = 0;   //bg_M
  uint32_t nof_nodes_gpu  = 0;   //bg_N_full
  uint32_t K_nodes_gpu    = 0;   //bg_K info nodes only

  /// Per-layer check-node degree, built by upload_bg_to_device()
  std::vector<uint8_t> h_nof_edges_per_layer;

  // HIP resources
  hipStream_t stream             = nullptr;
  int8_t*     h_interleaved_bits = nullptr;  //pinned host buffer [node][Z][Batch]

  // GPU device buffers
  int8_t*   d_soft_bits   = nullptr;  //[node][Z][Batch]
  int8_t*   d_c2v         = nullptr;  //[layer][edge][Z][Batch]
  uint8_t*  d_adj         = nullptr;  //[layer][edge] variable node indices
  uint16_t* d_shifts      = nullptr;  //[layer][edge] cyclic shifts
  uint8_t*  d_valid_flags = nullptr;  //[Batch] = 1 when codeword passes syndrome

  std::vector<uint8_t> h_valid_flags;  // host copy used for early-stop decision
  
  std::vector<uint8_t>  h_adj_flat;
  std::vector<uint16_t> h_shifts_flat;

};

} // namespace ocudu


  

