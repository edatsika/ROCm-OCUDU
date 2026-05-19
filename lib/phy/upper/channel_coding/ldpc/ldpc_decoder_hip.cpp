// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be
// subject to additional licensing requirements.

#include "ldpc_decoder_impl.h" //?
#include "ldpc_decoder_hip.h"
#include "ldpc_luts_impl.h"
#include "ocudu/support/ocudu_assert.h"
#include <algorithm>
#include <iostream>

namespace ocudu {

using namespace ocudu::ldpc;

// =============================================================================
// Constructor
// =============================================================================
ldpc_decoder_hip::ldpc_decoder_hip(bool cfg_force_decoding, bool cfg_early_stop_syndrome) :
  ldpc_decoder_impl(cfg_force_decoding, cfg_early_stop_syndrome)
{
  CHECK_HIP(hipStreamCreate(&stream));

  // Pinned host buffer – largest possible codeword × BATCH_SIZE
  const size_t max_soft =
      static_cast<size_t>(ldpc::MAX_BG_N_FULL) * ldpc::MAX_LIFTING_SIZE * BATCH_SIZE;
  CHECK_HIP(hipHostMalloc(&h_interleaved_bits, max_soft * sizeof(int8_t)));
  CHECK_HIP(hipMalloc    (&d_soft_bits,        max_soft * sizeof(int8_t)));

  // c2v buffer: [layer][edge][Z][Batch]
  const size_t max_c2v =
      static_cast<size_t>(ldpc::MAX_BG_M) * MAX_NODES_PER_LAYER *
      ldpc::MAX_LIFTING_SIZE * BATCH_SIZE;
  CHECK_HIP(hipMalloc(&d_c2v, max_c2v * sizeof(int8_t)));

  // Per-codeword syndrome-check flags
  CHECK_HIP(hipMalloc(&d_valid_flags, BATCH_SIZE * sizeof(uint8_t)));
  h_valid_flags.resize(BATCH_SIZE, 0u);
}

// =============================================================================
// Destructor
// =============================================================================
ldpc_decoder_hip::~ldpc_decoder_hip()
{
  if (h_interleaved_bits) CHECK_HIP(hipHostFree(h_interleaved_bits));
  if (d_soft_bits)        CHECK_HIP(hipFree(d_soft_bits));
  if (d_c2v)              CHECK_HIP(hipFree(d_c2v));
  if (d_adj)              CHECK_HIP(hipFree(d_adj));
  if (d_shifts)           CHECK_HIP(hipFree(d_shifts));
  if (d_valid_flags)      CHECK_HIP(hipFree(d_valid_flags));
  if (stream)             CHECK_HIP(hipStreamDestroy(stream));
}


void ldpc_decoder_hip::upload_bg_to_device()
{
  // Use Z of base (ldpc_decoder_impl)
  const uint32_t Z = this->lifting_size;
  
  if (Z == 0) {
      std::cerr << "ERROR: lifting_size is 0! Check if init() was called correctly." << std::endl;
      return; 
  }

  if (nof_layers_gpu == 0) {
      std::cerr << "ERROR: nof_layers_gpu is 0!" << std::endl;
      return;
  }

  // Prepare vectors outside the loop
  h_adj_flat.assign(nof_layers_gpu * MAX_NODES_PER_LAYER, 255); // 255 = NO_EDGE
  h_shifts_flat.assign(nof_layers_gpu * MAX_NODES_PER_LAYER, 0);
  h_nof_edges_per_layer.assign(nof_layers_gpu, 0);

  for (uint32_t l = 0; l < nof_layers_gpu; ++l) {
    const BG_adjacency_row_t& adj_row = get_current_graph()->get_adjacency_row(l);
    
    uint32_t edge_count = 0;
    for (auto var_node_idx : adj_row) {
        if (var_node_idx == NO_EDGE || var_node_idx >= get_bg_N_high_rate()) {
            break;
        }
        
        if (edge_count >= MAX_NODES_PER_LAYER) {
            break;
        }

        h_adj_flat[l * MAX_NODES_PER_LAYER + edge_count] = static_cast<uint8_t>(var_node_idx);
        
        uint32_t raw_shift = get_current_graph()->get_lifted_node(l, var_node_idx);
        h_shifts_flat[l * MAX_NODES_PER_LAYER + edge_count] = static_cast<uint16_t>(raw_shift % Z);
        
        edge_count++;
    }
    h_nof_edges_per_layer[l] = static_cast<uint8_t>(edge_count);
  }

  // Free memory in GPU
  if (d_adj)    { CHECK_HIP(hipFree(d_adj));    d_adj    = nullptr; }
  if (d_shifts) { CHECK_HIP(hipFree(d_shifts)); d_shifts = nullptr; }

  CHECK_HIP(hipMalloc(&d_adj,    h_adj_flat.size()    * sizeof(uint8_t)));
  CHECK_HIP(hipMalloc(&d_shifts, h_shifts_flat.size() * sizeof(uint16_t)));

  CHECK_HIP(hipMemcpy(d_adj,    h_adj_flat.data(),
                      h_adj_flat.size()    * sizeof(uint8_t),    hipMemcpyHostToDevice));
  CHECK_HIP(hipMemcpy(d_shifts, h_shifts_flat.data(),
                      h_shifts_flat.size() * sizeof(uint16_t),  hipMemcpyHostToDevice));

  std::cout << "--- HIP GPU Init [BG="
            << (current_cfg.base_graph == ldpc_base_graph_type::BG1 ? "1" : "2")
            << " LS=" << Z
            << " layers=" << nof_layers_gpu
            << " batch=" << BATCH_SIZE << "] ---\n";
}


// =============================================================================
// interleave()
//
// Packs BATCH_SIZE codewords from per-codeword AoS layout into the
// [node][Z][Batch] SoA layout required for coalesced GPU reads.
// =============================================================================
void ldpc_decoder_hip::interleave(const std::vector<span<const int8_t>>& inputs)
{
  const uint32_t Z = current_cfg.lifting_size;
  const uint32_t B = BATCH_SIZE;

  for (uint32_t b = 0; b < static_cast<uint32_t>(inputs.size()); ++b) {
    for (uint32_t n = 0; n < nof_nodes_gpu; ++n) {
      for (uint32_t z = 0; z < Z; ++z) {
        const size_t src  = n * Z + z;
        const size_t dest = ((size_t)n * Z + z) * B + b;
        h_interleaved_bits[dest] =
            (src < inputs[b].size()) ? inputs[b][src] : int8_t{0};
      }
    }
  }
}

// =============================================================================
// deinterleave()
//
// Extracts the K info-bit nodes from the interleaved GPU result back into
// per-codeword output spans.
// =============================================================================
void ldpc_decoder_hip::deinterleave(std::vector<span<int8_t>>& outputs)
{
  const uint32_t Z = current_cfg.lifting_size;
  const uint32_t B = BATCH_SIZE;

  for (uint32_t b = 0; b < static_cast<uint32_t>(outputs.size()); ++b) {
    for (uint32_t n = 0; n < K_nodes_gpu; ++n) {
      for (uint32_t z = 0; z < Z; ++z) {
        const size_t src  = ((size_t)n * Z + z) * B + b;
        const size_t dest = n * Z + z;
        if (dest < outputs[b].size())
          outputs[b][dest] = h_interleaved_bits[src];
      }
    }
  }
}

// =============================================================================
// decode()
//
// Overrides ldpc_decoder_impl::decode() so the CPU layered path is never
// entered for this subclass.  Pads the single codeword to a full batch of
// BATCH_SIZE and delegates to decode_batch().
// =============================================================================
std::optional<unsigned> ldpc_decoder_hip::decode(
    bit_buffer&                      output,
    span<const log_likelihood_ratio> input,
    crc_calculator*                  /*crc*/,
    const configuration&             cfg)
{
  // Build a one-codeword batch, padding with zeros to fill BATCH_SIZE
  const std::vector<int8_t> cw_in (input.data(), input.data() + input.size());
  const std::vector<int8_t> zero_in(input.size(), 0);
  std::vector<int8_t>       cw_out(output.size(), 0);
  const std::vector<int8_t> zero_out_data(output.size(), 0);

  std::vector<span<const int8_t>> in_spans;
  std::vector<span<int8_t>>       out_spans;
  in_spans.reserve(BATCH_SIZE);
  out_spans.reserve(BATCH_SIZE);

  in_spans.push_back(span<const int8_t>(cw_in));
  out_spans.push_back(span<int8_t>(cw_out));
  for (uint32_t i = 1; i < BATCH_SIZE; ++i) {
    in_spans.push_back(span<const int8_t>(zero_in));
    // Each padding slot needs its own writable buffer; we reuse zero_out_data
    // for all of them (they are discarded)
    out_spans.push_back(span<int8_t>(const_cast<int8_t*>(zero_out_data.data()),
                                     zero_out_data.size()));
  }

  // Update config for GPU path
  current_cfg         = cfg;
  nof_layers_gpu = (cfg.base_graph == ldpc_base_graph_type::BG1) ? 46u : 42u;
  nof_nodes_gpu  = (cfg.base_graph == ldpc_base_graph_type::BG1) ? 68u : 52u;
  K_nodes_gpu    = (cfg.base_graph == ldpc_base_graph_type::BG1) ? 22u : 10u;

  decode_batch(out_spans, in_spans);

  // Hard decision: negative LLR → bit = 1
  for (size_t i = 0; i < output.size(); ++i)
    output.insert((cw_out[i] < 0) ? 1u : 0u, i, 1);

  return cfg.max_iterations;
}

void ldpc_decoder_hip::decode_batch(std::vector<span<int8_t>>&             outputs,
                                     const std::vector<span<const int8_t>>& inputs)
{
  ocudu_assert(!inputs.empty() && inputs.size() <= BATCH_SIZE,
               "batch_size {} must be in [1, {}]", inputs.size(), BATCH_SIZE);
  ocudu_assert(outputs.size() == inputs.size(),
               "outputs.size() {} != inputs.size() {}", outputs.size(), inputs.size());

  if (d_adj == nullptr || this->lifting_size == 0) {
      throw std::runtime_error("GPU Decoder not initialized. Call prepare_gpu() first.");
  }

  const uint32_t Z          = this->lifting_size;
  const uint32_t batch_size = static_cast<uint32_t>(inputs.size());
  const size_t   soft_elems = static_cast<size_t>(nof_nodes_gpu) * Z * BATCH_SIZE;


  // Pack inputs into [node][Z][Batch]
  interleave(inputs);

  // Transfer to GPU and zero c2v messages
  CHECK_HIP(hipMemcpyAsync(d_soft_bits, h_interleaved_bits,
                           soft_elems * sizeof(int8_t),
                           hipMemcpyHostToDevice, stream));

  const size_t c2v_elems =
      static_cast<size_t>(nof_layers_gpu) * MAX_NODES_PER_LAYER * Z * BATCH_SIZE;
  CHECK_HIP(hipMemsetAsync(d_c2v, 0, c2v_elems * sizeof(int8_t), stream));


  // Layered decoding iterations
  const uint32_t max_iter = current_cfg.max_iterations;
  const bool     do_early = get_early_stop();

  for (uint32_t iter = 0; iter < max_iter; ++iter) {
    // Sequential layers loop
    for (uint32_t l = 0; l < nof_layers_gpu; ++l) {
      ldpc_process_layer_fused_batch(
          stream,
          d_soft_bits, 
          d_c2v,
          d_shifts, 
          d_adj,
          (int)h_nof_edges_per_layer[l], 
          (int)l,                        // Layer index
          (int)Z,                        // LS
          scaling_factor);
    }

    // Early Stop Check
    if (do_early && iter >= 2 && (iter % 2 == 0)) {
      // Flags (0 = invalid, 1 = valid)
      // Syndrome kernel should write 1 if parity is OK
      CHECK_HIP(hipMemsetAsync(d_valid_flags, 0, BATCH_SIZE * sizeof(uint8_t), stream));

      ldpc_syndrome_check_batch(
          stream,
          d_soft_bits, d_adj, d_shifts,
          d_valid_flags,
          nof_layers_gpu, Z, batch_size);

      CHECK_HIP(hipMemcpyAsync(h_valid_flags.data(), d_valid_flags,
                               batch_size * sizeof(uint8_t),
                               hipMemcpyDeviceToHost, stream));
      
      // Wait for syndrome check
      CHECK_HIP(hipStreamSynchronize(stream));

      bool all_done = true;
      for (uint32_t b = 0; b < batch_size; ++b) {
        if (!h_valid_flags[b]) { 
            all_done = false; 
            break; 
        }
      }
      if (all_done) break;
    }
  }

  // Transfer results back and unpack
  CHECK_HIP(hipMemcpyAsync(h_interleaved_bits, d_soft_bits,
                           soft_elems * sizeof(int8_t),
                           hipMemcpyDeviceToHost, stream));
  CHECK_HIP(hipStreamSynchronize(stream));

  deinterleave(outputs);
}



//Called by prepare_gpu
void ldpc_decoder_hip::init(const configuration& cfg) 
{
    ldpc_decoder_impl::init(cfg); 

    this->current_cfg = cfg;

    this->nof_layers_gpu = (cfg.base_graph == ldpc_base_graph_type::BG1) ? 46u : 42u;
    this->nof_nodes_gpu  = (cfg.base_graph == ldpc_base_graph_type::BG1) ? 68u : 52u;

    upload_bg_to_device(); 

    if (!stream) {
        hipStreamCreate(&stream);
    }
}

} // namespace ocudu
