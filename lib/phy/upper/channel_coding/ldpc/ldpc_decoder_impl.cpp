// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "ldpc_decoder_impl.h"
#include "ldpc_luts_impl.h"
#include "ocudu/ocuduvec/binary.h"
#include "ocudu/ocuduvec/circ_shift.h"
#include "ocudu/ocuduvec/copy.h"
#include "ocudu/ocuduvec/fill.h"
#include "ocudu/ocuduvec/zero.h"
#include "ocudu/support/ocudu_assert.h"
#include <hip/hip_runtime.h>
#include <iostream>

// edatsika
#define CHECK_HIP(cmd) \
{ \
    hipError_t error = cmd; \
    if (error != hipSuccess) { \
        fprintf(stderr, "HIP Error: %s at %s:%d\n", hipGetErrorString(error), __FILE__, __LINE__); \
        exit(EXIT_FAILURE); \
    } \
}

using namespace ocudu;
using namespace ocudu::ldpc;

void ldpc_decoder_impl::init(const configuration& cfg)
{
  uint8_t  pos   = get_lifting_size_position(cfg.lifting_size);
  unsigned skip  = (cfg.base_graph == ldpc_base_graph_type::BG2) ? NOF_LIFTING_SIZES : 0;
  current_graph  = &graph_array[skip + pos];
  bg_N_full      = current_graph->get_nof_BG_var_nodes_full();
  bg_N_short     = current_graph->get_nof_BG_var_nodes_short();
  bg_M           = current_graph->get_nof_BG_check_nodes();
  bg_K           = current_graph->get_nof_BG_info_nodes();
  bg_N_high_rate = bg_K + 4;
  ocudu_assert(bg_K == bg_N_full - bg_M, "Invalid bg_K value '{}'", bg_K);
  
  uint16_t new_lifting_size = static_cast<uint16_t>(cfg.lifting_size);

  max_iterations = cfg.max_iterations;
  ocudu_assert(max_iterations > 0, "Max iterations must be different to 0");

  unsigned nof_crc_bits = cfg.nof_crc_bits;
  ocudu_assert((nof_crc_bits == 16) || (nof_crc_bits == 24), "Invalid number of CRC bits.");

  nof_significant_bits = bg_K * new_lifting_size - cfg.nof_filler_bits;

  specific_init();

  // --- edatsika GPU RE-INITIALIZATION ONLY ON CHANGE ---
  // Have LS or BG changed?
  if (new_lifting_size != lifting_size || cfg.base_graph != last_bg) {
      
      if (d_soft_bits != nullptr) {
            CHECK_HIP(hipFree(d_soft_bits)); d_soft_bits = nullptr;
            CHECK_HIP(hipFree(d_c2v));       d_c2v = nullptr;
            CHECK_HIP(hipFree(d_v2c));       d_v2c = nullptr;
            CHECK_HIP(hipFree(d_v2c_copy));  d_v2c_copy = nullptr;
            CHECK_HIP(hipFree(d_adj_matrix)); d_adj_matrix = nullptr;
            CHECK_HIP(hipFree(d_row_offsets)); d_row_offsets = nullptr;
            CHECK_HIP(hipFree(d_row_lengths)); d_row_lengths = nullptr;
            CHECK_HIP(hipFree(d_shifts));      d_shifts = nullptr;
      }

      // Update pars
      lifting_size = new_lifting_size;
      last_bg      = cfg.base_graph;

      // Prepare data at host
      total_edges = 0;
      std::vector<uint8_t> h_offsets;
      std::vector<uint8_t> h_lengths;
      h_adj_data.clear();
      h_shifts.clear();

      for (unsigned i = 0; i < bg_M; ++i) {
          const auto& row = current_graph->get_adjacency_row(i);
          h_offsets.push_back(static_cast<uint8_t>(h_adj_data.size()));
          
          unsigned row_len = 0;
          for (auto node_idx : row) {
              if (node_idx == NO_EDGE || node_idx >= bg_N_high_rate) break;
              h_adj_data.push_back(static_cast<uint16_t>(node_idx));
    
              // Adapt shift to current Z
              //Check next 2 lines
              //uint16_t raw_shift = current_graph->get_lifted_node(i, node_idx);
              //h_shifts.push_back(raw_shift % lifting_size);
              h_shifts.push_back(current_graph->get_lifted_node(i, node_idx) % lifting_size);
              row_len++;
          }
          h_lengths.push_back(static_cast<uint8_t>(row_len));
          total_edges += row_len;
      }

      // New GPU allocations
      CHECK_HIP(hipMalloc(&d_soft_bits, (bg_N_full * lifting_size) * sizeof(float)));
      CHECK_HIP(hipMalloc(&d_c2v, (total_edges * lifting_size) * sizeof(float)));
      CHECK_HIP(hipMalloc(&d_v2c, (total_edges * lifting_size) * sizeof(float)));
      CHECK_HIP(hipMalloc(&d_v2c_copy, (total_edges * lifting_size) * sizeof(float)));

      CHECK_HIP(hipMalloc(&d_adj_matrix, h_adj_data.size() * sizeof(uint16_t)));
      CHECK_HIP(hipMemcpy(d_adj_matrix, h_adj_data.data(), h_adj_data.size() * sizeof(uint16_t), hipMemcpyHostToDevice));

      CHECK_HIP(hipMalloc(&d_row_offsets, h_offsets.size() * sizeof(uint8_t)));
      CHECK_HIP(hipMemcpy(d_row_offsets, h_offsets.data(), h_offsets.size() * sizeof(uint8_t), hipMemcpyHostToDevice));

      CHECK_HIP(hipMalloc(&d_row_lengths, h_lengths.size() * sizeof(uint8_t)));
      CHECK_HIP(hipMemcpy(d_row_lengths, h_lengths.data(), h_lengths.size() * sizeof(uint8_t), hipMemcpyHostToDevice));

      CHECK_HIP(hipMalloc(&d_shifts, h_shifts.size() * sizeof(uint16_t)));
      CHECK_HIP(hipMemcpy(d_shifts, h_shifts.data(), h_shifts.size() * sizeof(uint16_t), hipMemcpyHostToDevice));

      std::cout << "--- GPU Re-Init [BG=" << (cfg.base_graph == ldpc_base_graph_type::BG1 ? "1":"2") 
                << ", LS=" << lifting_size << "] ---" << std::endl;
        
        printf("DEBUG: LS=%d, total_edges=%u, h_adj_data.size=%zu\n", 
        lifting_size, total_edges, h_adj_data.size());
        size_t msg_bytes = (size_t)total_edges * lifting_size * sizeof(float);
        printf("DEBUG MALLOC: total_edges=%u, LS=%u, bytes=%zu\n", total_edges, lifting_size, msg_bytes);
        printf("DEBUG: bg_M=%u, h_offsets.size=%zu, h_lengths.size=%zu\n", 
        bg_M, h_offsets.size(), h_lengths.size());

  }
  
}


std::optional<unsigned> ldpc_decoder_impl::decode(bit_buffer&                      output,
                                                  span<const log_likelihood_ratio> input,
                                                  crc_calculator*                  crc,
                                                  const configuration&             cfg)
{
  init(cfg);

  uint16_t message_length   = bg_K * lifting_size;
  uint16_t max_input_length = bg_N_short * lifting_size;
  ocudu_assert(output.size() == message_length,
               "The output size {} is not equal to the message length {}.",
               output.size(),
               message_length);
  ocudu_assert(input.size() <= max_input_length,
               "The input size {} exceeds the maximum message length {}.",
               input.size(),
               max_input_length);

  // The minimum input length is message_length + two times the lifting size.
  uint16_t min_input_length = message_length + 2 * lifting_size;
  ocudu_assert(input.size() >= min_input_length,
               "The input length {} does not reach minimum {}",
               input.size(),
               min_input_length);

  // Find the last soft bit in the buffer and trim the output.
  const log_likelihood_ratio* last =
      std::find_if(input.rbegin(), input.rend(), [](const log_likelihood_ratio& in) { return in != 0; }).base();

  // Determine input length.
  unsigned input_size = std::distance(input.begin(), last);

  // The input meaningful number of bits must contain the msg length number of bits to decode the codeblock
  if ((input_size < message_length) && force_decoding) {
    // If the codeblock CRC check is external, set all bits to one (so that the CRC will fail)
    if (crc == nullptr) {
      output.one();
    }
    return std::nullopt;
  }

  // Ensure c2v msgs are not initialized
  //printf("DEBUG: Before std::fill\n");
  std::fill(is_check_to_var_initialized.begin(), is_check_to_var_initialized.end(), false);
  //printf("DEBUG: After std::fill\n");

  load_soft_bits(input, input_size);
  printf("DEBUG: After load_soft_bits\n");

  // The minimum codeblock length is message_length + four times the lifting size
  // (that is, the length of the high-rate region).
  uint16_t min_codeblock_length = message_length + 4 * lifting_size;
  // The decoder works with at least min_codeblock_length bits. The encoder also shortens
  // the codeblock by 2 * lifting size before returning it as output
  codeblock_length = std::max(input_size + 2UL * lifting_size, static_cast<size_t>(min_codeblock_length));
  // The decoder works with a codeblock length that is a multiple of the lifting size.
  if (codeblock_length % lifting_size != 0) {
    codeblock_length = (codeblock_length / lifting_size + 1) * lifting_size;
  }

  // Layered LDPC decoding: Layers depend on the previous layer but within a layer, many operations are parallel.
  // GPU parallelization should focus inside each layer, not across layers.

 // GENERAL DECODING LOOP
  unsigned nof_layers = codeblock_length / lifting_size - bg_K;
  
  //edatsika send LLRs to GPU
  size_t safe_copy_size = (bg_N_full * lifting_size) * sizeof(float);
  printf("--- Copy Debug ---\n");
  printf("d_soft_bits: %p\n", (void*)d_soft_bits);
  printf("soft_bits.data(): %p\n", (void*)soft_bits.data());
  printf("safe_copy_size(): %zu\n", safe_copy_size);
  //printf("Total bytes: %zu\n", soft_bits.size() * sizeof(float));
  
  // Calc bits related to current LS only
  //to put back: size_t current_ls_bytes = (bg_N_full * lifting_size) * sizeof(float);
  //Boundary check to be removed
  size_t current_ls_bytes = (bg_N_full * lifting_size) * sizeof(log_likelihood_ratio);
if (current_ls_bytes > soft_bits.size() * sizeof(log_likelihood_ratio)) {
    printf("FATAL: current_ls_bytes (%zu) > soft_bits capacity (%zu)\n", 
            current_ls_bytes, soft_bits.size() * sizeof(log_likelihood_ratio));
    return std::nullopt;
}

  CHECK_HIP(hipMemcpy(d_soft_bits, soft_bits.data(), current_ls_bytes, hipMemcpyHostToDevice));

  for (unsigned i_iteration = 0; i_iteration != max_iterations; ++i_iteration) {
    // Run all layers
    for (unsigned i_layer = 0; i_layer != nof_layers; ++i_layer) {
      update_variable_to_check_messages(i_layer);
      //CHECK_HIP(hipDeviceSynchronize());

      update_check_to_variable_messages(i_layer);
      //CHECK_HIP(hipDeviceSynchronize());

      update_soft_bits(i_layer);
      //CHECK_HIP(hipDeviceSynchronize());
    }

  //edatsika bring back results for CRC check
  CHECK_HIP(hipMemcpy(soft_bits.data(), d_soft_bits, current_ls_bytes, hipMemcpyDeviceToHost));

printf("--- GPU Return Debug [LS=%d] ---\n", lifting_size);
// Cast to read bytes directly
int8_t* raw_ptr = reinterpret_cast<int8_t*>(soft_bits.data());

for (int i = 0; i < 16; ++i) {
    printf("%d ", (int)raw_ptr[i]); // Print int LLR (-128 to 127)
}
printf("\n-------------------------------\n");

    // If a CRC calculator was passed with the configuration parameters
    if (crc != nullptr) {
      // Get hard bits
      bool hard_bits_success = get_hard_bits(output);

      // Early stop condition: CRC check must be zero
      if (hard_bits_success && crc->calculate(output.first(nof_significant_bits)) == 0) {
        return i_iteration + 1;
      }
    } else if (early_stop_syndrome) {
      // Get hard bits
      bool hard_bits_success = get_hard_bits(output);

      // Early stop condition: check syndrome
      if (hard_bits_success && check_syndrome()) {
        return i_iteration + 1;
      }
    }
  }

  // Skip any further decisions if an early stop condition is configured
  if ((crc != nullptr) || early_stop_syndrome) {
    return std::nullopt;
  }

  // Get current hard bits
  bool hard_bits_success = get_hard_bits(output);

  // Check syndrome for determining if the codeblock decoding is successful
  if (!hard_bits_success || !check_syndrome()) {
    return std::nullopt;
  }

  return max_iterations;
}

void ldpc_decoder_impl::load_soft_bits(span<const log_likelihood_ratio> llrs, unsigned nof_llr)
{
  if (soft_bits.size() == 0) {
      soft_bits.resize(ldpc::MAX_BG_N_FULL * ldpc::MAX_LIFTING_SIZE);
  }

  std::fill(soft_bits.begin(), soft_bits.end(), log_likelihood_ratio{0});

  unsigned nof_full_nodes = llrs.size() / lifting_size + 2;
  span<const log_likelihood_ratio> llr_view = llrs;
  
  span<log_likelihood_ratio> soft_bits_view(soft_bits.data(), soft_bits.size());

  // First 2 nodes always 0 (shortened)
  if (soft_bits_view.size() >= 2 * node_size_byte) {
      ocuduvec::zero(soft_bits_view.first(2 * node_size_byte));
      soft_bits_view = soft_bits_view.last(soft_bits_view.size() - 2 * node_size_byte);
  }

  for (unsigned i_node = 2, max_node = nof_full_nodes; i_node < max_node; ++i_node) {
    
    if (soft_bits_view.size() < node_size_byte) break; // Safety break

    if (nof_llr != 0) {
      clamp(soft_bits_view.first(lifting_size), llr_view.first(std::min((size_t)lifting_size, llr_view.size())), soft_bits_clamp_low, soft_bits_clamp_high);
    } else {
      ocuduvec::zero(soft_bits_view.first(lifting_size));
    }

    if (llr_view.size() >= lifting_size) {
        llr_view = llr_view.last(llr_view.size() - lifting_size);
    }
    nof_llr = (nof_llr >= lifting_size) ? (nof_llr - lifting_size) : 0;

    // Zero tail of node
    if (node_size_byte > lifting_size) {
        ocuduvec::zero(soft_bits_view.subspan(lifting_size, node_size_byte - lifting_size));
    }

    soft_bits_view = soft_bits_view.last(soft_bits_view.size() - node_size_byte);
  }

  // Last few bits
  unsigned tail_positions = llr_view.size();
  if (tail_positions != 0 && soft_bits_view.size() >= tail_positions) {
    ocuduvec::copy(soft_bits_view.first(tail_positions), llr_view);
    if (node_size_byte > tail_positions) {
        ocuduvec::zero(soft_bits_view.subspan(tail_positions, node_size_byte - tail_positions));
    }
  }
  
  printf("DEBUG: load_soft_bits finished successfully\n");
}

// edatsika replace with kernel, this is after extern c
void ldpc_decoder_impl::update_variable_to_check_messages(unsigned i_layer) 
{
    ldpc_v2c_subtraction(d_soft_bits, 
                           d_c2v, 
                           d_v2c, 
                           d_v2c_copy, 
                           d_adj_matrix, 
                           d_row_offsets, 
                           d_row_lengths, 
                           lifting_size, 
                           i_layer, 
                           is_check_to_var_initialized[i_layer]);
}

// edatsika  this is after extern c
void ldpc_decoder_impl::update_soft_bits(unsigned i_layer) 
{
    ldpc_soft_bits_update(d_soft_bits, 
                            d_v2c_copy, 
                            d_c2v, 
                            d_adj_matrix, 
                            d_shifts, 
                            d_row_offsets, 
                            d_row_lengths, 
                            lifting_size, 
                            i_layer);
}


// edatsika this is after extern c
void ldpc_decoder_impl::update_check_to_variable_messages(unsigned i_layer) 
{
    ldpc_c2v_min_sum(d_v2c, 
                     d_c2v, 
                     d_adj_matrix, 
                     d_shifts, 
                     d_row_offsets, 
                     d_row_lengths, 
                     lifting_size, 
                     i_layer,
                     total_edges); // new
    
    is_check_to_var_initialized[i_layer] = true;
}


bool ldpc_decoder_impl::get_hard_bits(bit_buffer& out) const
{
  if (lifting_size == node_size_byte) {
    span<const log_likelihood_ratio> llrs = span<const log_likelihood_ratio>(soft_bits).first(out.size());
    return hard_decision(out, llrs);
  }

  // Perform hard-decision of the LLRs from the soft_bits array directly into the output without any padding
  bool valid = true;
  for (unsigned i_node = 0; i_node != bg_K; ++i_node) {
    // View over the LLR
    span<const log_likelihood_ratio> current_soft =
        span<const log_likelihood_ratio>(soft_bits).subspan(node_size_byte * i_node, lifting_size);

    // Perform hard decision of the node
    valid &= hard_decision(out, current_soft, lifting_size * i_node);
  }

  return valid;
}

bool ldpc_decoder_impl::check_syndrome() const
{
  // Temporary buffers
  static_vector<log_likelihood_ratio, ldpc::MAX_LIFTING_SIZE> soft_shifted_bits(lifting_size);
  static_bit_buffer<ldpc::MAX_LIFTING_SIZE>                   hard_shifted_bits(lifting_size);
  static_bit_buffer<ldpc::MAX_LIFTING_SIZE>                   hard_syndrome_bits(lifting_size);

  // Make sure the last byte is zero for the static bit buffers
  hard_shifted_bits.get_buffer().back()  = 0;
  hard_syndrome_bits.get_buffer().back() = 0;

  // Calculate number of layers or check nodes
  unsigned nof_check_nodes = codeblock_length / lifting_size - bg_K;

  for (unsigned i_check_node = 0; i_check_node != nof_check_nodes; ++i_check_node) {
    // Obtain parity check node.
    const BG_adjacency_row_t& current_var_indices = current_graph->get_adjacency_row(i_check_node);

    for (BG_adjacency_row_t::const_iterator this_var_index = current_var_indices.cbegin(),
                                            last_var_index = current_var_indices.cend();
         (this_var_index != last_var_index) && (*this_var_index != NO_EDGE);
         ++this_var_index) {
      // Get shift.
      unsigned shift = current_graph->get_lifted_node(i_check_node, *this_var_index);

      // Select view of the soft bits
      span<const log_likelihood_ratio> this_soft_bits =
          span<const log_likelihood_ratio>(soft_bits).subspan((*this_var_index) * node_size_byte, lifting_size);

      // Circular shift of bits
      ocuduvec::circ_shift_backward(soft_shifted_bits, this_soft_bits, shift);

      // Perform hard decision in-place for the first node, early return if there is any undetermined soft bit
      if (this_var_index == current_var_indices.cbegin()) {
        if (!hard_decision(hard_syndrome_bits, soft_shifted_bits)) {
          return false;
        }
        continue;
      }

      // Hard decision on shifted soft bits, early return if there is any undetermined soft bit
      if (!hard_decision(hard_shifted_bits, soft_shifted_bits)) {
        return false;
      }

      // XOR this node hard-bits with the current syndrome
      ocuduvec::binary_xor(
          hard_syndrome_bits.get_buffer(), hard_shifted_bits.get_buffer(), hard_syndrome_bits.get_buffer());
    }

    // Check that all syndrome bits are zero. Early return false otherwise
    span<const uint8_t> hard_syndrome_bytes = hard_syndrome_bits.get_buffer();
    if (std::any_of(hard_syndrome_bytes.begin(), hard_syndrome_bytes.end(), [](uint8_t byte) { return byte != 0; })) {
      return false;
    }
  }

  return true;
}

// edatsika Free VRAM after decoding
ldpc_decoder_impl::~ldpc_decoder_impl()
{
  // Free buffers
  if (d_soft_bits) CHECK_HIP(hipFree(d_soft_bits));
  if (d_c2v)       CHECK_HIP(hipFree(d_c2v));
  if (d_v2c)       CHECK_HIP(hipFree(d_v2c));
  if (d_v2c_copy)  CHECK_HIP(hipFree(d_v2c_copy)); // keeps state of messages from VNs to CNs when updating soft bits

  // Free H matrix
  if (d_adj_matrix)  CHECK_HIP(hipFree(d_adj_matrix));
  if (d_row_offsets) CHECK_HIP(hipFree(d_row_offsets));
  if (d_row_lengths) CHECK_HIP(hipFree(d_row_lengths));
  
  // Free pointers
  d_soft_bits = nullptr;
  d_c2v = nullptr;
  d_v2c = nullptr;
  d_adj_matrix = nullptr;
  d_row_offsets = nullptr;
  d_row_lengths = nullptr;
}
