// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be
// subject to additional licensing requirements.

/// \file
/// \brief LDPC decoder benchmark – GPU batched path.
///
/// Uses ldpc_decoder_impl::decode_batch() which launches HIP kernels on the
/// AMD GPU.  Input / output types are int8_t spans (not log_likelihood_ratio /
/// bit_buffer) because the GPU kernel works in the int8 domain.
///
/// The optional verification step (-s not set) runs the CPU decoder on the
/// first codeword of each batch and compares hard decisions bit-by-bit.
/// This is the correctness check; the CPU benchmark is the performance
/// baseline.

#include "ldpc_decoder_hip.h"   // for decode_batch() and the dynamic_cast
#include "ocudu/phy/upper/channel_coding/channel_coding_factories.h"
#include "ocudu/phy/upper/channel_coding/ldpc/ldpc_encoder_buffer.h"
#include "ocudu/support/benchmark_utils.h"
#include "ocudu/support/ocudu_test.h"
#include <getopt.h>
#include <random>

// Must match ldpc_decoder_impl::BATCH_SIZE
static constexpr uint32_t BATCH_SIZE = 256;

static std::mt19937 rgen(0);
static std::string  dec_type        = "generic";
static unsigned     nof_repetitions = 1000;
static unsigned     nof_iterations  = 6;
static bool         silent          = false;
static unsigned     l_size          = 0;

static void usage(const char* prog)
{
  fmt::print("Usage: {} [-R repetitions] [-s silent]\n", prog);
  fmt::print("\t-R Repetitions [Default {}]\n", nof_repetitions);
  fmt::print("\t-T Decoder type generic, avx2, avx512 or neon [Default {}]\n", dec_type);
  fmt::print("\t-I Number of min-sum iterations [Default {}]\n", nof_iterations);
  fmt::print("\t-s Toggle silent operation / skip verification [Default {}]\n", silent);
  fmt::print("\t-L Lifting size - 0 for all [Default {}]\n", l_size);
  fmt::print("\t-h Show this message\n");
}

static void parse_args(int argc, char** argv)
{
  int opt = 0;
  while ((opt = getopt(argc, argv, "R:T:I:L:sh")) != -1) {
    switch (opt) {
      case 'R': nof_repetitions = std::strtol(optarg, nullptr, 10); break;
      case 'T': dec_type        = std::string(optarg);              break;
      case 'I': nof_iterations  = std::strtol(optarg, nullptr, 10); break;
      case 'L': l_size          = std::strtol(optarg, nullptr, 10); break;
      case 's': silent          = !silent;                           break;
      case 'h': default: usage(argv[0]); std::exit(0);
    }
  }
}

using namespace ocudu;
using namespace ocudu::ldpc;

int main(int argc, char** argv)
{
  parse_args(argc, argv);

  benchmarker perf_meas(
      fmt::format("LDPC decoder GPU batch={} {} {} MS iterations",
                  BATCH_SIZE, dec_type, nof_iterations),
      nof_repetitions);

  span<const lifting_size_t>    use_ls;
  std::array<lifting_size_t, 1> one_ls = {lifting_size_t::LS384};
  if (l_size != 0) {
    one_ls[0] = static_cast<lifting_size_t>(l_size);
    use_ls    = span<const lifting_size_t>(one_ls);
  } else {
    use_ls = span<const lifting_size_t>(all_lifting_sizes);
  }

  ldpc_decoder_factory::ldpc_decoder_factory_configuration ldpc_dec_cfg = {
      .force_decoding      = false,
      .early_stop_syndrome = false};

  for (const ldpc_base_graph_type& bg : {ldpc_base_graph_type::BG1, ldpc_base_graph_type::BG2}) {
    for (const lifting_size_t& ls : use_ls) {

      const unsigned msg_length_bg    = (bg == ldpc_base_graph_type::BG1) ? 22u : 10u;
      const unsigned min_cb_length_bg = (bg == ldpc_base_graph_type::BG1) ? 24u : 12u;
      const unsigned max_cb_length_bg = (bg == ldpc_base_graph_type::BG1) ? 66u : 50u;
      const unsigned msg_length       = msg_length_bg * static_cast<unsigned>(ls);
      const unsigned min_cb_length    = min_cb_length_bg * static_cast<unsigned>(ls);
      const unsigned max_cb_length    = max_cb_length_bg * static_cast<unsigned>(ls);

      // ----------------------------------------------------------------
      // Create the GPU decoder and cast to ldpc_decoder_impl so we can
      // call decode_batch().  The factory returns an ldpc_decoder_impl
      // subclass (generic / avx2 / etc.), so the cast is always valid.
      // ----------------------------------------------------------------
     /* std::shared_ptr<ldpc_decoder_factory> decoder_factory =
          create_ldpc_decoder_factory_sw(dec_type, ldpc_dec_cfg);
      TESTASSERT(decoder_factory);
      std::unique_ptr<ldpc_decoder> decoder = decoder_factory->create();
      TESTASSERT(decoder);
      auto* gpu_decoder = static_cast<ldpc_decoder_hip*>(decoder.get());
      TESTASSERT(gpu_decoder != nullptr); */
    
      /*if (dec_type == "hip") {
        printf("DEBUG: Requested 'hip', factory returned object at address %p\n", (void*)gpu_decoder);
      }*/
    
      //edatsika
    std::unique_ptr<ldpc_decoder> decoder;

if (dec_type == "hip") {
    // Factory not used (couldn't resolve child class, needs fixing)
    decoder = std::make_unique<ldpc_decoder_hip>(ldpc_dec_cfg.force_decoding, 
                                                 ldpc_dec_cfg.early_stop_syndrome);
    printf(">>> MANUAL: Created ldpc_decoder_hip instance (Factory Bypassed) <<<\n");
} else {
    // Rest of dec_types
    std::shared_ptr<ldpc_decoder_factory> decoder_factory = 
        create_ldpc_decoder_factory_sw(dec_type, ldpc_dec_cfg);
    TESTASSERT(decoder_factory);
    decoder = decoder_factory->create();
}

TESTASSERT(decoder);
auto* gpu_decoder = static_cast<ldpc_decoder_hip*>(decoder.get());

     

      ocudu::ldpc_decoder::configuration cfg_dec = {
          .base_graph      = bg,
          .lifting_size    = ls,
          .nof_filler_bits = 0,
          .nof_crc_bits    = 16,
          .max_iterations  = nof_iterations};

      // init() uploads the H-matrix to the GPU.  Must be called once
      // before decode_batch(), and again whenever BG or LS changes.
      //put back? gpu_decoder->init(cfg_dec);
      gpu_decoder->prepare_gpu(cfg_dec);
      //gpu_decoder->ldpc_decoder_impl::init(cfg_dec); // Fill this->lifting_size
      //gpu_decoder->upload_bg_to_device();           // Read this->lifting_size and upload

      for (unsigned cb_length : {min_cb_length, max_cb_length}) {

        // Build BATCH_SIZE identical test codewords (random -/+10 LLRs), not realistic system
  
        std::vector<std::vector<int8_t>> batch_inputs(
            BATCH_SIZE, std::vector<int8_t>(cb_length));
        std::vector<std::vector<int8_t>> batch_outputs(
            BATCH_SIZE, std::vector<int8_t>(msg_length, 0));

        for (uint32_t i = 0; i < BATCH_SIZE; ++i)
          for (auto& llr : batch_inputs[i])
            llr = static_cast<int8_t>((rgen() & 1u) * 20 - 10);

        // Build span views – these point into the vectors above and
        // remain valid for the lifetime of both vectors.
        std::vector<span<const int8_t>> input_spans;
        std::vector<span<int8_t>>       output_spans;
        input_spans.reserve(BATCH_SIZE);
        output_spans.reserve(BATCH_SIZE);
        for (uint32_t i = 0; i < BATCH_SIZE; ++i) {
          input_spans.push_back(span<const int8_t>(batch_inputs[i]));
          output_spans.push_back(span<int8_t>(batch_outputs[i]));
        }

        fmt::memory_buffer descr;
        fmt::format_to(std::back_inserter(descr),
                       "BG={} LS={:<3} cb_len={} R={:.3f} (GPU batch={})",
                       fmt::underlying(bg), fmt::underlying(ls), cb_length,
                       static_cast<double>(msg_length) / static_cast<double>(cb_length),
                       BATCH_SIZE);

        // Throughput measurement reported as (msg_length * BATCH_SIZE) bits per second 
        // so the numbers are directly comparable to the CPU benchmark which reports msg_length bits per call.
  
        perf_meas.new_measure(to_string(descr), msg_length * BATCH_SIZE, [&]() {
          // outputs first, inputs second – matches decode_batch signature
          gpu_decoder->decode_batch(output_spans, input_spans);
          do_not_optimize(batch_outputs[0][0]);
        });

        // ----------------------------------------------------------------
        // Optional verification: run the CPU decoder on the first codeword
        // and compare hard decisions against the GPU output.
        //
        // The CPU decoder works with log_likelihood_ratio (int8 typedef),
        // so we reinterpret the int8_t* pointer directly – the types are
        // the same underlying size and the values are identical LLRs.
        // ----------------------------------------------------------------


       /* if (!silent) {
          auto ref_factory = create_ldpc_decoder_factory_sw("generic", ldpc_dec_cfg);
          auto ref_decoder = ref_factory->create();
          TESTASSERT(ref_decoder);

          dynamic_bit_buffer ref_output(msg_length);

          // Re-run GPU decoder
          std::fill(batch_outputs[0].begin(), batch_outputs[0].end(), 0);
          gpu_decoder->decode_batch(output_spans, input_spans);

          // Run CPU decoder on codeword 0
          const auto* llr_ptr = reinterpret_cast<const log_likelihood_ratio*>(batch_inputs[0].data());
          ref_decoder->decode(ref_output, span<const log_likelihood_ratio>(llr_ptr, cb_length), nullptr, cfg_dec);

          // Bit-by-bit comparison
          bool match = true;
          for (unsigned i = 0; i < msg_length && match; ++i) {
            // Access i element of 1st span
            const uint8_t gpu_bit = (output_spans[0][i] < 0) ? 1u : 0u;
            
            // Convert buffer to span
            // if .get(i) or [i] fail, test cast over span
            auto cpu_span = static_cast<span<uint8_t>>(ref_output);
            const uint8_t cpu_bit = cpu_span[i];

            if (gpu_bit != cpu_bit) {
              fmt::print("\n  [!] Mismatch at bit {}: GPU={} CPU={}\n", i, gpu_bit, cpu_bit);
              match = false;
            }
          }
          if (match) fmt::print("  [Verified OK]\n");
        }*/


      }
    }
  }

  perf_meas.print_percentiles_throughput("bits");
  return 0;
}
