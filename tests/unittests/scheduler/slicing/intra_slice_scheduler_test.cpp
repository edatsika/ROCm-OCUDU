// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "tests/test_doubles/scheduler/cell_config_builder_profiles.h"
#include "tests/test_doubles/scheduler/scheduler_config_helper.h"
#include "tests/unittests/scheduler/test_utils/scheduler_test_simulator.h"
#include <gtest/gtest.h>

using namespace ocudu;

class intra_slice_scheduler_test : public scheduler_test_simulator, public ::testing::Test
{
public:
  intra_slice_scheduler_test() : scheduler_test_simulator(4, subcarrier_spacing::kHz30)
  {
    builder_params.tdd_ul_dl_cfg_common = cell_config_builder_profiles::create_tdd_pattern(
        cell_config_builder_profiles::tdd_pattern_profile_fr1_30khz::DDDSU);
    // So we can schedule PUSCHs from TDD pattern slot 1.
    builder_params.min_k2 = 3;

    add_cell(sched_config_helper::make_default_sched_cell_configuration_request(builder_params));

    // Add a regular (non-fallback) UE with a DRB. Its presence in the DRB slice ensures that ul_sched() does not return
    // early on the get_slice_ues().empty() check, so pusch_slot is updated when the k2=4 candidate is processed in TDD
    // pattern slot 0.
    auto reg_cfg     = sched_config_helper::create_default_sched_ue_creation_request(cell_cfg().params, {LCID_MIN_DRB});
    reg_cfg.ue_index = regular_ue_idx;
    reg_cfg.crnti    = regular_ue_rnti;
    reg_cfg.starts_in_fallback = false;
    scheduler_test_simulator::add_ue(reg_cfg, true);
  }

  cell_config_builder_params builder_params{cell_config_builder_profiles::create(duplex_mode::TDD)};
  // DDDSU period length in slots.
  static constexpr unsigned tdd_period       = 5;
  const du_cell_index_t     cell_idx         = to_du_cell_index(0);
  const du_ue_index_t       regular_ue_idx   = to_du_ue_index(0);
  const rnti_t              regular_ue_rnti  = to_rnti(0x4601);
  const du_ue_index_t       fallback_ue_idx  = to_du_ue_index(1);
  const rnti_t              fallback_ue_rnti = to_rnti(0x4602);
};

TEST_F(intra_slice_scheduler_test, intra_slice_scheduler_uses_up_to_date_resource_grid_state)
{
  // Advance until the beginning of a TDD pattern.
  while (next_slot.without_hyper_sfn().slot_index() % tdd_period != 0) {
    run_slot();
  }

  // Process TDD pattern slot 0. The intra-slice scheduler will run a slice candidate for UL slot 4 (but won't have
  // anything to allocate).
  run_slot();

  // Add a fallback UE with a small BSR.
  auto fb_cfg               = sched_config_helper::create_default_sched_ue_creation_request(cell_cfg().params, {});
  fb_cfg.ue_index           = fallback_ue_idx;
  fb_cfg.crnti              = fallback_ue_rnti;
  fb_cfg.starts_in_fallback = true;
  fb_cfg.ul_ccch_slot_rx    = next_slot.without_hyper_sfn();
  scheduler_test_simulator::add_ue(fb_cfg);

  push_bsr(ul_bsr_indication_message{
      cell_idx, fallback_ue_idx, fallback_ue_rnti, bsr_format::SHORT_BSR, {{uint_to_lcg_id(0), 128}}});

  // Give the regular UE a large BSR so it requests many PRBs (enough to overlap with the fallback UE's small allocation
  // if used_ul_vrbs is stale).
  push_bsr(ul_bsr_indication_message{
      cell_idx, regular_ue_idx, regular_ue_rnti, bsr_format::LONG_BSR, {{uint_to_lcg_id(2), 50000}}});

  // Process TDD pattern slot 1. The fallback scheduler will grant a PUSCH for the fallback UE, and the intra-slice
  // scheduler will do the same for the regular UE.
  run_slot();
  // Advance to TDD pattern slot 4 (the UL slot).
  run_slot();
  run_slot();
  run_slot();

  // Check the PUSCHs are scheduled and do not overlap.
  const sched_result*  result        = last_sched_result(cell_idx);
  const ul_sched_info* fb_pusch      = find_ue_pusch(fallback_ue_rnti, *result);
  const ul_sched_info* regular_pusch = find_ue_pusch(regular_ue_rnti, *result);

  ASSERT_NE(fb_pusch, nullptr) << "Fallback UE not scheduled on PUSCH in UL slot";
  ASSERT_NE(regular_pusch, nullptr) << "Regular UE not scheduled on PUSCH in UL slot";

  ASSERT_TRUE(fb_pusch->pusch_cfg.rbs.is_type1());
  ASSERT_TRUE(regular_pusch->pusch_cfg.rbs.is_type1());

  EXPECT_FALSE(fb_pusch->pusch_cfg.rbs.type1().overlaps(regular_pusch->pusch_cfg.rbs.type1()))
      << fmt::format("PUSCH RB collision detected: fallback UE rbs=[{}..{}) overlaps regular UE rbs=[{}..{})",
                     fb_pusch->pusch_cfg.rbs.type1().start(),
                     fb_pusch->pusch_cfg.rbs.type1().stop(),
                     regular_pusch->pusch_cfg.rbs.type1().start(),
                     regular_pusch->pusch_cfg.rbs.type1().stop());
}
