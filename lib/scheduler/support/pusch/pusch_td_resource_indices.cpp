// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "ocudu/scheduler/config/pusch_td_resource_indices.h"
#include "../../config/ue_configuration.h"
#include "pusch_default_time_allocation.h"
#include "ocudu/ocudulog/logger.h"
#include "ocudu/ocudulog/ocudulog.h"

using namespace ocudu;

using pusch_index_list = static_vector<unsigned, pusch_constants::MAX_NOF_PUSCH_TD_RES_ALLOCS>;

/// Get minimum value for k1 given the common and dedicated configurations.
static unsigned get_min_k1(span<const uint8_t> dl_data_to_ul_ack, const search_space_info* ss_info)
{
  unsigned min_k1 = *std::min(dl_data_to_ul_ack.begin(), dl_data_to_ul_ack.end());
  if (ss_info != nullptr) {
    min_k1 = *std::min(ss_info->get_k1_candidates().begin(), ss_info->get_k1_candidates().end());
  }
  return min_k1;
}

namespace {
struct dl_to_ul_k_map {
  dl_to_ul_k_map(unsigned sz_, const cell_configuration& cell_cfg_) : sz(sz_), cell_cfg(cell_cfg_)
  {
    dl_to_ul_map.assign(sz, std::vector<unsigned>(sz, 0));
    min_k2 = std::numeric_limits<unsigned>::max();
    for (const auto& td_res : cell_cfg.ul_cfg_common.init_ul_bwp.pusch_cfg_common.value().pusch_td_alloc_list) {
      min_k2 = std::min(static_cast<unsigned>(td_res.k2), min_k2);
    }
  }

  struct min_k {
    // Row index corresponding to min_k.
    unsigned value = 0;
    // Row index corresponding to min_k.
    unsigned dl_index = 0;
  };

  using min_k_vector = std::vector<min_k>;

  void build_matrix()
  {
    for (unsigned row_idx = 0; row_idx != sz; ++row_idx) {
      for (unsigned col_idx = 0; col_idx != sz; ++col_idx) {
        dl_to_ul_map[row_idx][col_idx] = compute_k(row_idx, col_idx);
      }
    }
  }

  void compute_min_ks()
  {
    min_ks.assign(sz, min_k{});
    for (unsigned col_idx = 0; col_idx != sz; ++col_idx) {
      for (unsigned row_idx = 0; row_idx != sz; ++row_idx) {
        if (dl_to_ul_map[row_idx][col_idx] == 0) {
          continue;
        }
        if (min_ks[col_idx].value != 0) {
          if (const bool update_row = dl_to_ul_map[row_idx][col_idx] < min_ks[col_idx].value; update_row) {
            min_ks[col_idx] = {dl_to_ul_map[row_idx][col_idx], row_idx};
          }
        } else {
          min_ks[col_idx].value    = dl_to_ul_map[row_idx][col_idx];
          min_ks[col_idx].dl_index = row_idx;
        }
      }
    }

    fmt::print("\nMin\nV: ");
    for (unsigned col_idx = 0; col_idx != sz; ++col_idx) {
      fmt::print("{}\t", min_ks[col_idx].value);
    }
    fmt::print("\nI: ");
    for (unsigned col_idx = 0; col_idx != sz; ++col_idx) {
      fmt::print("{}\t", min_ks[col_idx].dl_index);
    }
    fmt::print("\n");
  }

  void compute_dl_hist()
  {
    dl_hist.assign(sz, {});
    for (unsigned col_idx = 0; col_idx != sz; ++col_idx) {
      if (min_ks[col_idx].value != 0) {
        ocudu_assert(col_idx < min_ks.size() and min_ks[col_idx].dl_index < dl_hist.size(), "Wrong idx");
        dl_hist[min_ks[col_idx].dl_index].emplace_back(col_idx);
      }
    }

    fmt::print("\nHistogram\n");
    for (unsigned row_idx = 0; row_idx != sz; ++row_idx) {
      if (not dl_hist[row_idx].empty()) {
        ocudu_assert(row_idx < dl_hist.size(), "Wrong idx");
        for (auto hist : dl_hist[row_idx]) {
          fmt::print("{}\t", hist);
        }
      } else {
        fmt::print("x\t");
      }
      fmt::print("\n");
    }
    fmt::print("\n");
  }

  void remove_min_k(unsigned ul_sl)
  {
    ocudu_assert(ul_sl < min_ks.size() and min_ks[ul_sl].dl_index < dl_to_ul_map.size() and
                     ul_sl < dl_to_ul_map[min_ks[ul_sl].dl_index].size(),
                 "Wrong size");
    dl_to_ul_map[min_ks[ul_sl].dl_index][ul_sl] = 0;
  }

  std::vector<unsigned> get_dl_k2_values() const
  {
    std::vector<unsigned> dl_k2_values{sz, 0U};
    for (unsigned dl_sl_idx = 0; dl_sl_idx != sz; ++dl_sl_idx) {
      if (dl_hist[dl_sl_idx].size() == 1) {
        const unsigned ul_slot  = dl_hist[dl_sl_idx].front();
        dl_k2_values[dl_sl_idx] = min_ks[ul_slot].value;
      }
    }
    return dl_k2_values;
  }
  bool allocation_complete() const
  {
    bool complete = true;
    for (unsigned dl_sl_idx = 0; dl_sl_idx != sz; ++dl_sl_idx) {
      if (cell_cfg.is_dl_enabled(slot_point{cell_cfg.scs_common, dl_sl_idx}) and dl_hist[dl_sl_idx].size() > 1) {
        complete = false;
      }
    }

    for (unsigned ul_sl_idx = 0; ul_sl_idx != sz; ++ul_sl_idx) {
      if (cell_cfg.is_ul_enabled(slot_point{cell_cfg.scs_common, ul_sl_idx}) and min_ks[ul_sl_idx].value == 0U) {
        complete = false;
      }
    }

    return complete;
  }

  unsigned compute_k(unsigned dl_sl, unsigned ul_sl) const
  {
    if (not cell_cfg.is_ul_enabled(slot_point{cell_cfg.scs_common, ul_sl}) or
        not cell_cfg.is_dl_enabled(slot_point{cell_cfg.scs_common, dl_sl})) {
      return 0U;
    }
    // const unsigned candidate_k = ul_sl > dl_sl and ul_sl - dl_sl >= dl_sl ? ul_sl - dl_sl : sz + ul_sl - dl_sl;
    const unsigned candidate_k = ul_sl > dl_sl ? ul_sl - dl_sl : sz + ul_sl - dl_sl;
    return candidate_k >= min_k2 ? candidate_k : candidate_k + sz;
  }

  std::vector<unsigned> get_final_dl_k_mapping() const
  {
    std::vector<unsigned> final_dl_k_mapping(sz, 0);
    for (auto& min_elem : min_ks) {
      // Only values different from 0 have valid k.
      if (min_elem.value != 0U) {
        final_dl_k_mapping[min_elem.dl_index] = min_elem.value;
      }
    }

    fmt::print("\nIdx:\t");
    for (unsigned sl_idx = 0; sl_idx != sz; ++sl_idx) {
      fmt::print("{}\t", sl_idx);
    }
    fmt::print("\nk2: \t");
    for (unsigned sl_idx = 0; sl_idx != sz; ++sl_idx) {
      fmt::print("{}\t", final_dl_k_mapping[sl_idx]);
    }
    return final_dl_k_mapping;
  }
  void print_matrix()
  {
    fmt::print("\n");
    for (unsigned row_idx = 0; row_idx != sz; ++row_idx) {
      ocudu_assert(dl_to_ul_map.size() == sz, "Wrong size");
      for (unsigned col_idx = 0; col_idx != sz; ++col_idx) {
        ocudu_assert(dl_to_ul_map[row_idx].size() == sz, "Wrong size");
        fmt::print("{}\t", dl_to_ul_map[row_idx][col_idx]);
      }
      fmt::print("\n");
    }
  }

  using dl_to_ul = std::vector<unsigned>;
  std::vector<dl_to_ul> dl_to_ul_map;

  std::vector<std::vector<unsigned>> dl_hist;
  std::vector<min_k>                 min_ks;

  // Initialize this matrix.
  const unsigned            sz;
  const cell_configuration& cell_cfg;
  unsigned                  min_k2;
};
} // anonymous namespace

std::vector<unsigned> ocudu::compute_dl_ul_k_map(const cell_configuration& cell_cfg)
{
  const unsigned tdd_pattern_period_slots =
      cell_cfg.tdd_cfg_common.value().pattern1.dl_ul_tx_period_nof_slots +
      (cell_cfg.tdd_cfg_common.value().pattern2.has_value()
           ? cell_cfg.tdd_cfg_common.value().pattern2.value().dl_ul_tx_period_nof_slots
           : 0U);

  dl_to_ul_k_map dl_to_ul_mtx = dl_to_ul_k_map(tdd_pattern_period_slots, cell_cfg);

  // Build matrix.
  dl_to_ul_mtx.build_matrix();

  dl_to_ul_mtx.print_matrix();

  // Compute the minimum k.
  dl_to_ul_mtx.compute_min_ks();

  // Compute the hist.
  dl_to_ul_mtx.compute_dl_hist();

  //
  while (not dl_to_ul_mtx.allocation_complete()) {
    // Find all DL slots such that the histogram is > 1;
    std::vector<unsigned> dl_slots_non_1_hist;
    for (unsigned sl_idx = 0, sz = dl_to_ul_mtx.dl_hist.size(); sl_idx != sz; ++sl_idx) {
      if (dl_to_ul_mtx.dl_hist[sl_idx].size() > 1U) {
        dl_slots_non_1_hist.emplace_back(sl_idx);
      }
    }

    // Find the max among all min_k of the DL slots whose histogram > 1.
    unsigned max_min_k         = 0;
    unsigned dl_slot_max_min_k = 0;
    for (unsigned dl_sl : dl_slots_non_1_hist) {
      for (unsigned ul_sl : dl_to_ul_mtx.dl_hist[dl_sl]) {
        if (dl_to_ul_mtx.min_ks[ul_sl].value > max_min_k) {
          max_min_k         = dl_to_ul_mtx.min_ks[ul_sl].value;
          dl_slot_max_min_k = dl_sl;
        }
      }
    }

    // Reprocess UL slots that has the DL slot in common with that of dl_slot_max_min_k.
    for (unsigned ul_sl : dl_to_ul_mtx.dl_hist[dl_slot_max_min_k]) {
      if (dl_to_ul_mtx.compute_k(dl_slot_max_min_k, ul_sl) != max_min_k) {
        dl_to_ul_mtx.remove_min_k(dl_to_ul_mtx.dl_hist[dl_slot_max_min_k].front());
      }
    }

    dl_to_ul_mtx.print_matrix();

    // Recompute min ks and DL histogram
    dl_to_ul_mtx.compute_min_ks();
    dl_to_ul_mtx.compute_dl_hist();
  }

  return dl_to_ul_mtx.get_final_dl_k_mapping();
}

static span<const pusch_time_domain_resource_allocation>
get_pusch_time_domain_resource_table(const pusch_config_common& pusch_cfg_common, const search_space_info* ss_info)
{
  return ss_info != nullptr ? ss_info->pusch_time_domain_list : pusch_cfg_common.pusch_td_alloc_list;
}

/// Determine PUSCH TD resources for the FDD mode.
static pusch_index_list get_fdd_pusch_td_resource_indices(const pusch_config_common& pusch_cfg_common,
                                                          span<const uint8_t>        dl_data_to_ul_ack,
                                                          const search_space_info*   ss_info)
{
  const unsigned min_k1                 = get_min_k1(dl_data_to_ul_ack, ss_info);
  auto           pusch_time_domain_list = get_pusch_time_domain_resource_table(pusch_cfg_common, ss_info);

  pusch_index_list result;
  for (unsigned i = 0; i != pusch_time_domain_list.size(); ++i) {
    if (pusch_time_domain_list[i].k2 <= min_k1) {
      result.push_back(i);
    }
  }
  return result;
}

static bool is_dl_enabled_slot(slot_point slot, const std::optional<tdd_ul_dl_config_common>& tdd_cfg_common)
{
  if (not tdd_cfg_common.has_value()) {
    return true;
  }

  return has_active_tdd_dl_symbols(tdd_cfg_common.value(), slot.count());
}

pusch_index_list ocudu::get_pusch_td_resource_indices(slot_point                                    pdcch_slot,
                                                      const std::optional<tdd_ul_dl_config_common>& tdd_cfg_common,
                                                      const pusch_config_common&                    pusch_cfg_common,
                                                      span<const uint8_t>                           dl_data_to_ul_ack,
                                                      const search_space_info*                      ss_info)
{
  if (not tdd_cfg_common.has_value()) {
    // FDD case.
    return get_fdd_pusch_td_resource_indices(pusch_cfg_common, dl_data_to_ul_ack, ss_info);
  }

  // TDD case.
  const unsigned min_k1                 = get_min_k1(dl_data_to_ul_ack, ss_info);
  auto           pusch_time_domain_list = get_pusch_time_domain_resource_table(pusch_cfg_common, ss_info);
  const unsigned nof_full_ul_slots      = nof_full_ul_slots_per_tdd_period(tdd_cfg_common.value());
  const unsigned nof_full_dl_slots      = nof_dl_slots_per_tdd_period(tdd_cfg_common.value());
  const bool     is_dl_heavy            = nof_full_dl_slots >= nof_full_ul_slots;

  pusch_index_list result;
  for (unsigned td_idx = 0; td_idx != pusch_time_domain_list.size(); ++td_idx) {
    const pusch_time_domain_resource_allocation& pusch_td_res = pusch_time_domain_list[td_idx];
    // [Implementation-defined] PUSCH on partial UL slots is not supported.
    if (not is_tdd_full_ul_slot(tdd_cfg_common.value(), (pdcch_slot + pusch_td_res.k2).slot_index())) {
      continue;
    }

    if (is_dl_heavy and pusch_td_res.k2 <= min_k1) {
      // DL-heavy case.
      // [Implementation-defined] For DL heavy TDD configuration, in the PUSCH time domain resources list, we allow only
      // entries with the same k2 value that are less than or equal to minimum value of k1(s); these multiple entries
      // can have different symbols.
      if (not result.empty() and
          std::any_of(result.begin(),
                      result.end(),
                      [candidate_k2 = pusch_td_res.k2, pusch_time_domain_list](unsigned td_idx_it) {
                        return candidate_k2 != pusch_time_domain_list[td_idx_it].k2;
                      })) {
        break;
      }
      result.push_back(td_idx);
    }
    if (not is_dl_heavy) {
      // UL-heavy case.
      // [Implementation-defined] For UL heavy TDD configuration multiple k2 values are considered for scheduling
      // since it allows multiple UL PDCCH allocations in the same slot for same UE but with different k2 values.
      result.push_back(td_idx);
    }
  }
  return result;
}

std::vector<pusch_index_list>
ocudu::get_pusch_td_resource_indices_per_slot(subcarrier_spacing                            scs,
                                              const std::optional<tdd_ul_dl_config_common>& tdd_cfg_common,
                                              const pusch_config_common&                    pusch_cfg_common,
                                              span<const uint8_t>                           dl_data_to_ul_ack,
                                              const search_space_info*                      ss_info)
{
  // Note: [Implementation-defined] In case of FDD, we only consider one slot.
  if (not tdd_cfg_common.has_value()) {
    return {get_fdd_pusch_td_resource_indices(pusch_cfg_common, dl_data_to_ul_ack, ss_info)};
  }

  // NOTE: [Implementation-defined] In case of FDD, we consider only one slot as all slots are similar unlike in TDD
  // where there can be DL/UL full or partial slots.
  const unsigned nof_slots = nof_slots_per_tdd_period(tdd_cfg_common.value());

  // List circularly indexed by slot with the list of applicable PUSCH Time Domain resource indexes per slot.
  // NOTE: The list would be empty for UL slots.
  std::vector<pusch_index_list> pusch_td_list_per_slot(nof_slots);
  // Populate the initial list of applicable PUSCH time domain resources per slot.
  for (unsigned slot_idx = 0, e = nof_slots; slot_idx != e; ++slot_idx) {
    slot_point pdcch_slot{to_numerology_value(scs), slot_idx};
    if (is_dl_enabled_slot(pdcch_slot, tdd_cfg_common)) {
      pusch_td_list_per_slot[slot_idx] =
          get_pusch_td_resource_indices(pdcch_slot, tdd_cfg_common, pusch_cfg_common, dl_data_to_ul_ack, ss_info);
    }
  }
  return pusch_td_list_per_slot;
}

static std::optional<unsigned> find_td_index_with_k2(span<const pusch_time_domain_resource_allocation> pusch_res_list,
                                                     span<const unsigned>                              valid_indexes,
                                                     unsigned                                          k2)
{
  const auto* it =
      std::find_if(valid_indexes.begin(), valid_indexes.end(), [&pusch_res_list, k2](unsigned pusch_td_res_idx) {
        return pusch_res_list[pusch_td_res_idx].k2 == k2;
      });
  if (it == valid_indexes.end()) {
    return std::nullopt;
  }
  return *it;
}

std::vector<pusch_index_list>
ocudu::get_fairly_distributed_pusch_td_resource_indices(subcarrier_spacing                            scs,
                                                        const std::optional<tdd_ul_dl_config_common>& tdd_cfg_common,
                                                        const pusch_config_common&                    pusch_cfg_common,
                                                        span<const uint8_t>                           dl_data_to_ul_ack,
                                                        const search_space_info*                      ss_info)
{
  // List circularly indexed by slot with the list of applicable PUSCH Time Domain resource indexes per slot.
  // NOTE: The list would be empty for UL slots.
  std::vector<pusch_index_list> initial_pusch_td_list_per_slot =
      get_pusch_td_resource_indices_per_slot(scs, tdd_cfg_common, pusch_cfg_common, dl_data_to_ul_ack, ss_info);

  // In FDD case, we do not need to proceed further.
  if (not tdd_cfg_common.has_value()) {
    return initial_pusch_td_list_per_slot;
  }

  const unsigned nof_dl_slots      = nof_dl_slots_per_tdd_period(tdd_cfg_common.value());
  const unsigned nof_full_ul_slots = nof_full_ul_slots_per_tdd_period(tdd_cfg_common.value());

  // In DL-heavy case, we do not need to proceed further.
  if (nof_dl_slots >= nof_full_ul_slots) {
    return initial_pusch_td_list_per_slot;
  }

  const unsigned nof_slots = nof_slots_per_tdd_period(tdd_cfg_common.value());

  // Fetch the relevant PUSCH time domain resource list.
  span<const pusch_time_domain_resource_allocation> pusch_time_domain_list =
      get_pusch_time_domain_resource_table(pusch_cfg_common, ss_info);
  const unsigned max_k2 = pusch_time_domain_list.back().k2;

  // [Implementation-defined] Fairness is achieved by computing nof. UL PDCCHs to be scheduled per each PDCCH slot.
  // Then, iterating over UL slots finding the nearest PDCCH slot to it such that nof. UL PDCCHs at each PDCCH slot more
  // or less satisfies the earlier computed value.

  // Estimate the nof. UL PDCCHs that can be scheduled in each PDCCH slot.
  const auto nof_ul_pdcchs_per_dl_slot =
      static_cast<unsigned>(std::round(static_cast<double>(nof_full_ul_slots) / static_cast<double>(nof_dl_slots)));

  // List circularly indexed by slot with the list of applicable PUSCH Time Domain resource indexes per slot fairly
  // distributed among all the PDCCH slots.
  // NOTE: The list would be empty for UL slots.
  std::vector<pusch_index_list> final_pusch_td_list_per_slot(nof_slots);

  // Iterate from latest UL slot to earliest and find the closest PDCCH slot to that UL slot.
  // NOTE: There can be scenarios where the closest PDCCH slot may not be able to schedule PUSCH in the chosen UL slot.
  // In this case we move on next closest PDCCH slot, so on and so forth.
  for (unsigned candidate_idx = 0; candidate_idx != nof_slots; ++candidate_idx) {
    unsigned ul_slot_idx = nof_slots - candidate_idx;
    // Skip if it's not a UL slot.
    // TODO: Revisit when scheduling of PUSCH over partial UL slots is supported.
    if (not is_tdd_full_ul_slot(tdd_cfg_common.value(), ul_slot_idx)) {
      continue;
    }
    // Flag indicating whether a valid PDCCH slot for a given UL slot is found or not.
    bool                    no_pdcch_slot_found = true;
    std::optional<unsigned> last_valid_k2;
    for (unsigned k2 = 0; k2 <= max_k2; ++k2) {
      unsigned dl_slot_idx = (nof_slots + ul_slot_idx - k2) % nof_slots;
      // Skip if it's not a DL slot.
      if (not has_active_tdd_dl_symbols(tdd_cfg_common.value(), dl_slot_idx)) {
        continue;
      }
      // Check whether there is a PUSCH time domain resource with required k2 value for the PDCCH slot.
      std::optional<unsigned> idx =
          find_td_index_with_k2(pusch_time_domain_list, initial_pusch_td_list_per_slot[dl_slot_idx], k2);
      if (not idx.has_value()) {
        continue;
      }
      // Store PDCCH slot index at which a valid PUSCH time domain resource was found to schedule PUSCH at given UL
      // slot.
      last_valid_k2 = k2;
      // Skip if nof. PUSCH time domain resource indexes for this PDCCH slot exceed nof. UL PDCCHs that can be scheduled
      // in each PDCCH slot.
      if (final_pusch_td_list_per_slot[dl_slot_idx].size() >= nof_ul_pdcchs_per_dl_slot) {
        // Search for next PDCCH slot.
        continue;
      }
      // Store the nof. PUSCH time domain resource index for this PDCCH slot.
      final_pusch_td_list_per_slot[dl_slot_idx].push_back(*idx);
      no_pdcch_slot_found = false;
      break;
    }

    // Note: This should not happen if the config passed the validation.
    ocudu_sanity_check(
        last_valid_k2.has_value(), "Invalid TDD pattern which leads to UL slot index={} with no valid k2", ul_slot_idx);

    // [Implementation-defined] If no PDCCH slot is found we pick the last valid PDCCH slot for this UL slot, regardless
    // of the restriction to not allow more than \c nof_ul_pdcchs_per_dl_slot UL PDCCHs per PDCCH slot.
    if (no_pdcch_slot_found) {
      std::optional<uint8_t> min_k2;
      for (const auto& pusch_time_domain : pusch_time_domain_list) {
        min_k2 = std::min(min_k2.value_or(pusch_time_domain.k2), pusch_time_domain.k2);
      }
      const unsigned required_k2      = last_valid_k2.value();
      const unsigned pdcch_slot_index = (ul_slot_idx + nof_slots - required_k2) % nof_slots;

      // If the required k2 value is less than the minimum k2 value in the PUSCH time domain resource list, then we look
      // for the minimum k2 value that is greater than the DL-UL transmission period, as this is the PDCCH slot closest
      // to the PUSCH slot.
      std::optional<uint8_t> candidate_required_k2;
      if (required_k2 < min_k2.value()) {
        for (const auto& pusch_time_domain : pusch_time_domain_list) {
          if (pusch_time_domain.k2 > tdd_cfg_common.value().pattern1.dl_ul_tx_period_nof_slots) {
            candidate_required_k2 =
                std::min(candidate_required_k2.value_or(pusch_time_domain.k2), pusch_time_domain.k2);
          }
        }
      } else {
        candidate_required_k2 = required_k2;
      }
      // If a valid PUSCH time domain resource is found for the required k2 value, then we store it.
      std::optional<unsigned> pusch_td_res_idx_for_required_k2 = std::nullopt;
      if (candidate_required_k2.has_value()) {
        auto& init_push_list = initial_pusch_td_list_per_slot[pdcch_slot_index];
        auto* it             = std::find_if(init_push_list.begin(),
                                init_push_list.end(),
                                [&pusch_time_domain_list, candidate_required_k2](unsigned pusch_td_res_idx) {
                                  return pusch_time_domain_list[pusch_td_res_idx].k2 == candidate_required_k2.value();
                                });
        if (it != init_push_list.end()) {
          pusch_td_res_idx_for_required_k2.emplace(*it);
        }
      }
      if (pusch_td_res_idx_for_required_k2.has_value()) {
        final_pusch_td_list_per_slot[pdcch_slot_index].push_back(*pusch_td_res_idx_for_required_k2);
      } else {
        ocudulog::basic_logger& logger = ocudulog::fetch_basic_logger("SCHED", false);
        logger.warning("No valid PUSCH time domain resource found for UL slot dx={}", ul_slot_idx);
      }
    }
  }

  // Sort PUSCH time domain resource indexes (ascending order) in the final list of PUSCH time domain resources
  // maintained per slot.
  for (unsigned slot_idx = 0, e = nof_slots; slot_idx != e; ++slot_idx) {
    if (has_active_tdd_dl_symbols(tdd_cfg_common.value(), slot_idx)) {
      std::sort(final_pusch_td_list_per_slot[slot_idx].begin(), final_pusch_td_list_per_slot[slot_idx].end());
    }
  }

  return final_pusch_td_list_per_slot;
}
