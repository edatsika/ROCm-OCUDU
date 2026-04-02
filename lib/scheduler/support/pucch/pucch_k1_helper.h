// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "ocudu/adt/span.h"
#include "ocudu/adt/static_vector.h"
#include "ocudu/ran/pusch/pusch_constants.h"
#include <optional>
#include <vector>

namespace ocudu {

struct tdd_ul_dl_config_common;
struct pusch_time_domain_resource_allocation;

using k1_list = static_vector<uint8_t, 8>;

std::vector<static_vector<uint8_t, 8>>
get_pucch_k1_list_per_slot(span<const uint8_t>                                       dl_data_to_ul_ack,
                           const std::optional<tdd_ul_dl_config_common>&             tdd_cfg_common,
                           const std::vector<pusch_time_domain_resource_allocation>& pusch_td_alloc_list,
                           const std::vector<static_vector<unsigned, pusch_constants::MAX_NOF_PUSCH_TD_RES_ALLOCS>>&
                               pusch_td_resource_indices_per_slot);

} // namespace ocudu
