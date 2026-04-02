// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "ocudu/adt/byte_buffer.h"
#include "ocudu/cu_cp/cu_cp_location_reporting_types.h"
#include "ocudu/cu_cp/cu_cp_types.h"
#include "ocudu/cu_cp/inter_cu_handover_messages.h"
#include "ocudu/ran/cause/xnap_cause.h"
#include "ocudu/ran/cu_types.h"
#include "ocudu/ran/guami.h"
#include "ocudu/ran/nr_cgi.h"
#include "ocudu/security/security.h"
#include "ocudu/support/io/transport_layer_address.h"
#include "ocudu/xnap/xnap_types.h"
#include <chrono>

namespace ocudu::ocucp {

/// Related to common type XNAP handover request, defined in TS 38.423 section 9.1.1.1.
struct xnap_ue_context_info_ho_request {
  unsigned                                                              amf_ue_id;
  transport_layer_address                                               amf_addr;
  security::security_context                                            security_context;
  cu_cp_aggregate_maximum_bit_rate                                      ue_ambr;
  slotted_id_vector<pdu_session_id_t, cu_cp_pdu_session_res_setup_item> pdu_session_res_to_be_setup_list;
  byte_buffer                                                           rrc_handover_preparation_information;
  std::optional<location_report_request>                                location_report_info;
};

/// Common type XNAP handover request, defined in TS 38.423 section 9.1.1.1.
struct xnap_handover_request {
  ue_index_t                      ue_index;
  xnap_cause_t                    cause;
  nr_cell_global_id_t             nr_cgi;
  guami_t                         guami;
  xnap_ue_context_info_ho_request ue_context_info_ho_request;
  /// When true, signals a Conditional HO preparation (ch_oinfo_req IE is included in HandoverRequest).
  /// The target allocates resources but defers execution until the UE arrives.
  bool is_conditional_handover = false;
  /// Timeout for the CHO preparation at the target (used to populate cho_time_based_info). Only
  /// relevant when is_conditional_handover is true.
  std::chrono::milliseconds cho_timeout = std::chrono::milliseconds{0};
  // TODO: add optional fields.
};

struct xnap_handover_preparation_response {
  bool success = false;
  /// CHO-specific fields; populated only when is_conditional_handover == true.
  byte_buffer       packed_rrc_recfg; ///< Extracted from HO Request Ack; embedded in outer condRRCRecfg.
  unsigned          rrc_transaction_id = 0;
  peer_xnap_ue_id_t peer_xnap_ue_id    = peer_xnap_ue_id_t::invalid; ///< Target XNAP UE ID (for cancel).
};

struct xnap_handover_target_execution_context {
  xnc_peer_index_t                                    xnc_index;
  ue_index_t                                          ue_index;
  unsigned                                            amf_ue_id;
  std::vector<cu_cp_xn_pdu_session_res_admitted_item> pdu_session_res_admitted_list;
  std::vector<cu_cp_pdu_session_with_cause_item>      pdu_session_failed_to_setup_list;
  /// When true, the UE arrived via Conditional HO (source sent no explicit HO Command);
  /// the target execution follows the CHO sequence (TS 38.423 Section 8.2.4).
  bool is_conditional_handover = false;
};

} // namespace ocudu::ocucp
