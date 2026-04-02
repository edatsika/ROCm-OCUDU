// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "inter_cu_cho_target_execution_routine.h"
#include "../pdu_session_routine_helpers.h"
#include "ocudu/cu_cp/inter_cu_handover_messages.h"
#include "ocudu/security/security.h"
#include "ocudu/support/async/coroutine.h"
#include "ocudu/xnap/xnap.h"

using namespace ocudu;
using namespace ocucp;

inter_cu_cho_target_execution_routine::inter_cu_cho_target_execution_routine(
    cu_cp_ue*                                    ue_,
    const xnap_handover_target_execution_context& execution_ctxt_,
    e1ap_bearer_context_manager&                  e1ap_,
    ngap_interface&                               ngap_,
    xnap_interface*                               xnap_,
    ocudulog::basic_logger&                       logger_) :
  ue(ue_),
  execution_ctxt(execution_ctxt_),
  e1ap(e1ap_),
  ngap(ngap_),
  xnap(xnap_),
  logger(logger_)
{
}

void inter_cu_cho_target_execution_routine::operator()(coro_context<async_task<void>>& ctx)
{
  CORO_BEGIN(ctx);

  if (ue == nullptr) {
    logger.warning("\"{}\" failed. Cause: UE got removed", name());
    CORO_EARLY_RETURN();
  }

  if (xnap == nullptr) {
    logger.warning("\"{}\" failed. Cause: XNAP interface is nullptr", name());
    CORO_EARLY_RETURN();
  }

  logger.debug("ue={}: \"{}\" started...", ue->get_ue_index(), name());

  // Step 1: Await RRCReconfigurationComplete - the UE autonomously attached to the target cell.
  if (!initialize_reconfiguration_timeout()) {
    logger.warning(
        "ue={}: \"{}\" failed. Cause: Failed to initialize reconfiguration timeout", ue->get_ue_index(), name());
    CORO_EARLY_RETURN();
  }
  CORO_AWAIT_VALUE(reconf_result,
                   ue->get_rrc_ue()->handle_handover_reconfiguration_complete_expected(0, reconf_timeout));
  if (!reconf_result) {
    logger.warning(
        "ue={}: \"{}\" failed. Cause: Failed to receive RRC Reconfiguration Complete", ue->get_ue_index(), name());
    CORO_EARLY_RETURN();
  }

  // Step 2: Send HandoverSuccess to the source CU-CP.
  xnap->handle_handover_success_required(ue->get_ue_index(), ue->get_rrc_ue()->get_cell_context().cgi);

  // Step 3: Await SN Status Transfer from the source CU-CP.
  CORO_AWAIT_VALUE(sn_status, xnap->handle_sn_status_transfer_expected(ue->get_ue_index()));
  if (!sn_status.has_value()) {
    logger.warning("ue={}: \"{}\" failed. Cause: Failed to receive SN Status Transfer", ue->get_ue_index(), name());
    CORO_EARLY_RETURN();
  }

  // Step 4: Apply PDCP state to CU-UP via E1AP bearer context modification.
  bearer_context_modification_request.ue_index = ue->get_ue_index();
  fill_e1ap_bearer_context_modification_request();
  CORO_AWAIT_VALUE(bearer_context_modification_response,
                   e1ap.handle_bearer_context_modification_request(bearer_context_modification_request));

  // Step 5: Send Path Switch Request to AMF.
  path_switch_req = fill_path_switch_request();
  CORO_AWAIT_VALUE(path_switch_response,
                   ngap.get_ngap_control_message_handler().handle_path_switch_request_required(path_switch_req));
  // TODO: Handle path switch response and proceed with the routine accordingly.

  // Step 6: Send UE Context Release to source CU-CP.
  if (!xnap->handle_ue_context_release_required(ue->get_ue_index())) {
    logger.warning(
        "ue={}: \"{}\" failed. Cause: Failed to transmit UE Context Release", ue->get_ue_index(), name());
  }

  CORO_RETURN();
}

void inter_cu_cho_target_execution_routine::fill_e1ap_bearer_context_modification_request()
{
  auto& ng_request = bearer_context_modification_request.ng_ran_bearer_context_mod_request;
  ng_request.emplace();
  slotted_id_vector<pdu_session_id_t, e1ap_pdu_session_res_to_modify_item>& pdu_sessions =
      ng_request->pdu_session_res_to_modify_list;
  for (const cu_cp_drbs_subject_to_status_transfer_item& ngap_drb : sn_status->drbs_subject_to_status_transfer_list) {
    const up_drb_context& drb_ctx = ue->get_up_resource_manager().get_drb_context(ngap_drb.drb_id);
    pdu_session_id_t      psi     = drb_ctx.pdu_session_id;

    if (not pdu_sessions.contains(psi)) {
      e1ap_pdu_session_res_to_modify_item ps_item;
      ps_item.pdu_session_id = psi;
      pdu_sessions.emplace(psi, ps_item);
    }
    e1ap_pdu_session_res_to_modify_item& ps_item = pdu_sessions[psi];
    e1ap_drb_to_modify_item_ng_ran       drb_item;
    drb_item.drb_id = ngap_drb.drb_id;
    drb_item.pdcp_sn_status_info.emplace();

    drb_item.pdcp_sn_status_info->pdcp_status_transfer_ul.count_value.hfn     = ngap_drb.drb_status_ul.ul_count.hfn;
    drb_item.pdcp_sn_status_info->pdcp_status_transfer_ul.count_value.pdcp_sn = ngap_drb.drb_status_ul.ul_count.sn;

    drb_item.pdcp_sn_status_info->pdcp_status_transfer_dl.hfn     = ngap_drb.drb_status_dl.dl_count.hfn;
    drb_item.pdcp_sn_status_info->pdcp_status_transfer_dl.pdcp_sn = ngap_drb.drb_status_dl.dl_count.sn;

    drb_item.pdcp_cfg.emplace();
    fill_e1ap_drb_pdcp_config(*drb_item.pdcp_cfg, drb_ctx.pdcp_cfg);
    drb_item.pdcp_cfg->pdcp_reest = true;

    ps_item.drb_to_modify_list_ng_ran.emplace(drb_item.drb_id, drb_item);
  }
}

bool inter_cu_cho_target_execution_routine::initialize_reconfiguration_timeout()
{
  std::optional<std::chrono::milliseconds> t304_ms = ue->get_rrc_ue()->get_cell_context().timers.t304;
  if (!t304_ms.has_value()) {
    report_fatal_error("T304 not configured in cell context");
  }

  reconf_timeout =
      t304_ms.value() + std::chrono::milliseconds{/*We add 1s of extra time for the UE to reestablish*/ 1000};

  return true;
}

cu_cp_path_switch_request inter_cu_cho_target_execution_routine::fill_path_switch_request()
{
  cu_cp_path_switch_request path_switch_request;
  path_switch_request.ue_index              = execution_ctxt.ue_index;
  path_switch_request.source_amf_ue_ngap_id = execution_ctxt.amf_ue_id;

  path_switch_request.user_location_info.nr_cgi = ue->get_rrc_ue()->get_cell_context().cgi;
  path_switch_request.user_location_info.tai    = {ue->get_rrc_ue()->get_cell_context().cgi.plmn_id,
                                                    ue->get_rrc_ue()->get_cell_context().tac};

  path_switch_request.supported_enc_algos = ue->get_security_manager().get_security_context().supported_enc_algos;
  path_switch_request.supported_int_algos = ue->get_security_manager().get_security_context().supported_int_algos;

  for (const auto& pdu_session_item : execution_ctxt.pdu_session_res_admitted_list) {
    cu_cp_pdu_session_res_to_be_switched_dl_item pdu_session_to_be_switched_item;
    pdu_session_to_be_switched_item.pdu_session_id     = pdu_session_item.pdu_session_id;
    pdu_session_to_be_switched_item.dl_ngu_up_tnl_info = pdu_session_item.dl_ngu_up_tnl_info;
    for (const auto& qos_flow : pdu_session_item.qos_flows_setup_list) {
      pdu_session_to_be_switched_item.qos_flow_accepted_list.push_back(qos_flow.qos_flow_id);
    }

    path_switch_request.pdu_session_res_to_be_switched_dl_list.push_back(pdu_session_to_be_switched_item);
  }

  path_switch_request.pdu_session_res_failed_to_setup_list_ps_req =
      execution_ctxt.pdu_session_failed_to_setup_list;

  return path_switch_request;
}
