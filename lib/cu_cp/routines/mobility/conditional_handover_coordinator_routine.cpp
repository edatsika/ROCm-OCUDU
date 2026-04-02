// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "conditional_handover_coordinator_routine.h"
#include "../../du_processor/du_processor_repository.h"
#include "../../mobility_manager/mobility_manager_helpers.h"
#include "../../ue_manager/ue_manager_impl.h"
#include "ocudu/xnap/xnap_handover.h"
#include "conditional_handover_reconfiguration_routine.h"
#include "intra_cu_handover_routine.h"

using namespace ocudu;
using namespace ocudu::ocucp;

conditional_handover_coordinator_routine::conditional_handover_coordinator_routine(
    const cu_cp_intra_cu_cho_request& request_,
    du_processor_repository&          du_db_,
    cu_cp_impl_interface&             cu_cp_handler_,
    ue_manager&                       ue_mng_,
    mobility_manager&                 mobility_mng_,
    ngap_repository&                  ngap_db_,
    xnap_repository*                  xnap_db_,
    ocudulog::basic_logger&           logger_) :
  request(request_),
  du_db(du_db_),
  cu_cp_handler(cu_cp_handler_),
  ue_mng(ue_mng_),
  mobility_mng(mobility_mng_),
  ngap_db(ngap_db_),
  xnap_db(xnap_db_),
  logger(logger_)
{
}

async_task<void> conditional_handover_coordinator_routine::release_prepared_targets()
{
  return launch_async(
      [this, idx = size_t{0}, result = cu_cp_ue_context_release_complete{}, cmd = cu_cp_ue_context_release_command{}](
          coro_context<async_task<void>>& ctx) mutable {
        CORO_BEGIN(ctx);
        for (idx = 0; idx < prepared_cho_targets.size(); ++idx) {
          cmd                      = {};
          cmd.ue_index             = prepared_cho_targets[idx].target_ue_index;
          cmd.cause                = ngap_cause_radio_network_t::unspecified;
          cmd.requires_rrc_release = false;
          CORO_AWAIT_VALUE(result, cu_cp_handler.handle_ue_context_release_command(cmd));
        }
        CORO_RETURN();
      });
}

std::vector<async_task<cu_cp_intra_cu_handover_response>> conditional_handover_coordinator_routine::build_prep_tasks()
{
  std::vector<async_task<cu_cp_intra_cu_handover_response>> tasks;

  // Only handle intra-CU candidates (no xnc_index set).
  size_t intra_cu_count = 0;
  for (const auto& target : request.targets) {
    if (!target.xnc_index.has_value()) {
      ++intra_cu_count;
    }
  }
  tasks.reserve(intra_cu_count);
  prep_target_du_indices.reserve(intra_cu_count);

  size_t cond_recfg_id_counter = 0;
  for (const auto& target : request.targets) {
    ++cond_recfg_id_counter;
    if (target.xnc_index.has_value()) {
      // Inter-CU candidate — handled by build_inter_cu_prep_tasks().
      continue;
    }

    cu_cp_intra_cu_handover_request req;
    req.source_ue_index = request.source_ue_index;
    req.target_du_index = target.du_index;
    req.target_pci      = target.pci;
    req.cgi             = target.cgi;
    req.cho_preparation.emplace();
    req.cho_preparation->cond_recfg_id = static_cast<cond_recfg_id_t>(cond_recfg_id_counter);

    const du_index_t target_du = target.du_index;
    byte_buffer      sib1      = du_db.get_du_processor(target_du).get_mobility_handler().get_packed_sib1(target.cgi);

    prep_target_du_indices.push_back(target_du);
    tasks.push_back(
        launch_async<intra_cu_handover_routine>(req,
                                                sib1,
                                                du_db.get_du_processor(request.source_du_index).get_f1ap_handler(),
                                                du_db.get_du_processor(target_du).get_f1ap_handler(),
                                                cu_cp_handler,
                                                ue_mng,
                                                mobility_mng,
                                                logger));
  }

  return tasks;
}

std::vector<async_task<cu_cp_cho_candidate>>
conditional_handover_coordinator_routine::build_inter_cu_prep_tasks()
{
  std::vector<async_task<cu_cp_cho_candidate>> tasks;

  if (xnap_db == nullptr) {
    return tasks;
  }

  cu_cp_ue* ue = ue_mng.find_du_ue(request.source_ue_index);
  if (ue == nullptr) {
    logger.warning("ue={}: CHO inter-CU preparation skipped: source UE not found", request.source_ue_index);
    return tasks;
  }

  ngap_interface* ngap = ngap_db.find_ngap(ue->get_ue_context().plmn);
  if (ngap == nullptr) {
    logger.warning("ue={}: CHO inter-CU preparation skipped: NGAP not found", request.source_ue_index);
    return tasks;
  }

  const ngap_context_t& ngap_ctxt = ngap->get_ngap_context();
  std::optional<guami_t> served_guami;
  for (const auto& guami : ngap_ctxt.served_guami_list) {
    if (guami.plmn == ue->get_ue_context().plmn) {
      served_guami = guami;
      break;
    }
  }
  if (!served_guami.has_value()) {
    logger.warning("ue={}: CHO inter-CU preparation skipped: GUAMI not found for plmn={}",
                   request.source_ue_index,
                   ue->get_ue_context().plmn);
    return tasks;
  }

  amf_ue_id_t source_amf_ue_id = ngap->get_amf_ue_id(request.source_ue_index);
  if (source_amf_ue_id == amf_ue_id_t::invalid) {
    logger.warning("ue={}: CHO inter-CU preparation skipped: invalid AMF UE ID", request.source_ue_index);
    return tasks;
  }

  for (size_t i = 0; i < request.targets.size(); ++i) {
    const auto& target = request.targets[i];
    if (!target.xnc_index.has_value()) {
      continue;
    }

    xnap_interface* xnap = xnap_db->find_xnap(*target.xnc_index);
    if (xnap == nullptr) {
      logger.warning("ue={}: CHO inter-CU candidate skipped: XNAP not found for xnc_index={}",
                     request.source_ue_index,
                     *target.xnc_index);
      continue;
    }

    xnap_handover_request ho_request = generate_xnap_handover_request(
        request.source_ue_index,
        target.cgi,
        served_guami.value(),
        source_amf_ue_id,
        ue->get_ue_ambr(),
        ue->get_security_manager().get_security_context(),
        ue->get_up_resource_manager().get_pdu_sessions_map(),
        ue->get_rrc_ue()->get_rrc_ue_control_message_handler().get_packed_handover_preparation_message(),
        ue->get_location_manager().get_location_reporting_request());

    ho_request.is_conditional_handover = true;
    ho_request.cho_timeout             = request.timeout;

    // Pre-populate candidate with input fields; response fields filled in by the wrapper below.
    cu_cp_cho_candidate cand         = {};
    cand.cond_recfg_id               = static_cast<cond_recfg_id_t>(i + 1);
    cand.target_pci                  = target.pci;
    cand.target_cgi                  = target.cgi;
    cand.xnc_index                   = target.xnc_index;

    tasks.push_back(launch_async(
        [cand, xnap, ho_request, resp = xnap_handover_preparation_response{}](
            coro_context<async_task<cu_cp_cho_candidate>>& ctx) mutable {
          CORO_BEGIN(ctx);
          CORO_AWAIT_VALUE(resp, xnap->handle_handover_request_required(ho_request));
          if (resp.success) {
            cand.prepared_rrc_recfg          = std::move(resp.packed_rrc_recfg);
            cand.rrc_reconfig_transaction_id = resp.rrc_transaction_id;
            cand.peer_xnap_ue_id             = resp.peer_xnap_ue_id;
          }
          CORO_RETURN(cand);
        }));
  }

  return tasks;
}

void conditional_handover_coordinator_routine::operator()(coro_context<async_task<cu_cp_intra_cu_cho_response>>& ctx)
{
  CORO_BEGIN(ctx);

  logger.debug("ue={}: \"{}\" started...", request.source_ue_index, name());

  if (request.source_ue_index == ue_index_t::invalid || request.source_du_index == du_index_t::invalid ||
      request.targets.empty()) {
    logger.warning("CHO coordinator request is invalid");
    CORO_EARLY_RETURN(response);
  }

  source_ue = ue_mng.find_du_ue(request.source_ue_index);
  if (source_ue == nullptr || !source_ue->get_cho_context().has_value()) {
    logger.warning("ue={}: CHO coordinator failed. Source UE/CHO context missing", request.source_ue_index);
    CORO_EARLY_RETURN(response);
  }

  // Pre-start: verify CHO measurement config can be generated before preparing any targets.
  {
    rrc_ue_transfer_context source_rrc_context = source_ue->get_rrc_ue()->get_transfer_context();
    std::vector<pci_t>      candidate_target_pcis;
    for (const auto& target : request.targets) {
      candidate_target_pcis.push_back(target.pci);
    }
    if (!source_ue->get_rrc_ue()
             ->generate_meas_config(source_rrc_context.meas_cfg, true, candidate_target_pcis)
             .has_value()) {
      logger.warning("ue={}: CHO aborted. No conditional trigger configs match UE capabilities",
                     request.source_ue_index);
      CORO_EARLY_RETURN(response);
    }
  }

  // Phase 1a: Intra-CU CHO Preparation — launch all intra-CU candidates in parallel.
  source_ue->get_cho_context()->state = cu_cp_ue_cho_context::state_t::targets_preparation;
  CORO_AWAIT_VALUE(prep_responses, when_all(build_prep_tasks()));

  // No need to re-fetch source UE here: this routine runs on the source UE's
  // per-UE fifo_task_scheduler, which serializes same-UE procedures and
  // prevents concurrent UE release/procedure execution for this UE.
  ocudu_assert(source_ue != nullptr, "ue={}: Unexpected null source UE after CHO preparation", request.source_ue_index);

  // Phase 1b: Inter-CU CHO Preparation — launch all inter-CU candidates in parallel.
  CORO_AWAIT_VALUE(inter_cu_prep_responses, when_all(build_inter_cu_prep_tasks()));

  // Process intra-CU preparation responses.
  {
    size_t intra_cu_idx = 0;
    for (size_t i = 0; i < request.targets.size(); ++i) {
      const auto& target = request.targets[i];
      if (target.xnc_index.has_value()) {
        continue;
      }
      auto& prep_response = prep_responses[intra_cu_idx];
      ++intra_cu_idx;

      if (!prep_response.success || !prep_response.cho_preparation_result.has_value()) {
        logger.warning(
            "ue={}: CHO intra-CU candidate preparation failed for target_pci={}", request.source_ue_index, target.pci);
        continue;
      }

      cu_cp_cho_candidate candidate         = {};
      candidate.cond_recfg_id               = static_cast<cond_recfg_id_t>(i + 1);
      candidate.target_pci                  = target.pci;
      candidate.target_cgi                  = target.cgi;
      candidate.target_ue_index             = prep_response.cho_preparation_result->target_ue_index;
      candidate.prepared_rrc_recfg          = std::move(prep_response.cho_preparation_result->packed_rrc_recfg);
      candidate.rrc_reconfig_transaction_id = prep_response.cho_preparation_result->transaction_id;
      candidate.bearer_context_mod_request.ng_ran_bearer_context_mod_request =
          std::move(prep_response.cho_preparation_result->ng_ran_bearer_context_mod_request);

      if (candidate.target_ue_index != request.source_ue_index) {
        prepared_cho_targets.push_back({candidate.target_ue_index, target.du_index});
      }

      if (source_ue->get_cho_context().has_value()) {
        const ue_index_t target_ue_idx = candidate.target_ue_index;
        source_ue->get_cho_context()->candidates.push_back(std::move(candidate));

        // Set source_ue_idx on target UE so the source UE can be fetched directly.
        if (target_ue_idx != request.source_ue_index) {
          auto* target_ue_ptr = ue_mng.find_du_ue(target_ue_idx);
          if (target_ue_ptr != nullptr) {
            target_ue_ptr->get_cho_context().emplace();
            target_ue_ptr->get_cho_context()->role            = cu_cp_ue_cho_context::role_t::target;
            target_ue_ptr->get_cho_context()->source_ue_index = request.source_ue_index;
          }
        }
      }
    }
  }

  // Process inter-CU preparation responses.
  for (auto& candidate : inter_cu_prep_responses) {
    if (candidate.peer_xnap_ue_id == peer_xnap_ue_id_t::invalid) {
      logger.warning("ue={}: CHO inter-CU candidate preparation failed for target_pci={}",
                     request.source_ue_index,
                     candidate.target_pci);
      continue;
    }
    if (source_ue->get_cho_context().has_value()) {
      source_ue->get_cho_context()->candidates.push_back(std::move(candidate));
    }
  }

  // Finalize CHO preparation state on source UE.
  source_ue = ue_mng.find_du_ue(request.source_ue_index);
  if (source_ue == nullptr || !source_ue->get_cho_context().has_value() ||
      source_ue->get_cho_context()->candidates.empty()) {
    logger.warning("ue={}: CHO coordinator failed. No prepared candidates", request.source_ue_index);
    CORO_EARLY_RETURN(response);
  }

  // Phase 2: CHO Execution.
  source_ue->get_cho_context()->state    = cu_cp_ue_cho_context::state_t::rrc_reconfiguration;
  cho_reconfig_request.source_ue_index   = request.source_ue_index;
  cho_reconfig_request.timeout           = request.timeout;
  cho_reconfig_request.t1_thres_override = request.t1_thres_override;
  CORO_AWAIT_VALUE(cho_reconfig_result,
                   launch_async<conditional_handover_reconfiguration_routine>(
                       cho_reconfig_request,
                       *source_ue,
                       du_db.get_du_processor(request.source_du_index).get_f1ap_handler(),
                       cu_cp_handler,
                       cu_cp_handler,
                       ue_mng,
                       logger));
  if (!cho_reconfig_result) {
    logger.warning("ue={}: CHO coordinator failed. Reconfiguration phase failed", request.source_ue_index);
    CORO_EARLY_RETURN(response);
  }

  // Start cancellation timer. Fires conditional_handover_cancellation_routine if UE never executes CHO.
  cu_cp_handler.initialize_cho_execution_timer(request.source_ue_index, request.timeout);

  // Phase 3: CHO completion is handled asynchronously by conditional_handover_source_routine after Access Success,
  // or by inter_cu_cho_source_completion_routine after HandoverSuccess from the winning target CU-CP.

  logger.debug("ue={}: \"{}\" finished successfully", source_ue->get_ue_index(), name());
  response.success = true;
  CORO_RETURN(response);
}
