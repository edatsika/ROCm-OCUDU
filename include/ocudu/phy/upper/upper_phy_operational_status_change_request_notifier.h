/*
 *
 * Copyright 2021-2026 Software Radio Systems Limited
 *
 * By using this file, you agree to the terms and conditions set
 * forth in the LICENSE file which can be found at the top level of
 * the distribution.
 *
 */

#pragma once

namespace ocudu {

/// \brief Upper PHY operational status change request notifier.
///
/// This class encapsulates an intention to transition another component from its current operational status to a
/// desired target status. This class does not enforce the transition itself; it merely describes the request. The
/// actual decision to accept, reject, or defer the request is left to the receiving component or a coordinating
/// controller.
class upper_phy_operational_status_change_request_notifier
{
public:
  virtual ~upper_phy_operational_status_change_request_notifier() = default;

  /// Notifies a start request event.
  virtual void on_start_request() = 0;

  /// Notifies a stop request event.
  virtual void on_stop_request() = 0;
};

} // namespace ocudu
