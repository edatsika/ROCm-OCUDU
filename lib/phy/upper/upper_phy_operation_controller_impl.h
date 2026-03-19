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

#include "ocudu/phy/upper/downlink_processor.h"
#include "ocudu/phy/upper/rx_buffer_pool.h"
#include "ocudu/phy/upper/uplink_processor.h"
#include "ocudu/phy/upper/upper_phy_operation_controller.h"
#include "ocudu/phy/upper/upper_phy_operational_status_change_request_notifier.h"

namespace ocudu {

/// Upper PHY operation controller implementation that manages the start/stop of an upper PHY.
class upper_phy_operation_controller_impl : public upper_phy_operation_controller
{
public:
  upper_phy_operation_controller_impl(upper_phy_operational_status_change_request_notifier& request_notifier_,
                                      rx_buffer_pool_controller&                            rx_buf_pool_,
                                      downlink_processor_pool&                              dl_processor_pool_,
                                      uplink_processor_pool&                                ul_processor_pool_) :
    request_notifier(request_notifier_),
    rx_buf_pool(rx_buf_pool_),
    dl_processor_pool(dl_processor_pool_),
    ul_processor_pool(ul_processor_pool_)
  {
  }

  // See interface for documentation.
  void start() override { request_notifier.on_start_request(); }

  // See interface for documentation.
  void stop() override
  {
    request_notifier.on_stop_request();

    rx_buf_pool.stop();
    dl_processor_pool.stop();
    ul_processor_pool.stop();
  }

private:
  upper_phy_operational_status_change_request_notifier& request_notifier;
  rx_buffer_pool_controller&                            rx_buf_pool;
  downlink_processor_pool&                              dl_processor_pool;
  uplink_processor_pool&                                ul_processor_pool;
};

} // namespace ocudu
