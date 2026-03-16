#
# Copyright 2021-2026 Software Radio Systems Limited
#
# By using this file, you agree to the terms and conditions set
# forth in the LICENSE file which can be found at the top level of
# the distribution.
#

set(SOURCES_sidekiq
        radio_config_sidekiq_validator.cpp
        sidekiq_helper_functions.cpp
        radio_factory_sidekiq_impl.cpp
        radio_session_sidekiq_impl.cpp
        radio_sidekiq_card.cpp
        radio_sidekiq_rx_stream.cpp
        radio_sidekiq_tx_stream.cpp)
add_library(ocudu_radio_sidekiq STATIC ${SOURCES_sidekiq})

target_include_directories(ocudu_radio_sidekiq PRIVATE ${Sidekiq_INCLUDE_DIRS})
target_link_libraries(ocudu_radio_sidekiq
        ${Sidekiq_LIBRARIES}
        ocudulog
        ocuduvec)

install(TARGETS ocudu_radio_sidekiq EXPORT ocudu_export)