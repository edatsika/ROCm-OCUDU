#
# Copyright 2021-2024 Software Radio Systems Limited
#
# By using this file, you agree to the terms and conditions set
# forth in the LICENSE file which can be found at the top level of
# the distribution.
#

FIND_PACKAGE(PkgConfig REQUIRED)

IF(NOT Sidekiq_FOUND)

    FIND_PATH(
            Sidekiq_INCLUDE_DIRS
            NAMES sidekiq_api.h
            HINTS $ENV{Sidekiq_DIR}/inc
            $ENV{Sidekiq_DIR}/sidekiq_core/inc
            PATHS /usr/local/include
            /usr/include
    )

    FIND_LIBRARY(
            Sidekiq_LIBRARY
            NAMES sidekiq__x86_64.gcc
            HINTS $ENV{Sidekiq_DIR}/lib
            PATHS /usr/local/lib
            /usr/lib
            /usr/lib/x86_64-linux-gnu
            /usr/local/lib64
            /usr/local/lib32
    )

    FIND_LIBRARY(
            Sidekiq_LIBRARY_GLIB
            NAMES libglib-2.0.a
            HINTS $ENV{Sidekiq_DIR}/lib/support/x86_64.gcc/usr/lib/epiq
            PATHS /usr/local/lib
            /usr/lib
            /usr/lib/epiq
            /usr/lib/x86_64-linux-gnu
            /usr/local/lib64
            /usr/local/lib32
    )

    FIND_LIBRARY(
            Sidekiq_LIBRARY_USB
            NAMES libusb-1.0.a
            HINTS $ENV{Sidekiq_DIR}/lib/support/x86_64.gcc/usr/lib/epiq
            PATHS /usr/local/lib
            /usr/lib
            /usr/lib/epiq
            /usr/lib/x86_64-linux-gnu
            /usr/local/lib64
            /usr/local/lib32
    )

    FIND_LIBRARY(
            Sidekiq_LIBRARY_TIRPC
            NAMES libtirpc.so.3
            PATHS /usr/local/lib
            /usr/lib64
            /usr/lib/epiq
            /usr/lib/x86_64-linux-gnu
            /usr/local/lib64
            /usr/local/lib32
    )

    set(Sidekiq_LIBRARIES ${Sidekiq_LIBRARY} ${Sidekiq_LIBRARY_GLIB} ${Sidekiq_LIBRARY_USB} ${Sidekiq_LIBRARY_TIRPC})

    message(STATUS "Sidekiq LIBRARIES " ${Sidekiq_LIBRARIES})
    message(STATUS "Sidekiq INCLUDE DIRS " ${Sidekiq_INCLUDE_DIRS})

    INCLUDE(FindPackageHandleStandardArgs)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(Sidekiq DEFAULT_MSG Sidekiq_LIBRARIES Sidekiq_INCLUDE_DIRS)
    MARK_AS_ADVANCED(Sidekiq_LIBRARIES Sidekiq_INCLUDE_DIRS)

ENDIF(NOT Sidekiq_FOUND)
