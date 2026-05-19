# SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
# SPDX-License-Identifier: BSD-3-Clause-Open-MPI
# Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.


set(ldpc_sources
        ldpc_graph_impl.cpp
        ldpc_luts_impl.cpp
        ldpc_encoder_impl.cpp
        ldpc_encoder_generic.cpp
        ldpc_decoder_impl.cpp
        ldpc_decoder_generic.cpp
        ldpc_rate_matcher_impl.cpp
        ldpc_rate_dematcher_impl.cpp
        ldpc_segmenter_tx_impl.cpp
        ldpc_segmenter_rx_impl.cpp
	# edatsika
        # Add HIP implementation files
        ldpc_decoder_hip.cpp             
        batched_fused_kernel.hip         
        ldpc_syndrome_check_kernel.hip    
        )

if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    list(APPEND ldpc_sources
            ldpc_encoder_avx2.cpp
            ldpc_decoder_avx2.cpp
            ldpc_decoder_avx512.cpp
            ldpc_rate_dematcher_avx2_impl.cpp
            ldpc_rate_dematcher_avx512_impl.cpp)
    set_source_files_properties(ldpc_rate_dematcher_avx2_impl.cpp PROPERTIES COMPILE_OPTIONS "-mavx2;")
    set_source_files_properties(ldpc_rate_dematcher_avx512_impl.cpp PROPERTIES COMPILE_OPTIONS "-mavx512f;-mavx512bw;-mavx512vbmi;")
    set_source_files_properties(ldpc_encoder_avx2.cpp PROPERTIES COMPILE_OPTIONS "-mavx2;")
    set_source_files_properties(ldpc_decoder_avx2.cpp PROPERTIES COMPILE_OPTIONS "-mavx2;")
    set_source_files_properties(ldpc_decoder_avx512.cpp PROPERTIES COMPILE_OPTIONS "-mavx512f;-mavx512bw;")
endif (${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")

if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    list(APPEND ldpc_sources ldpc_encoder_neon.cpp)
    list(APPEND ldpc_sources ldpc_decoder_neon.cpp)
    list(APPEND ldpc_sources ldpc_rate_dematcher_neon_impl.cpp)
endif (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")

add_library(ocudu_ldpc STATIC ${ldpc_sources})

# edatsika
#set_target_properties(ocudu_ldpc PROPERTIES LINKER_LANGUAGE HIP)

#edatsika
target_include_directories(ocudu_ldpc PUBLIC 
    ${PROJECT_SOURCE_DIR}/lib
    ${PROJECT_SOURCE_DIR}       # Allow #include "ocudu/..."
    ${CMAKE_CURRENT_SOURCE_DIR}
    /opt/rocm/include
)


#target_link_libraries(ocudu_ldpc log_likelihood_ratio)
# edatsika linking with HIP
target_link_libraries(ocudu_ldpc PUBLIC 
    hip::host 
    hip::device 
    log_likelihood_ratio
)

# Added for batched fused kernel implementation as new decoder type
set_source_files_properties(ldpc_decoder_hip.cpp PROPERTIES LANGUAGE HIP)
set_source_files_properties(batched_fused_kernel.hip PROPERTIES LANGUAGE HIP)
set_source_files_properties(ldpc_syndrome_check_kernel.hip PROPERTIES LANGUAGE HIP)
set_source_files_properties(ldpc_decoder_impl.cpp PROPERTIES LANGUAGE HIP) 
target_compile_definitions(ocudu_ldpc PUBLIC GPU_HIP) # or private?
#target_compile_definitions(ocudu_ldpc PUBLIC BATCH_SIZE=256)
