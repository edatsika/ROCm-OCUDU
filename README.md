# ROCm-OCUDU

OCUDU (https://gitlab.com/ocudu/ocudu) is a permissively-licensed, open-source 5G (and beyond) CU/DU project designed for commercial deployment and broad industry adoption, as well as advanced research and development. OCUDU is a complete radio access network (RAN) solution compliant with 3GPP and O-RAN Alliance specifications and includes the full L1/2/3 stack with minimal external dependencies. OCUDU is governed under the Linux Foundation. For further information, please check [README-OCUDU](./README-OCUDU.md).

This repository is based on OCUDU and aims to explore GPU acceleration for 5G workloads using AMD ROCm.

## License

This project is licensed under the BSD 3-Clause Open MPI variant License – see the [LICENSE](./LICENSE) file for details.
Portions of this software may implement 3GPP specifications, which may be subject to additional licensing requirements.

## Testing LDPC Decoder

Current version implements **Layered LDPC decoding kernels** for **5G PHY** workloads using **AMD ROCm 7+**.

### Decoding Pipeline
The implementation follows a **Layered Iterative Message-Passing** approach. Instead of updating the entire matrix at once, the parity check matrix is processed in layers to achieve faster convergence. For each iteration and each layer, the fused kernel executes:

1.  **Variable-to-Check Update:** Processes messages from variable nodes to check nodes for the current layer.
2.  **Check-to-Variable Update:** Computes parity constraints and updates messages back to variable nodes.
3.  **Soft-bit (Posterior LLR) Update:** Updates the soft bits before moving to the next layer or iteration.

This design was chosen to simplify validation against a **CPU reference** and allow independent debugging of each step of the layered update.

### Requirements

To build and run this project, you need the following environment:

*   **Operating System:** Ubuntu 24.04
*   **ROCm Stack:** Version **7.0 or higher** (tested on ROCm 7.x)
*   **Compiler:** `hipcc` (part of the ROCm toolkit)
*   **GPU:** AMD Radeon or Instinct GPU with **ROCm support** (e.g., RDNA 2/3 or CDNA architectures)
*   **Dependencies:** Based on the **ocudu** framework (ensure all ocudu-specific dependencies are met)

### Hardware & Environment
*   **Tested on:** AMD Radeon **RX 6500 XT** (Target: `gfx1030`)
*   **ROCm Version:** 7.0+
*   **Override Note:** If the GPU is not officially supported by your ROCm version, you may need to set the following environment variable before running:
    ```bash
    export HSA_OVERRIDE_GFX_VERSION=10.3.0
    ```
### How to Build
A helper script (`build_ocudu.sh`) is provided to automate the environment setup and build process. 
It adds the current user to the render and video groups, sets gfx1030 support, configures CMake with ROCm 7+ paths and compiles the project using hipcc.
After a successful build, go to the benchmark directory `build/tests/benchmarks/phy/upper/channel_coding/ldpc` and run `./ldpc_decoder_benchmark`.


### LDPC Decoder Performance Analysis (AMD 6500 XT - RDNA2)
*   **Initial Benchmark Results**
Before implementing batching and asynchronous streams, we measured the performance of the Fused HIP Kernel on an AMD Radeon RX 6500 XT (16 CUs, 4GB VRAM) compared to the generic CPU implementation.

| Configuration | Device | 50th | 75th | 90th | 99th | 99.9th | Speedup (50th) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| **BG1, LS=384, R=0.917** | GPU | 19.4 | 19.3 | 19.0 | 5.8 | 0.0 | **4.3x** |
| | CPU | 4.5 | 4.4 | 4.4 | 4.0 | 3.2 | |
| **BG1, LS=384, R=0.333** | GPU | 5.2 | 5.1 | 3.7 | 3.6 | 1.8 | **5.7x** |
| | CPU | 0.9 | 0.9 | 0.9 | 0.9 | 0.9 | |
| **BG2, LS=384, R=0.833** | GPU | 11.6 | 11.4 | 10.7 | 3.9 | 2.7 | **2.5x** |
| | CPU | 4.5 | 4.4 | 4.4 | 4.4 | 4.3 | |
| **BG2, LS=384, R=0.200** | GPU | 3.0 | 2.9 | 2.9 | 1.9 | 1.0 | **4.2x** |
| | CPU | 0.7 | 0.7 | 0.7 | 0.6 | 0.6 | |

While the GPU significantly outperforms the CPU in median throughput (50th percentile), it exhibits performance drops at the tail percentiles (99.9th). 
This behavior confirms that the current implementation is bottlenecked by synchronous host-device communication, leading to complete stalls (0.0 Mbps) during kernel dispatching.
The GPU performs better in scenarios with larger code lengths (e.g., cb_len=25344). This indicates that as the computational workload per codeword increases, the data transfer overhead is better "amortized" over the total execution time. While CPU maintains stable and predictable performance (dropping only from 4.5 to 3.2 Mbps at the 99.9th percentile), the GPU experiences a reduction from 19.4 to 0.0 Mbps at the 99.9th percentile. The "0.0 Mbps" result indicates intermittent system stalls caused by synchronous calls (hipMemcpy), where the CPU blocks wait for the GPU and vice versa.

1.  **Host-device sync overhead:** Processing one codeword at a time forces the GPU to wait for the CPU to dispatch the next command. Profiling shows many idle gaps where the overhead of launching kernels exceeds the actual computation time.
2.  **Hardware Underutilization:** With a lifting size of 384, the kernel occupies only one block / one CU (out of 16 CUs on the RX 6500 XT GPU).
3.  **Synchronous Memory Stalls:** The drop to 0.0 Mbps at the 99.9th percentile indicates severe latency spikes caused by synchronous hipMemcpy calls and OS scheduling, preventing a continuous data pipeline.


<p align="center">
  <img src="docs/traces/trace-fused-L-384-GPU.png" alt="Perfetto Trace" width="900">
  <br>
  <em>Figure 1: Perfetto trace showing scattered kernel execution (blue) and host-side gaps.</em>
</p>

Trace generated as follows:
   ```bash
   cd ~/ROCm-OCUDU/tests/benchmarks/phy/upper/channel_coding/ldpc/
   rocprofv3 --hip-trace --kernel-trace --output-format pftrace -- ./ldpc_decoder_benchmark -L 384
   ```
### Next Steps
- **Multi-codeword batching:** Combine multiple codewords (e.g., batch size = 32 or 64) into a single kernel launch to fill all 16 CUs and hide CPU launch latency.
- **Asynchronous pipeline (HIP Streams):** Use hipStream_t and hipMemcpyAsync to overlap data transfers with kernel execution. While the GPU is decoding batch n, the PCIe bus will be transferring batch N+1.
- **Local data share (LDS) Optimization:** Move LLR data from registers to LDS in order to allow more batches to run concurrently per CU.

