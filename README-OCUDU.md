# The OCUDU Project

[![Pipeline](https://gitlab.com/ocudu/ocudu/badges/main/pipeline.svg)](https://gitlab.com/ocudu/ocudu/-/pipelines?scope=branches)
[![Documentation](https://img.shields.io/badge/docs-built-green?logo=docusaurus)](https://docs.ocudu.org)
![Code](https://img.shields.io/badge/code-C++17-informational)
![Build](https://img.shields.io/badge/build-CMake-informational)
[![License](https://img.shields.io/badge/license-BSD--3--Clause--Open--MPI-blue)](https://spdx.org/licenses/BSD-3-Clause-Open-MPI.html)
[![OpenSSF Best Practices](https://www.bestpractices.dev/projects/11899/badge)](https://www.bestpractices.dev/projects/11899)
[![Coverage](https://gitlab.com/ocudu/ocudu/badges/main/coverage.svg?min_good=98&min_acceptable=60)](https://docs.ocudu.org/coverage/index.html)

<img src="https://srs.io/wp-content/uploads/ocudu_color.png" alt="image" width="50%"/>

OCUDU is a permissively-licensed, open-source 5G (and beyond) CU/DU project designed for commercial deployment and broad industry adoption, as well as advanced research and development. OCUDU is a complete radio access network (RAN) solution compliant with 3GPP and O-RAN Alliance specifications and includes the full L1/2/3 stack with minimal external dependencies. OCUDU is governed under the Linux Foundation.

This repository contains the RAN source code, architecture documentation, and tooling.

For general information, visit https://ocudu.org.

## Getting started

Build instructions and user guides are provided in the [OCUDU User manual](https://docs.ocudu.org/user_manual/installation/). We also host an extensive selection of tutorials.

## Documentation

Complete project documentation including developer guideline, configuration reference, tutorials, etc. is
hosted in [this](https://gitlab.com/ocudu/ocudu_docs) repo. The most recent version of the documentation
is available [here](https://docs.ocudu.org).

## Contributing

Our project welcomes contributions from any member of our community. To get started contributing,
please take a look at the [Developer Guide](https://docs.ocudu.org/dev_guide/contributing_guide/) with detailed instructions on how to best engange with us.

## Governance

The OCUDU project is governed by a framework of principles, values, policies and processes to help our community and constituents towards our shared goals.

The [Governance](https://gitlab.com/ocudu/Governance) repo is used by the Technical Steering Committee, which oversees governance of the project.

## License

This project is licensed under the BSD 3-Clause Open MPI variant License – see the [LICENSE](./LICENSE) file for details.
Portions of this software may implement 3GPP specifications, which may be subject to additional licensing requirements.

## Testing (LDPC Decoder Only)

This project is based on **ocudu** and implements **Layered LDPC decoding kernels** for **5G PHY** workloads using **AMD ROCm 7+**.

### Decoding Pipeline
The implementation follows a **Layered Iterative Message-Passing** approach. Instead of updating the entire matrix at once, the parity check matrix is processed in layers to achieve faster convergence. For each iteration and each layer, the following kernels execute:

1.  **Variable-to-Check Update:** Processes messages from variable nodes to check nodes for the current layer.
2.  **Check-to-Variable Update:** Computes parity constraints and updates messages back to variable nodes.
3.  **Soft-bit (Posterior LLR) Update:** Updates the soft bits before moving to the next layer or iteration.

This design was chosen to simplify validation against a **CPU reference** and allow independent debugging of each step of the layered update.

### Performance Observations
The current implementation serves as a functional baseline, splitting the layered update into three distinct kernels for independent debugging of the message-passing logic. The throughput is currently bound by the serial nature of kernel launches within the layered loops.

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

### Next Steps
- [ ] **Kernel Fusion:** Merge the three kernels per layer into a single launch to drastically reduce driver overhead.
- [ ] **LDS (Local Data Share) Optimization:** Utilize AMD's **LDS** to store and exchange LLRs/messages, minimizing slow global memory transactions.
- [ ] **Profiling:** Quantify the exact cache hit rates and occupancy.
                 
