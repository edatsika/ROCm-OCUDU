// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "csi_report_on_puxch_helpers.h"
#include "ocudu/adt/interval.h"
#include "ocudu/ran/csi_report/csi_report_on_puxch_utils.h"
#include "ocudu/support/error_handling.h"

using namespace ocudu;

static ri_li_cqi_cri_sizes
get_codebook_ri_li_cqi_cri_sizes(const std::monostate&, ri_restriction_type, csi_report_data::ri_type, unsigned)
{
  report_error("Failed to get codebook RI/LI/CRI sizes: invalid codebook configuration.");
}

static ri_li_cqi_cri_sizes get_codebook_ri_li_cqi_cri_sizes(const pmi_codebook_one_port&,
                                                            ri_restriction_type,
                                                            csi_report_data::ri_type,
                                                            unsigned nof_csi_rs_resources)
{
  return {.ri                         = 0,
          .li                         = 0,
          .wideband_cqi_first_tb      = 4,
          .wideband_cqi_second_tb     = 0,
          .subband_diff_cqi_first_tb  = 2,
          .subband_diff_cqi_second_tb = 0,
          .cri                        = log2_ceil(nof_csi_rs_resources)};
}

static ri_li_cqi_cri_sizes get_codebook_ri_li_cqi_cri_sizes(const pmi_codebook_two_port&,
                                                            ri_restriction_type      ri_restriction,
                                                            csi_report_data::ri_type ri,
                                                            unsigned                 nof_csi_rs_resources)
{
  unsigned ri_uint              = ri.value();
  unsigned ri_restriction_count = static_cast<unsigned>(ri_restriction.count());

  return {.ri                         = std::min(1U, log2_ceil(ri_restriction_count)),
          .li                         = log2_ceil(ri_uint),
          .wideband_cqi_first_tb      = 4,
          .wideband_cqi_second_tb     = 0,
          .subband_diff_cqi_first_tb  = 2,
          .subband_diff_cqi_second_tb = 0,
          .cri                        = log2_ceil(nof_csi_rs_resources)};
}

/// Returns the field bit-widths for RI, LI, wideband CQI, and CRI for Type I, single-panel codebook as per
/// TS38.212 Table 6.3.1.1.2-3.
static ri_li_cqi_cri_sizes get_codebook_ri_li_cqi_cri_sizes(const pmi_codebook_typeI_single_panel& pmi_codebook,
                                                            ri_restriction_type                    ri_restriction,
                                                            csi_report_data::ri_type               ri,
                                                            unsigned                               nof_csi_rs_resources)
{
  unsigned nof_csi_antenna_ports = csi_report_get_nof_csi_rs_antenna_ports(pmi_codebook);
  ocudu_assert(nof_csi_antenna_ports == 4, "Only four ports are currently supported.");

  ri_li_cqi_cri_sizes result;

  unsigned ri_uint              = ri.value();
  unsigned ri_restriction_count = static_cast<unsigned>(ri_restriction.count());

  // Calculate RI field size.
  ocudu_assert(ri_restriction.find_lowest(true) >= 0,
               "The RI restriction field (i.e., {}) must have at least one true value.",
               ri_restriction);

  if (nof_csi_antenna_ports == 4) {
    result.ri = std::min(2U, log2_ceil(ri_restriction_count));
  } else {
    result.ri = log2_ceil(ri_restriction_count);
  }

  // Calculate LI field size.
  result.li = std::min(2U, log2_ceil(ri_uint));

  // Wideband CQI for the first TB field size.
  result.wideband_cqi_first_tb = 4;

  // Wideband CQI for the second TB field size.
  if (ri.value() > 4) {
    result.wideband_cqi_second_tb = 4;
  } else {
    result.wideband_cqi_second_tb = 0;
  }

  // Subband differential CQI for the first TB.
  result.subband_diff_cqi_first_tb = 2;

  // Subband differential CQI for the second TB.
  if (ri.value() > 4) {
    result.subband_diff_cqi_second_tb = 2;
  } else {
    result.subband_diff_cqi_second_tb = 0;
  }

  result.cri = log2_ceil(nof_csi_rs_resources);

  return result;
}

ri_li_cqi_cri_sizes ocudu::get_ri_li_cqi_cri_sizes(pmi_codebook_config      pmi_codebook,
                                                   ri_restriction_type      ri_restriction,
                                                   csi_report_data::ri_type ri,
                                                   unsigned                 nof_csi_rs_resources)
{
  // Calculate CRI field size. The number of CSI resources in the corresponding resource set must be at least one and up
  // to 64 (see TS38.331 Section 6.3.2, Information Element \c NZP-CSI-RS-ResourceSet).
  constexpr interval<unsigned, true> nof_csi_res_range(1, 64);
  ocudu_assert(nof_csi_res_range.contains(nof_csi_rs_resources),
               "The number of CSI-RS resources in the resource set, i.e., {} exceeds the valid range {}.",
               nof_csi_rs_resources,
               nof_csi_res_range);

  return std::visit(
      [ri_restriction, ri, nof_csi_rs_resources](const auto& item) {
        return get_codebook_ri_li_cqi_cri_sizes(item, ri_restriction, ri, nof_csi_rs_resources);
      },
      pmi_codebook);
}

namespace {

/// Collects PMI sizes.
struct csi_report_typeI_single_panel_pmi_sizes {
  unsigned i_1_1;
  unsigned i_1_2;
  unsigned i_1_3;
  unsigned i_2;
};

} // namespace

/// Gets PMI sizes for TypeI-SinglePanel, Mode 1 codebook configuration as per TS38.212 Table 6.3.1.1.2-1.
static csi_report_typeI_single_panel_pmi_sizes
csi_report_get_pmi_sizes_typeI_single_panel_mode1(const pmi_codebook_single_panel_info& panel_info,
                                                  csi_report_data::ri_type              ri)
{
  unsigned nof_csi_rs_antenna_ports = 2 * panel_info.n1 * panel_info.n2;
  unsigned N1                       = panel_info.n1;
  unsigned N2                       = panel_info.n2;
  unsigned O1                       = panel_info.o1;
  unsigned O2                       = panel_info.o2;

  if ((ri == 1) && (nof_csi_rs_antenna_ports > 2) && (N2 == 1)) {
    return {log2_ceil(N1 * O1), log2_ceil(N2 * O2), 0, 2};
  }

  if ((ri == 2) && (nof_csi_rs_antenna_ports == 4) && (N2 == 1)) {
    return {log2_ceil(N1 * O1), log2_ceil(N2 * O2), 1, 1};
  }

  if ((ri == 2) && (nof_csi_rs_antenna_ports > 4) && (N2 == 1)) {
    return {log2_ceil(N1 * O1), log2_ceil(N2 * O2), 2, 1};
  }

  if (((ri == 3) || (ri == 4)) && (nof_csi_rs_antenna_ports == 4)) {
    return {log2_ceil(N1 * O1), log2_ceil(N2 * O2), 0, 1};
  }

  report_error("Unhandled case with ri={} nof_csi_rs_antenna_ports={} N2={}.", ri, nof_csi_rs_antenna_ports, N2);
}

static unsigned get_size_pmi(std::monostate, csi_report_data::ri_type)
{
  report_error("Failed to get PMI size: invalid codebook configuration.");
}

static unsigned get_size_pmi(pmi_codebook_one_port, csi_report_data::ri_type)
{
  return 0;
}

static unsigned get_size_pmi(pmi_codebook_two_port, csi_report_data::ri_type ri)
{
  ocudu_assert(ri <= 2, "Invalid rank indicator (i.e., {}).", ri);
  if (ri == 2) {
    return 1;
  }

  return 2;
}

static unsigned get_size_pmi(const pmi_codebook_typeI_single_panel& codebook, csi_report_data::ri_type ri)
{
  ocudu_assert(codebook.mode == pmi_codebook_typeI_mode::one, "Only mode 1 is currently supported.");

  unsigned count = 0;

  csi_report_typeI_single_panel_pmi_sizes sizes =
      csi_report_get_pmi_sizes_typeI_single_panel_mode1(get_single_panel_info(codebook.n1_n2), ri);

  count += sizes.i_1_1;
  count += sizes.i_1_2;
  count += sizes.i_1_3;
  count += sizes.i_2;

  return count;
}

unsigned ocudu::csi_report_get_size_pmi(pmi_codebook_config codebook, csi_report_data::ri_type ri)
{
  return std::visit([ri](const auto& item) { return get_size_pmi(item, ri); }, codebook);
}

csi_report_data::wideband_cqi_type ocudu::csi_report_unpack_wideband_cqi(csi_report_packed packed)
{
  ocudu_assert(packed.size() == 4, "Packed size (i.e., {}) must be 4 bits.", packed.size());
  return packed.extract(0, 4);
}

static csi_report_pmi unpack_pmi(const std::monostate&, const csi_report_packed&, csi_report_data::ri_type)
{
  report_error("Failed to unpack PMI: invalid codebook configuration.");
}

static csi_report_pmi unpack_pmi(const pmi_codebook_one_port&, const csi_report_packed&, csi_report_data::ri_type)
{
  return {};
}

static csi_report_pmi
unpack_pmi(const pmi_codebook_two_port&, const csi_report_packed& packed, csi_report_data::ri_type)
{
  csi_report_pmi::two_antenna_port result;
  result.pmi = packed.extract(0, packed.size());

  return csi_report_pmi{result};
}

static csi_report_pmi
unpack_pmi(pmi_codebook_typeI_single_panel codebook, const csi_report_packed& packed, csi_report_data::ri_type ri)
{
  ocudu_assert(codebook.mode == pmi_codebook_typeI_mode::one, "Only mode 1 is currently supported.");

  unsigned                                        count = 0;
  csi_report_pmi::typeI_single_panel_4ports_mode1 result;

  csi_report_typeI_single_panel_pmi_sizes sizes =
      csi_report_get_pmi_sizes_typeI_single_panel_mode1(get_single_panel_info(codebook.n1_n2), ri);

  result.i_1_1 = packed.extract(count, sizes.i_1_1);
  count += sizes.i_1_1;

  ocudu_assert(sizes.i_1_2 == 0, "PMI field i_1_2 size must be 0 bits for 4 ports.");

  if (ri > 1) {
    result.i_1_3.emplace(packed.extract(count, sizes.i_1_3));
    count += sizes.i_1_3;
  }

  result.i_2 = packed.extract(count, sizes.i_2);
  count += sizes.i_2;

  ocudu_assert(packed.size() == count,
               "Packet input size (i.e., {}) does not match with the fields size (i.e., {})",
               packed.size(),
               count);

  return csi_report_pmi{result};
}

/// Unpacks PMI.
csi_report_pmi
ocudu::csi_report_unpack_pmi(const csi_report_packed& packed, pmi_codebook_config codebook, csi_report_data::ri_type ri)
{
  return std::visit([&packed, ri](const auto& item) { return unpack_pmi(item, packed, ri); }, codebook);
}

csi_report_data::ri_type ocudu::csi_report_unpack_ri(const csi_report_packed&   ri_packed,
                                                     const ri_restriction_type& ri_restriction)
{
  unsigned ri = 1;
  if (!ri_packed.empty()) {
    ri = ri_packed.extract(0, ri_packed.size());

    ocudu_assert(ri < ri_restriction.count(),
                 "Packed RI, i.e., {}, is out of bounds given the number of allowed rank values, i.e., {}.",
                 ri,
                 ri_restriction.count());

    ri = ri_restriction.get_bit_positions()[ri] + 1;
  }
  return ri;
}
