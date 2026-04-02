// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "ocudu/ran/precoding/precoding_codebooks.h"
#include "ocudu/adt/interval.h"

using namespace ocudu;

precoding_weight_matrix ocudu::make_single_port()
{
  return make_one_layer_one_port(1, 0);
}

precoding_weight_matrix ocudu::make_one_layer_one_port(unsigned nof_ports, unsigned selected_i_port)
{
  interval<unsigned, false> selected_i_port_range(0, nof_ports);
  ocudu_assert(selected_i_port_range.contains(selected_i_port),
               "The given port identifier (i.e., {}) is out of the valid range {}",
               selected_i_port,
               selected_i_port_range);

  precoding_weight_matrix result(1, nof_ports);

  // Set weights per port.
  for (unsigned i_port = 0; i_port != nof_ports; ++i_port) {
    cf_t port_weight = (i_port == selected_i_port) ? 1.0F : 0.0F;
    result.set_coefficient(port_weight, 0, i_port);
  }

  return result;
}

precoding_weight_matrix ocudu::make_one_layer_all_ports(unsigned nof_ports)
{
  interval<unsigned, true> nof_ports_range(1, precoding_constants::MAX_NOF_PORTS);
  ocudu_assert(nof_ports_range.contains(nof_ports),
               "The number of ports (i.e., {}) is out of the valid range {}.",
               nof_ports,
               nof_ports_range);

  precoding_weight_matrix result(1, nof_ports);

  // Set normalized weights per port.
  cf_t weight = {1.0F / std::sqrt(static_cast<float>(nof_ports)), 0.0F};
  for (unsigned i_port = 0; i_port != nof_ports; ++i_port) {
    result.set_coefficient(weight, 0, i_port);
  }

  return result;
}

precoding_weight_matrix ocudu::make_identity(unsigned nof_streams)
{
  static constexpr interval<unsigned, true> nof_streams_range(1, precoding_constants::MAX_NOF_LAYERS);

  ocudu_assert(nof_streams_range.contains(nof_streams),
               "The number of streams (i.e., {}) is out of the valid range {}.",
               nof_streams,
               nof_streams_range);

  precoding_weight_matrix result(nof_streams, nof_streams);

  cf_t normalised_weight = 1.0F / std::sqrt(static_cast<float>(nof_streams));

  // Set weights per port.
  for (unsigned i_layer = 0; i_layer != nof_streams; ++i_layer) {
    for (unsigned i_port = 0; i_port != nof_streams; ++i_port) {
      cf_t weight = (i_layer == i_port) ? normalised_weight : 0.0F;
      result.set_coefficient(weight, i_layer, i_port);
    }
  }
  return result;
}

precoding_weight_matrix ocudu::make_one_layer_two_ports(unsigned i_codebook)
{
  static constexpr interval<unsigned, true>           i_codebook_range(0, 3);
  static constexpr cf_t                               sqrt_1_2         = {M_SQRT1_2, 0};
  static constexpr cf_t                               j_sqrt_1_2       = {0, M_SQRT1_2};
  static constexpr cf_t                               minus_sqrt_1_2   = {-M_SQRT1_2, 0};
  static constexpr cf_t                               minus_j_sqrt_1_2 = {0, -M_SQRT1_2};
  static constexpr std::array<std::array<cf_t, 2>, 4> codebooks        = {
      {{sqrt_1_2, sqrt_1_2}, {sqrt_1_2, j_sqrt_1_2}, {sqrt_1_2, minus_sqrt_1_2}, {sqrt_1_2, minus_j_sqrt_1_2}}};

  ocudu_assert(i_codebook_range.contains(i_codebook),
               "The given codebook identifier (i.e., {}) is out of the range {}",
               i_codebook,
               i_codebook_range);

  precoding_weight_matrix result(1, 2);

  // Select codebook.
  span<const cf_t> codebook = codebooks[i_codebook];

  // Set weights per port.
  for (unsigned i_port = 0; i_port != 2; ++i_port) {
    result.set_coefficient(codebook[i_port], 0, i_port);
  }

  return result;
}

precoding_weight_matrix ocudu::make_two_layer_two_ports(unsigned i_codebook)
{
  static constexpr interval<unsigned, true>           i_codebook_range(0, 1);
  static constexpr cf_t                               dot_five         = {0.5F, 0.0F};
  static constexpr cf_t                               minus_dot_five   = {-0.5F, 0.0F};
  static constexpr cf_t                               j_dot_five       = {0.0F, 0.5F};
  static constexpr cf_t                               minus_j_dot_five = {0.0F, -0.5F};
  static constexpr std::array<std::array<cf_t, 2>, 2> codebook0 = {{{dot_five, dot_five}, {dot_five, minus_dot_five}}};
  static constexpr std::array<std::array<cf_t, 2>, 2> codebook1 = {
      {{dot_five, dot_five}, {j_dot_five, minus_j_dot_five}}};

  ocudu_assert(i_codebook_range.contains(i_codebook),
               "The given codebook identifier (i.e., {}) is out of the range {}",
               i_codebook,
               i_codebook_range);

  precoding_weight_matrix result(2, 2);

  // Select codebook.
  const std::array<std::array<cf_t, 2>, 2>& codebook = (i_codebook == 0) ? codebook0 : codebook1;

  // Set weights per port.
  for (unsigned i_layer = 0; i_layer != 2; ++i_layer) {
    span<const cf_t> codebook_layer = codebook[i_layer];
    for (unsigned i_port = 0; i_port != 2; ++i_port) {
      result.set_coefficient(codebook_layer[i_port], i_layer, i_port);
    }
  }

  return result;
}

/// \brief Generates precoding weights for a desired layer of a Type I Single-Panel codebook, as defined in TS 38.214
/// Section 5.2.2.2.1.
///
/// The generated column follows the structure:
///
/// \f[
///   W_\text{layer} = \begin{bmatrix} \nu_{l,m} \\ \varphi_n \cdot \nu_{l,m} \end{bmatrix}
/// \f]
///
/// where \f$\nu_{l,m}\f$ is the beam steering vector and \f$\varphi_n = e^{j \cdot \text{pol\_offset\_rad}}\f$
/// is the co-phasing factor between the two cross-polarization antenna groups.
///
/// The beam steering vector \f$\nu_{l,m}\f$ is built as a Kronecker product of horizontal and vertical phase offset
/// vectors. For antenna element at horizontal position \f$i_h\f$ and vertical position \f$i_v\f$, the beam coefficient
/// is:
///
/// \f[
///   \nu_{l,m} [i_h, i_v] = e^{j 2\pi l \cdot i_h / (O_1 N_1)} \cdot e^{j 2\pi m \cdot i_v / (O_2 N_2)}
/// \f]
///
/// The parameters \c l_norm and \c m_norm represent the beam indices \f$l\f$ and \f$m\f$ normalized by the total number
/// of oversampled beams in each plane, i.e.,
///
/// \f$l_\text{norm} = l / (O_1 N_1)\f$
/// \f$m_\text{norm} = m / (O_2 N_2)\f$
///
/// \note The normalization factor \f$1/\sqrt{v \cdot P_\text{CSI-RS}}\f$ (where \f$v\f$ is the number of layers
/// and \f$P_\text{CSI-RS} = 2 N_1 N_2\f$ is the number of CSI-RS ports) is not applied by this function. The caller is
/// responsible for applying it after assembling all layers.
///
/// \param[out] result Precoding weight matrix where the generated weights will be stored.
/// \param[in] i_layer Layer index within the precoding weight matrix.
/// \param[in] N_1 Number of antenna elements in the horizontal plane.
/// \param[in] N_2 Number of antenna elements in the vertical plane.
/// \param[in] l_norm Normalized horizontal beam index, i.e., \f$l / (O_1 N_1)\f$.
/// \param[in] m_norm Normalized vertical beam index, i.e., \f$m / (O_2 N_2)\f$.
/// \param[in] pol_offset_rad  Cross-polarization phase offset,
static void make_layer_type1_sp_mode1(precoding_weight_matrix& result,
                                      unsigned                 i_layer,
                                      unsigned                 N_1,
                                      unsigned                 N_2,
                                      float                    l_norm,
                                      float                    m_norm,
                                      float                    pol_offset_rad)
{
  // Phase increment for each beam coefficient. This defines the direction of the beam in the horizontal plane.
  float phase_inc_h_rad = l_norm * 2.0F * M_PI;
  // Phase increment for each beam coefficient. This defines the direction of the beam in the vertical plane.
  float phase_inc_v_rad = m_norm * 2.0F * M_PI;
  // Get the number of physical ports, which is half the number of CSI-RS ports given that two polarizations are used.
  unsigned nof_ports_2 = result.get_nof_ports() / 2;

  // Phasor to increment the phase of each beam coefficient in the horizontal plane.
  cf_t phase_inc_h = std::polar(1.0F, phase_inc_h_rad);
  // Phasor to increment the phase of each beam coefficient in the vertical plane.
  cf_t phase_inc_v = std::polar(1.0F, phase_inc_v_rad);
  // Phasor to apply the phase offset between the first and second polarizations.
  cf_t polarization_offset = std::polar(1.0F, pol_offset_rad);

  // Current beam coefficient in the horizontal plane. Initially set to one.
  cf_t beam_coef_h = std::polar(1.0F, 0.0F);
  // Current beam coefficient in the vertical plane. Initially set to one.
  cf_t beam_coef_v = std::polar(1.0F, 0.0F);

  // Current processed port index.
  unsigned i_port = 0;

  // Iterate every horizontal physical port.
  for (unsigned i_h = 0; i_h != N_1; ++i_h) {
    // Reset vertical beam coefficient.
    beam_coef_v = std::polar(1.0F, 0.0F);

    // For each horizontal port, iterate every vertical physical port.
    for (unsigned i_v = 0; i_v != N_2; ++i_v) {
      // Compute the beam coefficient for this port and layer.
      cf_t beam_coef = beam_coef_h * beam_coef_v;
      // First polarization.
      result.set_coefficient(beam_coef, i_layer, i_port);
      // Second polarization, same beam but with phase offset.
      result.set_coefficient(polarization_offset * beam_coef, i_layer, nof_ports_2 + i_port);

      // Increase the port index.
      i_port++;
      // Apply the phase increment to the vertical beam coefficient.
      beam_coef_v *= phase_inc_v;
    }

    // Apply the phase increment to the horizontal beam coefficient.
    beam_coef_h *= phase_inc_h;
  }
}

precoding_weight_matrix ocudu::make_one_layer_four_ports_type1_sp_mode1(unsigned beam_azimuth_id, unsigned pol_shift_id)
{
  // Beam oversampling factor in the horizontal plane, from TS38.214 Section 5.2.2.2.1.
  static constexpr unsigned O_1 = 4;
  // Number of cross-polarized antenna elements, from TS38.214 Section 5.2.2.2.1.
  static constexpr unsigned N_1 = 2;
  // Number of possible horizontal beams to choose from.
  static constexpr unsigned nof_beams = O_1 * N_1;
  // Number of possible polarization phase shifts to choose from.
  static constexpr unsigned nof_pol_shifts = 4;

  static constexpr interval<unsigned, false> beam_azimuth_range(0, nof_beams);
  static constexpr interval<unsigned, false> pol_phase_shift_range(0, nof_pol_shifts);

  ocudu_assert(beam_azimuth_range.contains(beam_azimuth_id),
               "The given beam azimuth identifier i1_1 (i.e., {}) is out of the range {}",
               beam_azimuth_id,
               beam_azimuth_range);

  ocudu_assert(pol_phase_shift_range.contains(pol_shift_id),
               "The given polarization phase shift i2 (i.e., {}) is out of the range {}",
               pol_shift_id,
               pol_phase_shift_range);

  // Precoding weight matrix for one layer mapped into four antenna ports.
  precoding_weight_matrix result(1, 4);

  // Normalized horizontal beam index.
  float l_norm = static_cast<float>(beam_azimuth_id) / static_cast<float>(nof_beams);
  // Normalized vertical beam index.
  float m_norm = 0.0f;
  // Polarization phase shift. This defines the relative phase between the cross-polarized antenna elements.
  float pol_phase_shift_rad = M_PI_2 * static_cast<float>(pol_shift_id);

  // Build layer 0 beam coefficients.
  make_layer_type1_sp_mode1(result, 0, N_1, 1, l_norm, m_norm, pol_phase_shift_rad);

  // Apply scaling.
  float scaling = 1.0F / std::sqrt(result.get_nof_layers() * result.get_nof_ports());
  result *= scaling;

  return result;
}

precoding_weight_matrix ocudu::make_two_layer_four_ports_type1_sp_mode1(unsigned beam_azimuth_id,
                                                                        unsigned beam_offset_id,
                                                                        unsigned pol_shift_id)
{
  // Beam oversampling factor in the horizontal plane, from TS38.214 Section 5.2.2.2.1.
  static constexpr unsigned O_1 = 4;
  // Number of cross-polarized antenna elements, from TS38.214 Section 5.2.2.2.1.
  static constexpr unsigned N_1 = 2;
  // Number of possible horizontal beams to choose from.
  static constexpr unsigned nof_beams = O_1 * N_1;
  // Number of possible polarization phase shifts to choose from.
  static constexpr unsigned nof_pol_shifts = 2;

  static constexpr interval<unsigned, false> beam_azimuth_range(0, nof_beams);
  static constexpr interval<unsigned, false> beam_offset_range(0, 2);
  static constexpr interval<unsigned, false> pol_phase_shift_range(0, nof_pol_shifts);

  ocudu_assert(beam_azimuth_range.contains(beam_azimuth_id),
               "The given beam azimuth identifier i1_1 (i.e., {}) is out of the range {}",
               beam_azimuth_id,
               beam_azimuth_range);

  ocudu_assert(beam_offset_range.contains(beam_offset_id),
               "The given beam offset identifier i1_3 (i.e., {}) is out of the range {}",
               beam_offset_id,
               beam_offset_range);

  ocudu_assert(pol_phase_shift_range.contains(pol_shift_id),
               "The given polarization phase shift i2 (i.e., {}) is out of the range {}",
               pol_shift_id,
               pol_phase_shift_range);

  // Precoding weight matrix for two layers mapped into four antenna ports.
  precoding_weight_matrix result(2, 4);

  // Select k1 as per TS38.214 Section 5.2.2.2.1-4.
  static constexpr unsigned k1 = O_1;

  // Normalized horizontal beam index.
  float l_norm = static_cast<float>(beam_azimuth_id) / static_cast<float>(nof_beams);
  // Normalized vertical beam index.
  float m_norm = 0.0f;
  // Polarization phase shift. This defines the relative phase between the cross-polarized antenna elements.
  float pol_phase_shift_rad = M_PI_2 * static_cast<float>(pol_shift_id);

  // Build layer 0 beam coefficients.
  make_layer_type1_sp_mode1(result, 0, N_1, 1, l_norm, m_norm, pol_phase_shift_rad);

  // Build layer 1 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id + k1) / static_cast<float>(nof_beams);
  make_layer_type1_sp_mode1(result, 1, N_1, 1, l_norm, m_norm, pol_phase_shift_rad + M_PI);

  // Apply scaling.
  float scaling = 1.0F / std::sqrt(result.get_nof_layers() * result.get_nof_ports());
  result *= scaling;

  return result;
}

precoding_weight_matrix ocudu::make_three_layer_four_ports_type1_sp(unsigned beam_azimuth_id, unsigned pol_shift_id)
{
  // Beam oversampling factor in the horizontal plane, from TS38.214 Section 5.2.2.2.1.
  static constexpr unsigned O_1 = 4;
  // Number of cross-polarized antenna elements, from TS38.214 Section 5.2.2.2.1.
  static constexpr unsigned N_1 = 2;
  // Number of possible horizontal beams to choose from.
  static constexpr unsigned nof_beams = O_1 * N_1;
  // Number of possible polarization phase shifts to choose from.
  static constexpr unsigned nof_pol_shifts = 2;

  static constexpr interval<unsigned, false> beam_azimuth_range(0, nof_beams);
  static constexpr interval<unsigned, false> pol_phase_shift_range(0, nof_pol_shifts);

  ocudu_assert(beam_azimuth_range.contains(beam_azimuth_id),
               "The given beam azimuth identifier i1_1 (i.e., {}) is out of the range {}",
               beam_azimuth_id,
               beam_azimuth_range);

  ocudu_assert(pol_phase_shift_range.contains(pol_shift_id),
               "The given polarization phase shift i2 (i.e., {}) is out of the range {}",
               pol_shift_id,
               pol_phase_shift_range);

  // Precoding weight matrix for three layers mapped into four antenna ports.
  precoding_weight_matrix result(3, 4);

  // Select k1 as per TS38.214 Section 5.2.2.2.1-4.
  static constexpr unsigned k1 = O_1;

  // Normalized horizontal beam index.
  float l_norm = static_cast<float>(beam_azimuth_id) / static_cast<float>(nof_beams);
  // Normalized vertical beam index.
  float m_norm = 0.0f;
  // Polarization phase shift. This defines the relative phase between the cross-polarized antenna elements.
  float pol_phase_shift_rad = M_PI_2 * static_cast<float>(pol_shift_id);

  // Build layer 0 beam coefficients.
  make_layer_type1_sp_mode1(result, 0, N_1, 1, l_norm, m_norm, pol_phase_shift_rad);

  // Build layer 2 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 0.
  make_layer_type1_sp_mode1(result, 2, N_1, 1, l_norm, m_norm, pol_phase_shift_rad + M_PI);

  // Build layer 1 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id + k1) / static_cast<float>(nof_beams);
  make_layer_type1_sp_mode1(result, 1, N_1, 1, l_norm, m_norm, pol_phase_shift_rad);

  // Apply scaling.
  float scaling = 1.0F / std::sqrt(result.get_nof_layers() * result.get_nof_ports());
  result *= scaling;

  return result;
}

precoding_weight_matrix ocudu::make_four_layer_four_ports_type1_sp(unsigned beam_azimuth_id, unsigned pol_shift_id)
{
  // Beam oversampling factor in the horizontal plane, from TS38.214 Section 5.2.2.2.1.
  static constexpr unsigned O_1 = 4;
  // Number of cross-polarized antenna elements, from TS38.214 Section 5.2.2.2.1.
  static constexpr unsigned N_1 = 2;
  // Number of possible horizontal beams to choose from.
  static constexpr unsigned nof_beams = O_1 * N_1;
  // Number of possible polarization phase shifts to choose from.
  static constexpr unsigned nof_pol_shifts = 2;

  static constexpr interval<unsigned, false> beam_azimuth_range(0, nof_beams);
  static constexpr interval<unsigned, false> pol_phase_shift_range(0, nof_pol_shifts);

  ocudu_assert(beam_azimuth_range.contains(beam_azimuth_id),
               "The given beam azimuth identifier i1_1 (i.e., {}) is out of the range {}",
               beam_azimuth_id,
               beam_azimuth_range);

  ocudu_assert(pol_phase_shift_range.contains(pol_shift_id),
               "The given polarization phase shift i2 (i.e., {}) is out of the range {}",
               pol_shift_id,
               pol_phase_shift_range);

  // Precoding weight matrix for three layers mapped into four antenna ports.
  precoding_weight_matrix result(4, 4);

  // Select k1 as per TS38.214 Section 5.2.2.2.1-4.
  static constexpr unsigned k1 = O_1;

  // Normalized horizontal beam index.
  float l_norm = static_cast<float>(beam_azimuth_id) / static_cast<float>(nof_beams);
  // Normalized vertical beam index.
  float m_norm = 0.0f;
  // Polarization phase shift. This defines the relative phase between the cross-polarized antenna elements.
  float pol_phase_shift_rad = M_PI_2 * static_cast<float>(pol_shift_id);

  // Build layer 0 beam coefficients.
  make_layer_type1_sp_mode1(result, 0, N_1, 1, l_norm, m_norm, pol_phase_shift_rad);

  // Build layer 2 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 0.
  make_layer_type1_sp_mode1(result, 2, N_1, 1, l_norm, m_norm, pol_phase_shift_rad + M_PI);

  // Build layer 1 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id + k1) / static_cast<float>(nof_beams);
  make_layer_type1_sp_mode1(result, 1, N_1, 1, l_norm, m_norm, pol_phase_shift_rad);

  // Build layer 3 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 1.
  make_layer_type1_sp_mode1(result, 3, N_1, 1, l_norm, m_norm, pol_phase_shift_rad + M_PI);

  // Apply scaling.
  float scaling = 1.0F / std::sqrt(result.get_nof_layers() * result.get_nof_ports());
  result *= scaling;

  return result;
}

precoding_weight_matrix ocudu::make_one_layer_eight_ports_type1_sp_mode1(unsigned beam_azimuth_id,
                                                                         unsigned pol_shift_id)
{
  // Beam oversampling factor in the horizontal plane, from TS38.214 Section 5.2.2.2.1.
  static constexpr unsigned O_1 = 4;
  // Number of cross-polarized antenna elements, from TS38.214 Section 5.2.2.2.1.
  static constexpr unsigned N_1 = 4;
  // Number of possible horizontal beams to choose from.
  static constexpr unsigned nof_beams = O_1 * N_1;
  // Number of possible polarization phase shifts to choose from.
  static constexpr unsigned nof_pol_shifts = 4;

  static constexpr interval<unsigned, false> beam_azimuth_range(0, nof_beams);
  static constexpr interval<unsigned, false> pol_phase_shift_range(0, nof_pol_shifts);

  ocudu_assert(beam_azimuth_range.contains(beam_azimuth_id),
               "The given beam azimuth identifier i1_1 (i.e., {}) is out of the range {}",
               beam_azimuth_id,
               beam_azimuth_range);

  ocudu_assert(pol_phase_shift_range.contains(pol_shift_id),
               "The given polarization phase shift i2 (i.e., {}) is out of the range {}",
               pol_shift_id,
               pol_phase_shift_range);

  // Precoding weight matrix for one layer mapped into eight antenna ports.
  precoding_weight_matrix result(1, 8);

  // Normalized horizontal beam index.
  float l_norm = static_cast<float>(beam_azimuth_id) / static_cast<float>(nof_beams);
  // Normalized vertical beam index.
  float m_norm = 0.0f;
  // Polarization phase shift. This defines the relative phase between the cross-polarized antenna elements.
  float pol_phase_shift_rad = M_PI_2 * static_cast<float>(pol_shift_id);

  // Build layer 0 beam coefficients.
  make_layer_type1_sp_mode1(result, 0, N_1, 1, l_norm, m_norm, pol_phase_shift_rad);

  // Apply scaling.
  float scaling = 1.0F / std::sqrt(result.get_nof_layers() * result.get_nof_ports());
  result *= scaling;

  return result;
}

precoding_weight_matrix ocudu::make_one_layer_eight_ports_type1_sp_mode1(unsigned beam_azimuth_id,
                                                                         unsigned beam_elevation_id,
                                                                         unsigned pol_shift_id)
{
  // Beam oversampling factor in the horizontal and vertical plane, from TS38.214 Section 5.2.2.2.1. The index is
  // dropped given that it is the same in both planes, i.e., O1 = O2 = 4.
  static constexpr unsigned O = 4;
  // Number of cross-polarized antenna elements, from TS38.214 Section 5.2.2.2.1. The index is dropped given that it is
  // them same in both planes, i.e., N1 = N2 = 2.
  static constexpr unsigned N = 2;
  // Number of possible beams per plane to choose from.
  static constexpr unsigned nof_beams_per_plane = O * N;
  // Number of possible polarization phase shifts to choose from.
  static constexpr unsigned nof_pol_shifts = 4;

  static constexpr interval<unsigned, false> beam_azimuth_range(0, nof_beams_per_plane);
  static constexpr interval<unsigned, false> beam_elevation_range(0, nof_beams_per_plane);
  static constexpr interval<unsigned, false> pol_phase_shift_range(0, nof_pol_shifts);

  ocudu_assert(beam_azimuth_range.contains(beam_azimuth_id),
               "The given beam azimuth identifier i1_1 (i.e., {}) is out of the range {}",
               beam_azimuth_id,
               beam_azimuth_range);

  ocudu_assert(beam_elevation_range.contains(beam_elevation_id),
               "The given beam elevation identifier i1_2 (i.e., {}) is out of the range {}",
               beam_elevation_id,
               beam_elevation_range);

  ocudu_assert(pol_phase_shift_range.contains(pol_shift_id),
               "The given polarization phase shift i2 (i.e., {}) is out of the range {}",
               pol_shift_id,
               pol_phase_shift_range);

  // Precoding weight matrix for one layer mapped into eight antenna ports.
  precoding_weight_matrix result(1, 8);

  // Normalized horizontal beam index.
  float l_norm = static_cast<float>(beam_azimuth_id) / static_cast<float>(nof_beams_per_plane);
  // Normalized vertical beam index.
  float m_norm = static_cast<float>(beam_elevation_id) / static_cast<float>(nof_beams_per_plane);
  ;
  // Polarization phase shift. This defines the relative phase between the cross-polarized antenna elements.
  float pol_phase_shift_rad = M_PI_2 * static_cast<float>(pol_shift_id);

  // Build layer 0 beam coefficients.
  make_layer_type1_sp_mode1(result, 0, N, N, l_norm, m_norm, pol_phase_shift_rad);

  // Apply scaling.
  float scaling = 1.0F / std::sqrt(result.get_nof_layers() * result.get_nof_ports());
  result *= scaling;

  return result;
}

precoding_weight_matrix ocudu::make_two_layer_eight_ports_type1_sp_mode1(unsigned beam_azimuth_id,
                                                                         unsigned beam_offset_id,
                                                                         unsigned pol_shift_id)
{
  // Beam oversampling factor in the horizontal plane, from TS38.214 Section 5.2.2.2.1.
  static constexpr unsigned O_1 = 4;
  // Number of cross-polarized antenna elements, from TS38.214 Section 5.2.2.2.1.
  static constexpr unsigned N_1 = 4;
  // Number of possible horizontal beams to choose from.
  static constexpr unsigned nof_beams = O_1 * N_1;
  // Number of possible polarization phase shifts to choose from.
  static constexpr unsigned nof_pol_shifts = 2;

  static constexpr interval<unsigned, false> beam_azimuth_range(0, nof_beams);
  static constexpr interval<unsigned, false> beam_offset_range(0, 4);
  static constexpr interval<unsigned, false> pol_phase_shift_range(0, nof_pol_shifts);

  ocudu_assert(beam_azimuth_range.contains(beam_azimuth_id),
               "The given beam azimuth identifier i1_1 (i.e., {}) is out of the range {}",
               beam_azimuth_id,
               beam_azimuth_range);

  ocudu_assert(beam_offset_range.contains(beam_offset_id),
               "The given beam offset identifier i1_3 (i.e., {}) is out of the range {}",
               beam_offset_id,
               beam_offset_range);

  ocudu_assert(pol_phase_shift_range.contains(pol_shift_id),
               "The given polarization phase shift i2 (i.e., {}) is out of the range {}",
               pol_shift_id,
               pol_phase_shift_range);

  // Precoding weight matrix for two layers mapped into four antenna ports.
  precoding_weight_matrix result(2, 8);

  // Select k1 as per TS38.214 Section 5.2.2.2.1-3.
  unsigned k1 = beam_offset_id * O_1;

  // Normalized horizontal beam index.
  float l_norm = static_cast<float>(beam_azimuth_id) / static_cast<float>(nof_beams);
  // Normalized vertical beam index.
  float m_norm = 0.0f;
  // Polarization phase shift. This defines the relative phase between the cross-polarized antenna elements.
  float pol_phase_shift_rad = M_PI_2 * static_cast<float>(pol_shift_id);

  // Build layer 0 beam coefficients.
  make_layer_type1_sp_mode1(result, 0, N_1, 1, l_norm, m_norm, pol_phase_shift_rad);

  // Build layer 1 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id + k1) / static_cast<float>(nof_beams);
  make_layer_type1_sp_mode1(result, 1, N_1, 1, l_norm, m_norm, pol_phase_shift_rad + M_PI);

  // Apply scaling.
  float scaling = 1.0F / std::sqrt(result.get_nof_layers() * result.get_nof_ports());
  result *= scaling;

  return result;
}

precoding_weight_matrix ocudu::make_two_layer_eight_ports_type1_sp_mode1(unsigned beam_azimuth_id,
                                                                         unsigned beam_elevation_id,
                                                                         unsigned beam_offset_id,
                                                                         unsigned pol_shift_id)
{
  // Beam oversampling factor in the horizontal and vertical plane, from TS38.214 Section 5.2.2.2.1. The index is
  // dropped given that it is the same in both planes, i.e., O1 = O2 = 4.
  static constexpr unsigned O = 4;
  // Number of cross-polarized antenna elements, from TS38.214 Section 5.2.2.2.1. The index is dropped given that it is
  // them same in both planes, i.e., N1 = N2 = 2.
  static constexpr unsigned N = 2;
  // Number of possible beams per plane to choose from.
  static constexpr unsigned nof_beams_per_plane = O * N;
  // Number of possible polarization phase shifts to choose from.
  static constexpr unsigned nof_pol_shifts = 2;

  static constexpr interval<unsigned, false> beam_azimuth_range(0, nof_beams_per_plane);
  static constexpr interval<unsigned, false> beam_elevation_range(0, nof_beams_per_plane);
  static constexpr interval<unsigned, false> beam_offset_range(0, 4);
  static constexpr interval<unsigned, false> pol_phase_shift_range(0, nof_pol_shifts);

  ocudu_assert(beam_azimuth_range.contains(beam_azimuth_id),
               "The given beam azimuth identifier i1_1 (i.e., {}) is out of the range {}",
               beam_azimuth_id,
               beam_azimuth_range);

  ocudu_assert(beam_elevation_range.contains(beam_elevation_id),
               "The given beam elevation identifier i1_2 (i.e., {}) is out of the range {}",
               beam_elevation_id,
               beam_elevation_range);

  ocudu_assert(beam_offset_range.contains(beam_offset_id),
               "The given beam offset identifier i1_3 (i.e., {}) is out of the range {}",
               beam_offset_id,
               beam_offset_range);

  ocudu_assert(pol_phase_shift_range.contains(pol_shift_id),
               "The given polarization phase shift i2 (i.e., {}) is out of the range {}",
               pol_shift_id,
               pol_phase_shift_range);

  // Precoding weight matrix for one layer mapped into eight antenna ports.
  precoding_weight_matrix result(2, 8);

  // Select k1 and k2 as per TS38.214 Section 5.2.2.2.1-3.
  unsigned k1 = (beam_offset_id % 2 == 0) ? 0 : O;
  unsigned k2 = (beam_offset_id < 2) ? 0 : O;

  // Normalized horizontal beam index.
  float l_norm = static_cast<float>(beam_azimuth_id) / static_cast<float>(nof_beams_per_plane);
  // Normalized vertical beam index.
  float m_norm = static_cast<float>(beam_elevation_id) / static_cast<float>(nof_beams_per_plane);
  // Polarization phase shift. This defines the relative phase between the cross-polarized antenna elements.
  float pol_phase_shift_rad = M_PI_2 * static_cast<float>(pol_shift_id);

  // Build layer 0 beam coefficients.
  make_layer_type1_sp_mode1(result, 0, N, N, l_norm, m_norm, pol_phase_shift_rad);

  // Build layer 1 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id + k1) / static_cast<float>(nof_beams_per_plane);
  m_norm = static_cast<float>(beam_elevation_id + k2) / static_cast<float>(nof_beams_per_plane);
  make_layer_type1_sp_mode1(result, 1, N, N, l_norm, m_norm, pol_phase_shift_rad + M_PI);

  // Apply scaling.
  float scaling = 1.0F / std::sqrt(result.get_nof_layers() * result.get_nof_ports());
  result *= scaling;

  return result;
}

precoding_weight_matrix
ocudu::make_three_layer_eight_ports_type1_sp(unsigned beam_azimuth_id, unsigned beam_offset_id, unsigned pol_shift_id)
{
  // Beam oversampling factor in the horizontal plane, from TS38.214 Section 5.2.2.2.1.
  static constexpr unsigned O_1 = 4;
  // Number of cross-polarized antenna elements, from TS38.214 Section 5.2.2.2.1.
  static constexpr unsigned N_1 = 4;
  // Number of possible horizontal beams to choose from.
  static constexpr unsigned nof_beams = O_1 * N_1;
  // Number of possible polarization phase shifts to choose from.
  static constexpr unsigned nof_pol_shifts = 2;

  static constexpr interval<unsigned, false> beam_azimuth_range(0, nof_beams);
  static constexpr interval<unsigned, false> beam_offset_range(0, 3);
  static constexpr interval<unsigned, false> pol_phase_shift_range(0, nof_pol_shifts);

  ocudu_assert(beam_azimuth_range.contains(beam_azimuth_id),
               "The given beam azimuth identifier i1_1 (i.e., {}) is out of the range {}",
               beam_azimuth_id,
               beam_azimuth_range);

  ocudu_assert(beam_offset_range.contains(beam_offset_id),
               "The given beam offset identifier i1_3 (i.e., {}) is out of the range {}",
               beam_offset_id,
               beam_offset_range);

  ocudu_assert(pol_phase_shift_range.contains(pol_shift_id),
               "The given polarization phase shift i2 (i.e., {}) is out of the range {}",
               pol_shift_id,
               pol_phase_shift_range);

  // Precoding weight matrix for three layers mapped into eight antenna ports.
  precoding_weight_matrix result(3, 8);

  // Select k1 as per TS38.214 Section 5.2.2.2.1-4.
  unsigned k1 = (beam_offset_id + 1) * O_1;

  // Normalized horizontal beam index.
  float l_norm = static_cast<float>(beam_azimuth_id) / static_cast<float>(nof_beams);
  // Normalized vertical beam index.
  float m_norm = 0.0f;
  // Polarization phase shift. This defines the relative phase between the cross-polarized antenna elements.
  float pol_phase_shift_rad = M_PI_2 * static_cast<float>(pol_shift_id);

  // Build layer 0 beam coefficients.
  make_layer_type1_sp_mode1(result, 0, N_1, 1, l_norm, m_norm, pol_phase_shift_rad);

  // Build layer 2 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 0.
  make_layer_type1_sp_mode1(result, 2, N_1, 1, l_norm, m_norm, pol_phase_shift_rad + M_PI);

  // Build layer 1 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id + k1) / static_cast<float>(nof_beams);
  make_layer_type1_sp_mode1(result, 1, N_1, 1, l_norm, m_norm, pol_phase_shift_rad);

  // Apply scaling.
  float scaling = 1.0F / std::sqrt(result.get_nof_layers() * result.get_nof_ports());
  result *= scaling;

  return result;
}

precoding_weight_matrix ocudu::make_three_layer_eight_ports_type1_sp(unsigned beam_azimuth_id,
                                                                     unsigned beam_elevation_id,
                                                                     unsigned beam_offset_id,
                                                                     unsigned pol_shift_id)
{
  // Beam oversampling factor in the horizontal and vertical plane, from TS38.214 Section 5.2.2.2.1. The index is
  // dropped given that it is the same in both planes, i.e., O1 = O2 = 4.
  static constexpr unsigned O = 4;
  // Number of cross-polarized antenna elements, from TS38.214 Section 5.2.2.2.1. The index is dropped given that it is
  // them same in both planes, i.e., N1 = N2 = 2.
  static constexpr unsigned N = 2;
  // Number of possible beams per plane to choose from.
  static constexpr unsigned nof_beams_per_plane = O * N;
  // Number of possible polarization phase shifts to choose from.
  static constexpr unsigned nof_pol_shifts = 2;

  static constexpr interval<unsigned, false> beam_azimuth_range(0, nof_beams_per_plane);
  static constexpr interval<unsigned, false> beam_elevation_range(0, nof_beams_per_plane);
  static constexpr interval<unsigned, false> beam_offset_range(0, 3);
  static constexpr interval<unsigned, false> pol_phase_shift_range(0, nof_pol_shifts);

  ocudu_assert(beam_azimuth_range.contains(beam_azimuth_id),
               "The given beam azimuth identifier i1_1 (i.e., {}) is out of the range {}",
               beam_azimuth_id,
               beam_azimuth_range);

  ocudu_assert(beam_elevation_range.contains(beam_elevation_id),
               "The given beam elevation identifier i1_2 (i.e., {}) is out of the range {}",
               beam_elevation_id,
               beam_elevation_range);

  ocudu_assert(beam_offset_range.contains(beam_offset_id),
               "The given beam offset identifier i1_3 (i.e., {}) is out of the range {}",
               beam_offset_id,
               beam_offset_range);

  ocudu_assert(pol_phase_shift_range.contains(pol_shift_id),
               "The given polarization phase shift i2 (i.e., {}) is out of the range {}",
               pol_shift_id,
               pol_phase_shift_range);

  // Precoding weight matrix for three layers mapped into eight antenna ports.
  precoding_weight_matrix result(3, 8);

  // Select k1 and k2 as per TS38.214 Section 5.2.2.2.1-4.
  unsigned k1 = (beam_offset_id == 1) ? 0 : O;
  unsigned k2 = (beam_offset_id == 0) ? 0 : O;

  // Normalized horizontal beam index.
  float l_norm = static_cast<float>(beam_azimuth_id) / static_cast<float>(nof_beams_per_plane);
  // Normalized vertical beam index.
  float m_norm = static_cast<float>(beam_elevation_id) / static_cast<float>(nof_beams_per_plane);
  ;
  // Polarization phase shift. This defines the relative phase between the cross-polarized antenna elements.
  float pol_phase_shift_rad = M_PI_2 * static_cast<float>(pol_shift_id);

  // Build layer 0 beam coefficients.
  make_layer_type1_sp_mode1(result, 0, N, N, l_norm, m_norm, pol_phase_shift_rad);

  // Build layer 2 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 0.
  make_layer_type1_sp_mode1(result, 2, N, N, l_norm, m_norm, pol_phase_shift_rad + M_PI);

  // Build layer 1 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id + k1) / static_cast<float>(nof_beams_per_plane);
  m_norm = static_cast<float>(beam_elevation_id + k2) / static_cast<float>(nof_beams_per_plane);
  make_layer_type1_sp_mode1(result, 1, N, N, l_norm, m_norm, pol_phase_shift_rad);

  // Apply scaling.
  float scaling = 1.0F / std::sqrt(result.get_nof_layers() * result.get_nof_ports());
  result *= scaling;

  return result;
}

precoding_weight_matrix
ocudu::make_four_layer_eight_ports_type1_sp(unsigned beam_azimuth_id, unsigned beam_offset_id, unsigned pol_shift_id)
{
  // Beam oversampling factor in the horizontal plane, from TS38.214 Section 5.2.2.2.1.
  static constexpr unsigned O_1 = 4;
  // Number of cross-polarized antenna elements, from TS38.214 Section 5.2.2.2.1.
  static constexpr unsigned N_1 = 4;
  // Number of possible horizontal beams to choose from.
  static constexpr unsigned nof_beams = O_1 * N_1;
  // Number of possible polarization phase shifts to choose from.
  static constexpr unsigned nof_pol_shifts = 2;

  static constexpr interval<unsigned, false> beam_azimuth_range(0, nof_beams);
  static constexpr interval<unsigned, false> beam_offset_range(0, 3);
  static constexpr interval<unsigned, false> pol_phase_shift_range(0, nof_pol_shifts);

  ocudu_assert(beam_azimuth_range.contains(beam_azimuth_id),
               "The given beam azimuth identifier i1_1 (i.e., {}) is out of the range {}",
               beam_azimuth_id,
               beam_azimuth_range);

  ocudu_assert(beam_offset_range.contains(beam_offset_id),
               "The given beam offset identifier i1_3 (i.e., {}) is out of the range {}",
               beam_offset_id,
               beam_offset_range);

  ocudu_assert(pol_phase_shift_range.contains(pol_shift_id),
               "The given polarization phase shift i2 (i.e., {}) is out of the range {}",
               pol_shift_id,
               pol_phase_shift_range);

  // Precoding weight matrix for three layers mapped into eight antenna ports.
  precoding_weight_matrix result(4, 8);

  // Select k1 as per TS38.214 Section 5.2.2.2.1-4.
  unsigned k1 = (beam_offset_id + 1) * O_1;

  // Normalized horizontal beam index.
  float l_norm = static_cast<float>(beam_azimuth_id) / static_cast<float>(nof_beams);
  // Normalized vertical beam index.
  float m_norm = 0.0f;
  // Polarization phase shift. This defines the relative phase between the cross-polarized antenna elements.
  float pol_phase_shift_rad = M_PI_2 * static_cast<float>(pol_shift_id);

  // Build layer 0 beam coefficients.
  make_layer_type1_sp_mode1(result, 0, N_1, 1, l_norm, m_norm, pol_phase_shift_rad);

  // Build layer 2 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 0.
  make_layer_type1_sp_mode1(result, 2, N_1, 1, l_norm, m_norm, pol_phase_shift_rad + M_PI);

  // Build layer 1 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id + k1) / static_cast<float>(nof_beams);
  make_layer_type1_sp_mode1(result, 1, N_1, 1, l_norm, m_norm, pol_phase_shift_rad);

  // Build layer 3 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 1.
  make_layer_type1_sp_mode1(result, 3, N_1, 1, l_norm, m_norm, pol_phase_shift_rad + M_PI);

  // Apply scaling.
  float scaling = 1.0F / std::sqrt(result.get_nof_layers() * result.get_nof_ports());
  result *= scaling;

  return result;
}

precoding_weight_matrix ocudu::make_four_layer_eight_ports_type1_sp(unsigned beam_azimuth_id,
                                                                    unsigned beam_elevation_id,
                                                                    unsigned beam_offset_id,
                                                                    unsigned pol_shift_id)
{
  // Beam oversampling factor in the horizontal and vertical plane, from TS38.214 Section 5.2.2.2.1. The index is
  // dropped given that it is the same in both planes, i.e., O1 = O2 = 4.
  static constexpr unsigned O = 4;
  // Number of cross-polarized antenna elements, from TS38.214 Section 5.2.2.2.1. The index is dropped given that it is
  // them same in both planes, i.e., N1 = N2 = 2.
  static constexpr unsigned N = 2;
  // Number of possible beams per plane to choose from.
  static constexpr unsigned nof_beams_per_plane = O * N;
  // Number of possible polarization phase shifts to choose from.
  static constexpr unsigned nof_pol_shifts = 2;

  static constexpr interval<unsigned, false> beam_azimuth_range(0, nof_beams_per_plane);
  static constexpr interval<unsigned, false> beam_elevation_range(0, nof_beams_per_plane);
  static constexpr interval<unsigned, false> beam_offset_range(0, 3);
  static constexpr interval<unsigned, false> pol_phase_shift_range(0, nof_pol_shifts);

  ocudu_assert(beam_azimuth_range.contains(beam_azimuth_id),
               "The given beam azimuth identifier i1_1 (i.e., {}) is out of the range {}",
               beam_azimuth_id,
               beam_azimuth_range);

  ocudu_assert(beam_elevation_range.contains(beam_elevation_id),
               "The given beam elevation identifier i1_2 (i.e., {}) is out of the range {}",
               beam_elevation_id,
               beam_elevation_range);

  ocudu_assert(beam_offset_range.contains(beam_offset_id),
               "The given beam offset identifier i1_3 (i.e., {}) is out of the range {}",
               beam_offset_id,
               beam_offset_range);

  ocudu_assert(pol_phase_shift_range.contains(pol_shift_id),
               "The given polarization phase shift i2 (i.e., {}) is out of the range {}",
               pol_shift_id,
               pol_phase_shift_range);

  // Precoding weight matrix for three layers mapped into eight antenna ports.
  precoding_weight_matrix result(4, 8);

  // Select k1 and k2 as per TS38.214 Section 5.2.2.2.1-4.
  unsigned k1 = (beam_offset_id == 1) ? 0 : O;
  unsigned k2 = (beam_offset_id == 0) ? 0 : O;

  // Normalized horizontal beam index.
  float l_norm = static_cast<float>(beam_azimuth_id) / static_cast<float>(nof_beams_per_plane);
  // Normalized vertical beam index.
  float m_norm = static_cast<float>(beam_elevation_id) / static_cast<float>(nof_beams_per_plane);
  ;
  // Polarization phase shift. This defines the relative phase between the cross-polarized antenna elements.
  float pol_phase_shift_rad = M_PI_2 * static_cast<float>(pol_shift_id);

  // Build layer 0 beam coefficients.
  make_layer_type1_sp_mode1(result, 0, N, N, l_norm, m_norm, pol_phase_shift_rad);

  // Build layer 2 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 0.
  make_layer_type1_sp_mode1(result, 2, N, N, l_norm, m_norm, pol_phase_shift_rad + M_PI);

  // Build layer 1 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id + k1) / static_cast<float>(nof_beams_per_plane);
  m_norm = static_cast<float>(beam_elevation_id + k2) / static_cast<float>(nof_beams_per_plane);
  make_layer_type1_sp_mode1(result, 1, N, N, l_norm, m_norm, pol_phase_shift_rad);

  // Build layer 3 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 2.
  make_layer_type1_sp_mode1(result, 3, N, N, l_norm, m_norm, pol_phase_shift_rad + M_PI);

  // Apply scaling.
  float scaling = 1.0F / std::sqrt(result.get_nof_layers() * result.get_nof_ports());
  result *= scaling;

  return result;
}

precoding_weight_matrix ocudu::make_five_layer_eight_ports_type1_sp(unsigned beam_azimuth_id, unsigned pol_shift_id)
{
  // Beam oversampling factor in the horizontal plane, from TS38.214 Section 5.2.2.2.1.
  static constexpr unsigned O_1 = 4;
  // Number of cross-polarized antenna elements, from TS38.214 Section 5.2.2.2.1.
  static constexpr unsigned N_1 = 4;
  // Number of possible horizontal beams to choose from.
  static constexpr unsigned nof_beams = O_1 * N_1;
  // Number of possible polarization phase shifts to choose from.
  static constexpr unsigned nof_pol_shifts = 2;

  static constexpr interval<unsigned, false> beam_azimuth_range(0, nof_beams);
  static constexpr interval<unsigned, false> pol_phase_shift_range(0, nof_pol_shifts);

  ocudu_assert(beam_azimuth_range.contains(beam_azimuth_id),
               "The given beam azimuth identifier i1_1 (i.e., {}) is out of the range {}",
               beam_azimuth_id,
               beam_azimuth_range);

  ocudu_assert(pol_phase_shift_range.contains(pol_shift_id),
               "The given polarization phase shift i2 (i.e., {}) is out of the range {}",
               pol_shift_id,
               pol_phase_shift_range);

  // Precoding weight matrix for three layers mapped into eight antenna ports.
  precoding_weight_matrix result(5, 8);

  // Normalized horizontal beam index.
  float l_norm = static_cast<float>(beam_azimuth_id) / static_cast<float>(nof_beams);
  // Normalized vertical beam index.
  float m_norm = 0.0f;
  // Polarization phase shift. This defines the relative phase between the cross-polarized antenna elements.
  float pol_phase_shift_rad = M_PI_2 * static_cast<float>(pol_shift_id);

  // Build layer 0 beam coefficients.
  make_layer_type1_sp_mode1(result, 0, N_1, 1, l_norm, m_norm, pol_phase_shift_rad);

  // Build layer 1 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 0.
  make_layer_type1_sp_mode1(result, 1, N_1, 1, l_norm, m_norm, pol_phase_shift_rad + M_PI);

  // Build layer 2 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id + O_1) / static_cast<float>(nof_beams);
  make_layer_type1_sp_mode1(result, 2, N_1, 1, l_norm, m_norm, 0.0f);

  // Build layer 3 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 2.
  make_layer_type1_sp_mode1(result, 3, N_1, 1, l_norm, m_norm, M_PI);

  // Build layer 4 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id + 2 * O_1) / static_cast<float>(nof_beams);
  make_layer_type1_sp_mode1(result, 4, N_1, 1, l_norm, m_norm, 0.0f);

  // Apply scaling.
  float scaling = 1.0F / std::sqrt(result.get_nof_layers() * result.get_nof_ports());
  result *= scaling;

  return result;
}

precoding_weight_matrix
ocudu::make_five_layer_eight_ports_type1_sp(unsigned beam_azimuth_id, unsigned beam_elevation_id, unsigned pol_shift_id)
{
  // Beam oversampling factor in the horizontal and vertical plane, from TS38.214 Section 5.2.2.2.1. The index is
  // dropped given that it is the same in both planes, i.e., O1 = O2 = 4.
  static constexpr unsigned O = 4;
  // Number of cross-polarized antenna elements, from TS38.214 Section 5.2.2.2.1. The index is dropped given that it is
  // them same in both planes, i.e., N1 = N2 = 2.
  static constexpr unsigned N = 2;
  // Number of possible beams per plane to choose from.
  static constexpr unsigned nof_beams_per_plane = O * N;
  // Number of possible polarization phase shifts to choose from.
  static constexpr unsigned nof_pol_shifts = 2;

  static constexpr interval<unsigned, false> beam_azimuth_range(0, nof_beams_per_plane);
  static constexpr interval<unsigned, false> beam_elevation_range(0, nof_beams_per_plane);
  static constexpr interval<unsigned, false> pol_phase_shift_range(0, nof_pol_shifts);

  ocudu_assert(beam_azimuth_range.contains(beam_azimuth_id),
               "The given beam azimuth identifier i1_1 (i.e., {}) is out of the range {}",
               beam_azimuth_id,
               beam_azimuth_range);

  ocudu_assert(beam_elevation_range.contains(beam_elevation_id),
               "The given beam elevation identifier i1_2 (i.e., {}) is out of the range {}",
               beam_elevation_id,
               beam_elevation_range);

  ocudu_assert(pol_phase_shift_range.contains(pol_shift_id),
               "The given polarization phase shift i2 (i.e., {}) is out of the range {}",
               pol_shift_id,
               pol_phase_shift_range);

  // Precoding weight matrix for three layers mapped into eight antenna ports.
  precoding_weight_matrix result(5, 8);

  // Normalized horizontal beam index.
  float l_norm = static_cast<float>(beam_azimuth_id) / static_cast<float>(nof_beams_per_plane);
  // Normalized vertical beam index.
  float m_norm = static_cast<float>(beam_elevation_id) / static_cast<float>(nof_beams_per_plane);
  ;
  // Polarization phase shift. This defines the relative phase between the cross-polarized antenna elements.
  float pol_phase_shift_rad = M_PI_2 * static_cast<float>(pol_shift_id);

  // Build layer 0 beam coefficients.
  make_layer_type1_sp_mode1(result, 0, N, N, l_norm, m_norm, pol_phase_shift_rad);

  // Build layer 1 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 0.
  make_layer_type1_sp_mode1(result, 1, N, N, l_norm, m_norm, pol_phase_shift_rad + M_PI);

  // Build layer 2 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id + O) / static_cast<float>(nof_beams_per_plane);
  make_layer_type1_sp_mode1(result, 2, N, N, l_norm, m_norm, 0.0f);

  // Build layer 3 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 2.
  make_layer_type1_sp_mode1(result, 3, N, N, l_norm, m_norm, M_PI);

  // Build layer 4 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  m_norm = static_cast<float>(beam_elevation_id + O) / static_cast<float>(nof_beams_per_plane);
  make_layer_type1_sp_mode1(result, 4, N, N, l_norm, m_norm, 0.0f);

  // Apply scaling.
  float scaling = 1.0F / std::sqrt(result.get_nof_layers() * result.get_nof_ports());
  result *= scaling;

  return result;
}

precoding_weight_matrix ocudu::make_six_layer_eight_ports_type1_sp(unsigned beam_azimuth_id, unsigned pol_shift_id)
{
  // Beam oversampling factor in the horizontal plane, from TS38.214 Section 5.2.2.2.1.
  static constexpr unsigned O_1 = 4;
  // Number of cross-polarized antenna elements, from TS38.214 Section 5.2.2.2.1.
  static constexpr unsigned N_1 = 4;
  // Number of possible horizontal beams to choose from.
  static constexpr unsigned nof_beams = O_1 * N_1;
  // Number of possible polarization phase shifts to choose from.
  static constexpr unsigned nof_pol_shifts = 2;

  static constexpr interval<unsigned, false> beam_azimuth_range(0, nof_beams);
  static constexpr interval<unsigned, false> pol_phase_shift_range(0, nof_pol_shifts);

  ocudu_assert(beam_azimuth_range.contains(beam_azimuth_id),
               "The given beam azimuth identifier i1_1 (i.e., {}) is out of the range {}",
               beam_azimuth_id,
               beam_azimuth_range);

  ocudu_assert(pol_phase_shift_range.contains(pol_shift_id),
               "The given polarization phase shift i2 (i.e., {}) is out of the range {}",
               pol_shift_id,
               pol_phase_shift_range);

  // Precoding weight matrix for three layers mapped into eight antenna ports.
  precoding_weight_matrix result(6, 8);

  // Normalized horizontal beam index.
  float l_norm = static_cast<float>(beam_azimuth_id) / static_cast<float>(nof_beams);
  // Normalized vertical beam index.
  float m_norm = 0.0f;
  // Polarization phase shift. This defines the relative phase between the cross-polarized antenna elements.
  float pol_phase_shift_rad = M_PI_2 * static_cast<float>(pol_shift_id);

  // Build layer 0 beam coefficients.
  make_layer_type1_sp_mode1(result, 0, N_1, 1, l_norm, m_norm, pol_phase_shift_rad);

  // Build layer 1 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 0.
  make_layer_type1_sp_mode1(result, 1, N_1, 1, l_norm, m_norm, pol_phase_shift_rad + M_PI);

  // Build layer 2 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id + O_1) / static_cast<float>(nof_beams);
  make_layer_type1_sp_mode1(result, 2, N_1, 1, l_norm, m_norm, 0.0f);

  // Build layer 3 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 2.
  make_layer_type1_sp_mode1(result, 3, N_1, 1, l_norm, m_norm, M_PI);

  // Build layer 4 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id + 2 * O_1) / static_cast<float>(nof_beams);
  make_layer_type1_sp_mode1(result, 4, N_1, 1, l_norm, m_norm, 0.0f);

  // Build layer 5 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 4.
  make_layer_type1_sp_mode1(result, 5, N_1, 1, l_norm, m_norm, M_PI);

  // Apply scaling.
  float scaling = 1.0F / std::sqrt(result.get_nof_layers() * result.get_nof_ports());
  result *= scaling;

  return result;
}

precoding_weight_matrix
ocudu::make_six_layer_eight_ports_type1_sp(unsigned beam_azimuth_id, unsigned beam_elevation_id, unsigned pol_shift_id)
{
  // Beam oversampling factor in the horizontal and vertical plane, from TS38.214 Section 5.2.2.2.1. The index is
  // dropped given that it is the same in both planes, i.e., O1 = O2 = 4.
  static constexpr unsigned O = 4;
  // Number of cross-polarized antenna elements, from TS38.214 Section 5.2.2.2.1. The index is dropped given that it is
  // them same in both planes, i.e., N1 = N2 = 2.
  static constexpr unsigned N = 2;
  // Number of possible beams per plane to choose from.
  static constexpr unsigned nof_beams_per_plane = O * N;
  // Number of possible polarization phase shifts to choose from.
  static constexpr unsigned nof_pol_shifts = 2;

  static constexpr interval<unsigned, false> beam_azimuth_range(0, nof_beams_per_plane);
  static constexpr interval<unsigned, false> beam_elevation_range(0, nof_beams_per_plane);
  static constexpr interval<unsigned, false> pol_phase_shift_range(0, nof_pol_shifts);

  ocudu_assert(beam_azimuth_range.contains(beam_azimuth_id),
               "The given beam azimuth identifier i1_1 (i.e., {}) is out of the range {}",
               beam_azimuth_id,
               beam_azimuth_range);

  ocudu_assert(beam_elevation_range.contains(beam_elevation_id),
               "The given beam elevation identifier i1_2 (i.e., {}) is out of the range {}",
               beam_elevation_id,
               beam_elevation_range);

  ocudu_assert(pol_phase_shift_range.contains(pol_shift_id),
               "The given polarization phase shift i2 (i.e., {}) is out of the range {}",
               pol_shift_id,
               pol_phase_shift_range);

  // Precoding weight matrix for three layers mapped into eight antenna ports.
  precoding_weight_matrix result(6, 8);

  // Normalized horizontal beam index.
  float l_norm = static_cast<float>(beam_azimuth_id) / static_cast<float>(nof_beams_per_plane);
  // Normalized vertical beam index.
  float m_norm = static_cast<float>(beam_elevation_id) / static_cast<float>(nof_beams_per_plane);
  ;
  // Polarization phase shift. This defines the relative phase between the cross-polarized antenna elements.
  float pol_phase_shift_rad = M_PI_2 * static_cast<float>(pol_shift_id);

  // Build layer 0 beam coefficients.
  make_layer_type1_sp_mode1(result, 0, N, N, l_norm, m_norm, pol_phase_shift_rad);

  // Build layer 1 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 0.
  make_layer_type1_sp_mode1(result, 1, N, N, l_norm, m_norm, pol_phase_shift_rad + M_PI);

  // Build layer 2 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id + O) / static_cast<float>(nof_beams_per_plane);
  make_layer_type1_sp_mode1(result, 2, N, N, l_norm, m_norm, 0.0f);

  // Build layer 3 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 2.
  make_layer_type1_sp_mode1(result, 3, N, N, l_norm, m_norm, M_PI);

  // Build layer 4 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  m_norm = static_cast<float>(beam_elevation_id + O) / static_cast<float>(nof_beams_per_plane);
  make_layer_type1_sp_mode1(result, 4, N, N, l_norm, m_norm, 0.0f);

  // Build layer 5 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 4.
  make_layer_type1_sp_mode1(result, 5, N, N, l_norm, m_norm, M_PI);

  // Apply scaling.
  float scaling = 1.0F / std::sqrt(result.get_nof_layers() * result.get_nof_ports());
  result *= scaling;

  return result;
}

precoding_weight_matrix ocudu::make_seven_layer_eight_ports_type1_sp(unsigned beam_azimuth_id, unsigned pol_shift_id)
{
  // Beam oversampling factor in the horizontal plane, from TS38.214 Section 5.2.2.2.1.
  static constexpr unsigned O_1 = 4;
  // Number of cross-polarized antenna elements, from TS38.214 Section 5.2.2.2.1.
  static constexpr unsigned N_1 = 4;
  // Number of possible horizontal beams to choose from.
  static constexpr unsigned nof_beams = O_1 * N_1 / 2;
  // Number of possible polarization phase shifts to choose from.
  static constexpr unsigned nof_pol_shifts = 2;

  static constexpr interval<unsigned, false> beam_azimuth_range(0, nof_beams);
  static constexpr interval<unsigned, false> pol_phase_shift_range(0, nof_pol_shifts);

  ocudu_assert(beam_azimuth_range.contains(beam_azimuth_id),
               "The given beam azimuth identifier i1_1 (i.e., {}) is out of the range {}",
               beam_azimuth_id,
               beam_azimuth_range);

  ocudu_assert(pol_phase_shift_range.contains(pol_shift_id),
               "The given polarization phase shift i2 (i.e., {}) is out of the range {}",
               pol_shift_id,
               pol_phase_shift_range);

  // Precoding weight matrix for three layers mapped into eight antenna ports.
  precoding_weight_matrix result(7, 8);

  // Normalized horizontal beam index.
  float l_norm = static_cast<float>(beam_azimuth_id) / static_cast<float>(nof_beams);
  // Normalized vertical beam index.
  float m_norm = 0.0f;
  // Polarization phase shift. This defines the relative phase between the cross-polarized antenna elements.
  float pol_phase_shift_rad = M_PI_2 * static_cast<float>(pol_shift_id);

  // Build layer 0 beam coefficients.
  make_layer_type1_sp_mode1(result, 0, N_1, 1, l_norm, m_norm, pol_phase_shift_rad);

  // Build layer 1 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 0.
  make_layer_type1_sp_mode1(result, 1, N_1, 1, l_norm, m_norm, pol_phase_shift_rad + M_PI);

  // Build layer 2 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id + O_1) / static_cast<float>(nof_beams);
  make_layer_type1_sp_mode1(result, 2, N_1, 1, l_norm, m_norm, pol_phase_shift_rad);

  // Build layer 3 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id + 2 * O_1) / static_cast<float>(nof_beams);
  make_layer_type1_sp_mode1(result, 3, N_1, 1, l_norm, m_norm, 0.0f);

  // Build layer 4 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 3.
  make_layer_type1_sp_mode1(result, 4, N_1, 1, l_norm, m_norm, M_PI);

  // Build layer 5 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id + 3 * O_1) / static_cast<float>(nof_beams);
  make_layer_type1_sp_mode1(result, 5, N_1, 1, l_norm, m_norm, 0.0f);

  // Build layer 6 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 5.
  make_layer_type1_sp_mode1(result, 6, N_1, 1, l_norm, m_norm, M_PI);

  // Apply scaling.
  float scaling = 1.0F / std::sqrt(result.get_nof_layers() * result.get_nof_ports());
  result *= scaling;

  return result;
}

precoding_weight_matrix ocudu::make_seven_layer_eight_ports_type1_sp(unsigned beam_azimuth_id,
                                                                     unsigned beam_elevation_id,
                                                                     unsigned pol_shift_id)
{
  // Beam oversampling factor in the horizontal and vertical plane, from TS38.214 Section 5.2.2.2.1. The index is
  // dropped given that it is the same in both planes, i.e., O1 = O2 = 4.
  static constexpr unsigned O = 4;
  // Number of cross-polarized antenna elements, from TS38.214 Section 5.2.2.2.1. The index is dropped given that it is
  // them same in both planes, i.e., N1 = N2 = 2.
  static constexpr unsigned N = 2;
  // Number of possible beams per plane to choose from.
  static constexpr unsigned nof_beams_per_plane = O * N;
  // Number of possible polarization phase shifts to choose from.
  static constexpr unsigned nof_pol_shifts = 2;

  static constexpr interval<unsigned, false> beam_azimuth_range(0, nof_beams_per_plane);
  static constexpr interval<unsigned, false> beam_elevation_range(0, nof_beams_per_plane);
  static constexpr interval<unsigned, false> pol_phase_shift_range(0, nof_pol_shifts);

  ocudu_assert(beam_azimuth_range.contains(beam_azimuth_id),
               "The given beam azimuth identifier i1_1 (i.e., {}) is out of the range {}",
               beam_azimuth_id,
               beam_azimuth_range);

  ocudu_assert(beam_elevation_range.contains(beam_elevation_id),
               "The given beam elevation identifier i1_2 (i.e., {}) is out of the range {}",
               beam_elevation_id,
               beam_elevation_range);

  ocudu_assert(pol_phase_shift_range.contains(pol_shift_id),
               "The given polarization phase shift i2 (i.e., {}) is out of the range {}",
               pol_shift_id,
               pol_phase_shift_range);

  // Precoding weight matrix for three layers mapped into eight antenna ports.
  precoding_weight_matrix result(7, 8);

  // Normalized horizontal beam index.
  float l_norm = static_cast<float>(beam_azimuth_id) / static_cast<float>(nof_beams_per_plane);
  // Normalized vertical beam index.
  float m_norm = static_cast<float>(beam_elevation_id) / static_cast<float>(nof_beams_per_plane);
  ;
  // Polarization phase shift. This defines the relative phase between the cross-polarized antenna elements.
  float pol_phase_shift_rad = M_PI_2 * static_cast<float>(pol_shift_id);

  // Build layer 0 beam coefficients.
  make_layer_type1_sp_mode1(result, 0, N, N, l_norm, m_norm, pol_phase_shift_rad);

  // Build layer 1 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 0.
  make_layer_type1_sp_mode1(result, 1, N, N, l_norm, m_norm, pol_phase_shift_rad + M_PI);

  // Build layer 2 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id + O) / static_cast<float>(nof_beams_per_plane);
  make_layer_type1_sp_mode1(result, 2, N, N, l_norm, m_norm, pol_phase_shift_rad);

  // Build layer 3 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id) / static_cast<float>(nof_beams_per_plane);
  m_norm = static_cast<float>(beam_elevation_id + O) / static_cast<float>(nof_beams_per_plane);
  make_layer_type1_sp_mode1(result, 3, N, N, l_norm, m_norm, 0.0f);

  // Build layer 4 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 3.
  make_layer_type1_sp_mode1(result, 4, N, N, l_norm, m_norm, M_PI);

  // Build layer 5 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id + O) / static_cast<float>(nof_beams_per_plane);
  m_norm = static_cast<float>(beam_elevation_id + O) / static_cast<float>(nof_beams_per_plane);
  make_layer_type1_sp_mode1(result, 5, N, N, l_norm, m_norm, 0.0f);

  // Build layer 6 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 5.
  make_layer_type1_sp_mode1(result, 6, N, N, l_norm, m_norm, M_PI);

  // Apply scaling.
  float scaling = 1.0F / std::sqrt(result.get_nof_layers() * result.get_nof_ports());
  result *= scaling;

  return result;
}

precoding_weight_matrix ocudu::make_eight_layer_eight_ports_type1_sp(unsigned beam_azimuth_id, unsigned pol_shift_id)
{
  // Beam oversampling factor in the horizontal plane, from TS38.214 Section 5.2.2.2.1.
  static constexpr unsigned O_1 = 4;
  // Number of cross-polarized antenna elements, from TS38.214 Section 5.2.2.2.1.
  static constexpr unsigned N_1 = 4;
  // Number of possible horizontal beams to choose from.
  static constexpr unsigned nof_beams = O_1 * N_1 / 2;
  // Number of possible polarization phase shifts to choose from.
  static constexpr unsigned nof_pol_shifts = 2;

  static constexpr interval<unsigned, false> beam_azimuth_range(0, nof_beams);
  static constexpr interval<unsigned, false> pol_phase_shift_range(0, nof_pol_shifts);

  ocudu_assert(beam_azimuth_range.contains(beam_azimuth_id),
               "The given beam azimuth identifier i1_1 (i.e., {}) is out of the range {}",
               beam_azimuth_id,
               beam_azimuth_range);

  ocudu_assert(pol_phase_shift_range.contains(pol_shift_id),
               "The given polarization phase shift i2 (i.e., {}) is out of the range {}",
               pol_shift_id,
               pol_phase_shift_range);

  // Precoding weight matrix for three layers mapped into eight antenna ports.
  precoding_weight_matrix result(8, 8);

  // Normalized horizontal beam index.
  float l_norm = static_cast<float>(beam_azimuth_id) / static_cast<float>(nof_beams);
  // Normalized vertical beam index.
  float m_norm = 0.0f;
  // Polarization phase shift. This defines the relative phase between the cross-polarized antenna elements.
  float pol_phase_shift_rad = M_PI_2 * static_cast<float>(pol_shift_id);

  // Build layer 0 beam coefficients.
  make_layer_type1_sp_mode1(result, 0, N_1, 1, l_norm, m_norm, pol_phase_shift_rad);

  // Build layer 1 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 0.
  make_layer_type1_sp_mode1(result, 1, N_1, 1, l_norm, m_norm, pol_phase_shift_rad + M_PI);

  // Build layer 2 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id + O_1) / static_cast<float>(nof_beams);
  make_layer_type1_sp_mode1(result, 2, N_1, 1, l_norm, m_norm, pol_phase_shift_rad);

  // Build layer 3 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 2.
  make_layer_type1_sp_mode1(result, 3, N_1, 1, l_norm, m_norm, pol_phase_shift_rad + M_PI);

  // Build layer 4 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id + 2 * O_1) / static_cast<float>(nof_beams);
  make_layer_type1_sp_mode1(result, 4, N_1, 1, l_norm, m_norm, 0.0f);

  // Build layer 5 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 4.
  make_layer_type1_sp_mode1(result, 5, N_1, 1, l_norm, m_norm, M_PI);

  // Build layer 6 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id + 3 * O_1) / static_cast<float>(nof_beams);
  make_layer_type1_sp_mode1(result, 6, N_1, 1, l_norm, m_norm, 0.0f);

  // Build layer 7 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 6.
  make_layer_type1_sp_mode1(result, 7, N_1, 1, l_norm, m_norm, M_PI);

  // Apply scaling.
  float scaling = 1.0F / std::sqrt(result.get_nof_layers() * result.get_nof_ports());
  result *= scaling;

  return result;
}

precoding_weight_matrix ocudu::make_eight_layer_eight_ports_type1_sp(unsigned beam_azimuth_id,
                                                                     unsigned beam_elevation_id,
                                                                     unsigned pol_shift_id)
{
  // Beam oversampling factor in the horizontal and vertical plane, from TS38.214 Section 5.2.2.2.1. The index is
  // dropped given that it is the same in both planes, i.e., O1 = O2 = 4.
  static constexpr unsigned O = 4;
  // Number of cross-polarized antenna elements, from TS38.214 Section 5.2.2.2.1. The index is dropped given that it is
  // them same in both planes, i.e., N1 = N2 = 2.
  static constexpr unsigned N = 2;
  // Number of possible beams per plane to choose from.
  static constexpr unsigned nof_beams_per_plane = O * N;
  // Number of possible polarization phase shifts to choose from.
  static constexpr unsigned nof_pol_shifts = 2;

  static constexpr interval<unsigned, false> beam_azimuth_range(0, nof_beams_per_plane);
  static constexpr interval<unsigned, false> beam_elevation_range(0, nof_beams_per_plane);
  static constexpr interval<unsigned, false> pol_phase_shift_range(0, nof_pol_shifts);

  ocudu_assert(beam_azimuth_range.contains(beam_azimuth_id),
               "The given beam azimuth identifier i1_1 (i.e., {}) is out of the range {}",
               beam_azimuth_id,
               beam_azimuth_range);

  ocudu_assert(beam_elevation_range.contains(beam_elevation_id),
               "The given beam elevation identifier i1_2 (i.e., {}) is out of the range {}",
               beam_elevation_id,
               beam_elevation_range);

  ocudu_assert(pol_phase_shift_range.contains(pol_shift_id),
               "The given polarization phase shift i2 (i.e., {}) is out of the range {}",
               pol_shift_id,
               pol_phase_shift_range);

  // Precoding weight matrix for three layers mapped into eight antenna ports.
  precoding_weight_matrix result(8, 8);

  // Normalized horizontal beam index.
  float l_norm = static_cast<float>(beam_azimuth_id) / static_cast<float>(nof_beams_per_plane);
  // Normalized vertical beam index.
  float m_norm = static_cast<float>(beam_elevation_id) / static_cast<float>(nof_beams_per_plane);
  ;
  // Polarization phase shift. This defines the relative phase between the cross-polarized antenna elements.
  float pol_phase_shift_rad = M_PI_2 * static_cast<float>(pol_shift_id);

  // Build layer 0 beam coefficients.
  make_layer_type1_sp_mode1(result, 0, N, N, l_norm, m_norm, pol_phase_shift_rad);

  // Build layer 1 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 0.
  make_layer_type1_sp_mode1(result, 1, N, N, l_norm, m_norm, pol_phase_shift_rad + M_PI);

  // Build layer 2 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id + O) / static_cast<float>(nof_beams_per_plane);
  make_layer_type1_sp_mode1(result, 2, N, N, l_norm, m_norm, pol_phase_shift_rad);

  // Build layer 3 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 2.
  make_layer_type1_sp_mode1(result, 3, N, N, l_norm, m_norm, pol_phase_shift_rad + M_PI);

  // Build layer 4 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id) / static_cast<float>(nof_beams_per_plane);
  m_norm = static_cast<float>(beam_elevation_id + O) / static_cast<float>(nof_beams_per_plane);
  make_layer_type1_sp_mode1(result, 4, N, N, l_norm, m_norm, 0.0f);

  // Build layer 5 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 4.
  make_layer_type1_sp_mode1(result, 5, N, N, l_norm, m_norm, M_PI);

  // Build layer 6 beam coefficients. Recalculate phase increment for adjacent beam antenna elements.
  l_norm = static_cast<float>(beam_azimuth_id + O) / static_cast<float>(nof_beams_per_plane);
  make_layer_type1_sp_mode1(result, 6, N, N, l_norm, m_norm, 0.0f);

  // Build layer 7 beam coefficients. The phase increment for adjacent beam antenna elements is the same as layer 6.
  make_layer_type1_sp_mode1(result, 7, N, N, l_norm, m_norm, M_PI);

  // Apply scaling.
  float scaling = 1.0F / std::sqrt(result.get_nof_layers() * result.get_nof_ports());
  result *= scaling;

  return result;
}
