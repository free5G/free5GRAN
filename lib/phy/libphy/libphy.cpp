/*
 * Copyright 2020-2021 Telecom Paris

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 */

#include "libphy.h"

#include <fftw3.h>

#include <algorithm>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <chrono>
#include <complex>
#include <iostream>
#include <vector>
#include <fstream>

#include "../../utils/common_utils/common_utils.h"
#include "../../utils/sequence_generator/sequence_generator.h"
#include "../../variables/common_variables/common_variables.h"
#include "../physical_channel/physical_channel.h"
#include "../synchronization/synchronization.h"
#include "../transport_channel/transport_channel.h"

using namespace std;

void free5GRAN::phy::signal_processing::channelEstimation(
    complex<float>* pilots,
    complex<float>* reference_pilot,
    vector<vector<int>>& pilot_indexes,
    vector<vector<complex<float>>>& coefficients,
    float& snr,
    int num_sc,
    int num_symbols,
    int pilot_size) {
  /**
   * \fn channelEstimation
   * \brief Estimate channel coefficients for equalization
   * \details
   * - Compute channel coefficients for pilots
   * - Linear interpolation in frequency domain
   * - Linear interpolation in time domain
   *
   * \param[in] pilots: Signal received pilots
   * \param[in] reference_pilot: Generated pilots
   * \param[in] pilot_indexes: Pilot indexes in the signal
   * \param[out] coefficients: Channel coefficient returned by the function
   * \param[out] snr: Computed SNR
   * \param[in] num_sc: Number of subcarriers in the signal
   * \param[in] num_symbols: Number of symbols in the signal
   * \param[in] pilot_size: Pilot size
   */

  // Initializing variables
  complex<float> noise_coef;
  vector<int> found_indexes, symbols_with_pilot, symbols_with_no_pilot;
  vector<complex<float>> coef_same_sc;
  int upper_index, lower_index, step, step_width;
  float re_int, im_int;
  float mean_power = 0;
  float mean_noise = 0;

  // Computing pilots transport_channel coefficients
  for (int i = 0; i < pilot_size; i++) {
    coefficients[pilot_indexes[0][i]][pilot_indexes[1][i]] =
        pilots[i] * conj(reference_pilot[i]) /
        (float)pow(abs(reference_pilot[i]), 2);
  }

  // Interpolating in frequency domain to complete transport_channel
  // coefficients grid
  for (int symbol = 0; symbol < num_symbols; symbol++) {
    found_indexes.clear();
    /*
     * Adding in found_indexes all the pilot index that are in the current
     * symbol
     */
    for (int k = 0; k < pilot_size; k++) {
      if (pilot_indexes[0][k] == symbol) {
        found_indexes.push_back(pilot_indexes[1][k]);
      }
    }

    if (found_indexes.size() != 0) {
      symbols_with_pilot.push_back(symbol);
      /*
       * Linear interpolation. Looping over each subcarrier of the current
       * symbol
       */
      for (int sc = 0; sc < num_sc; sc++) {
        // If the current resource element is not a pilot
        if (find(found_indexes.begin(), found_indexes.end(), sc) ==
            found_indexes.end()) {
          // Get lower and upper pilot indexes for interpolating. If no lower,
          // take the two closest upper pilots and respectively if no upper
          // pilot
          if (sc < found_indexes[0]) {
            lower_index = found_indexes[0];
            upper_index = found_indexes[1];
          } else if (sc > found_indexes[found_indexes.size() - 1]) {
            lower_index = found_indexes[found_indexes.size() - 2];
            upper_index = found_indexes[found_indexes.size() - 1];
          } else {
            for (int k = 0; k < found_indexes.size() - 1; k++) {
              if (found_indexes[k] < sc && found_indexes[k + 1] > sc) {
                lower_index = found_indexes[k];
                upper_index = found_indexes[k + 1];
                break;
              }
            }
          }
          step = sc - lower_index;
          step_width = upper_index - lower_index;
          // Interpolate real and imaginary part of the resource element
          // transport_channel coefficient
          re_int = step *
                       (real(coefficients[symbol][upper_index]) -
                        real(coefficients[symbol][lower_index])) /
                       step_width +
                   real(coefficients[symbol][lower_index]);
          im_int = step *
                       (imag(coefficients[symbol][upper_index]) -
                        imag(coefficients[symbol][lower_index])) /
                       step_width +
                   imag(coefficients[symbol][lower_index]);
          coefficients[symbol][sc] = complex<float>(re_int, im_int);
          // Compute mean power over all the resource elements
          mean_power += pow(abs(coefficients[symbol][sc]), 2);
        }
      }
    } else {
      symbols_with_no_pilot.push_back(symbol);
    }
  }

  /*
   *  Time domain linear interpolation
   */

  for (int s : symbols_with_no_pilot) {
    if (s < symbols_with_pilot[0]) {
      lower_index = symbols_with_pilot[0];
      upper_index = symbols_with_pilot[1];
    } else if (s > symbols_with_pilot[symbols_with_pilot.size() - 1]) {
      lower_index = symbols_with_pilot[symbols_with_pilot.size() - 2];
      upper_index = symbols_with_pilot[symbols_with_pilot.size() - 1];
    } else {
      for (int k = 0; k < symbols_with_pilot.size() - 1; k++) {
        if (symbols_with_pilot[k] < s && symbols_with_pilot[k + 1] > s) {
          lower_index = symbols_with_pilot[k];
          upper_index = symbols_with_pilot[k + 1];
          break;
        }
      }
    }
    step = s - lower_index;
    step_width = upper_index - lower_index;

    for (int sc = 0; sc < num_sc; sc++) {
      re_int = step *
                   (real(coefficients[upper_index][sc]) -
                    real(coefficients[lower_index][sc])) /
                   step_width +
               real(coefficients[lower_index][sc]);
      im_int = step *
                   (imag(coefficients[upper_index][sc]) -
                    imag(coefficients[lower_index][sc])) /
                   step_width +
               imag(coefficients[lower_index][sc]);
      coefficients[s][sc] = complex<float>(re_int, im_int);
      // Compute mean power over all the resource elements
      mean_power += pow(abs(coefficients[s][sc]), 2);
    }
  }
  // Computing signal power.
  mean_power /= num_sc * num_symbols;
  mean_power = 10 * log10(mean_power);

  // Computing signal noise by time averaging the mean standard deviation of the
  // coefficients on all the subcarriers
  for (int symbol = 0; symbol < num_symbols; symbol++) {
    noise_coef = 0;
    for (int sc = 0; sc < num_sc; sc++) {
      noise_coef += coefficients[symbol][sc];
    }
    noise_coef /= num_sc;
    for (int sc = 0; sc < num_sc; sc++) {
      mean_noise += pow(abs(coefficients[symbol][sc] - noise_coef), 2) /
                    num_sc * num_symbols;
    }
  }
  mean_noise = 10 * log10(mean_noise);

  snr = mean_power - mean_noise;
}

void free5GRAN::phy::signal_processing::hard_demodulation(
    vector<complex<float>> signal,
    int* bits,
    int signal_length,
    int modulation_scheme) {
  /**
   * \brief Samples hard demodulation
   * \param[in] signal: Input IQ data to be demodulated
   * \param[in] signal_length: Number of symbols to be demodulated
   * \param[in] modulation_scheme: Modulation used (0: BPSK, 1: QPSK)
   * \param[out] bits: Output hard bits
   */
  for (int i = 0; i < signal_length; i++) {
    /*
     * BPSK demodulation
     */
    if (modulation_scheme == 0) {
      if (real(signal[i]) > 0) {
        bits[i] = 0;
      } else {
        bits[i] = 1;
      }
    }
    /*
     * QPSK demodulation
     */
    else if (modulation_scheme == 1) {
      if (real(signal[i]) > 0) {
        bits[2 * i] = 0;
      } else {
        bits[2 * i] = 1;
      }
      if (imag(signal[i]) > 0) {
        bits[2 * i + 1] = 0;
      } else {
        bits[2 * i + 1] = 1;
      }
    }
  }
}

void free5GRAN::phy::signal_processing::soft_demodulation(
    vector<complex<float>> signal,
    double* soft_bits,
    int signal_length,
    int modulation_scheme) {
  /**
   * \fn soft_demodulation
   * \brief Samples soft demodulation
   * \details
   * - Compute minimum distance to 1 and 0 candidate sample
   * - Take the difference between shortest distance to 1 and shortest distance
   * to 0
   *
   * \param[in] signal: Input IQ data to be demodulated
   * \param[out] bits: Output LLR bits
   * \param[in] signal_length: Number of symbols to be demodulated
   * \param[in] modulation_scheme: Modulation used (0: BPSK, 1: QPSK)
   */
  /*
   * QPSK soft-demodulation
   */
  if (modulation_scheme == 1) {
    complex<float> s_00, s_01, s_10, s_11;
    double const_power = 1 / sqrt(2);
    s_00 = complex<double>(const_power, const_power);
    s_01 = complex<double>(const_power, -const_power);
    s_10 = complex<double>(-const_power, const_power);
    s_11 = complex<double>(-const_power, -const_power);
    for (int i = 0; i < signal_length; i++) {
      double l_01, l_02, l_11, l_12;
      // Computing distance from current signal point to each of the four
      // theoretical QPSK points
      l_01 = abs(signal[i] - s_00);
      l_02 = abs(signal[i] - s_01);
      l_11 = abs(signal[i] - s_10);
      l_12 = abs(signal[i] - s_11);
      // Determine LLR soft bit of the first QPSK bit
      soft_bits[2 * i] = pow(min(l_11, l_12), 2) - pow(min(l_01, l_02), 2);
      // Computing distance from current signal point to each of the four
      // theoretical QPSK points
      l_01 = abs(signal[i] - s_00);
      l_02 = abs(signal[i] - s_10);
      l_11 = abs(signal[i] - s_01);
      l_12 = abs(signal[i] - s_11);
      // Determine LLR soft bit of the second QPSK bit
      soft_bits[2 * i + 1] = pow(min(l_11, l_12), 2) - pow(min(l_01, l_02), 2);
    }
  }
}

void free5GRAN::phy::signal_processing::compute_fine_frequency_offset(
    vector<complex<float>> input_signal,
    int symbol_duration,
    int fft_size,
    int cp_length,
    int scs,
    float& output,
    int num_symbols) {
  /**
   * \fn compute_fine_frequency_offset
   * \brief  Compute fine frequency offset of a received signal by computing
   * phase offset between cyclic prefix and corresponding symbol data.
   * \param[in] input_signal: Received signal
   * \param[in] symbol_duration: Number of samples per symbol
   * \param[in] fft_size: Number of samples in a symbol, after removing cyclic
   * prefix \param[in] cp_length: Cyclic prefix size \param[in] scs: Subcarrier
   * spacing \param[out] output: Frequency offset in Hertz \param[in]
   * num_symbols: Number of symbols in the signal
   */

  complex<float> out;
  float phase_offset = 0;

  // Looping over all the symbols
  for (int symbol = 0; symbol < num_symbols; symbol++) {
    // Initialize symbol phase offset to 0
    out = 0;
    // Loop over all the samples of the cyclic prefix
    for (int i = 0; i < cp_length; i++) {
      // Increment symbol phase offset by the result of
      // the correlation of the studied sample of
      // the cyclic prefix with the
      // corresponding sample of the input signal
      out += conj(input_signal[i + symbol * symbol_duration]) *
             input_signal[i + symbol * symbol_duration + fft_size];
    }
    phase_offset += arg(out);
  }
  // Average phase offset over all the symbols
  phase_offset /= num_symbols;
  // Computing frequency offset (output) corresponding
  // to the computed phase offset
  output = scs * phase_offset / (2 * M_PI);
}

void free5GRAN::phy::signal_processing::transpose_signal(
    vector<complex<float>>* input_signal,
    float freq_offset,
    int sample_rate,
    int input_length) {
  /**
   * \fn transpose_signal
   * \brief Shift signal in frequency domain by a frequency offset
   * \param[in] input_signal: Input signal to be shifted
   * \param[in] freq_offset: Frequency offset
   * \param[in] sample_rate: Signal sample rate
   * \param[in] input_length: Input signal length
   */
  complex<float> j(0, 1);
  for (int i = 0; i < input_length; i++) {
    (*input_signal)[i] =
        (*input_signal)[i] *
        exp(complex<float>(-2, 0) * j * complex<float>(M_PI, 0) * freq_offset *
            complex<float>((float)i, 0) / complex<float>(sample_rate, 0));
  }
}

void free5GRAN::phy::signal_processing::channel_demapper(
    vector<vector<complex<float>>>& input_signal,
    vector<vector<vector<int>>>& ref,
    complex<float>** output_channels,
    vector<vector<vector<int>>>& output_indexes,
    int num_channels,
    int num_symbols,
    int num_sc) {
  /**
   * \fn channel_demapper
   * \brief Consumes an input signal and returns different channels containing
   * corresponding data, based on ref indexes \param[in] input_signal: Input
   * signal to be de-mapped \param[in] ref: Array containing the indexes of the
   * different channels inside the OFDM grid \param[in] channel_sizes: Array
   * containing the outpu transport_channel sizes \param[out] output_channels:
   * Channels symbols \param[out] output_indexes: Indexes of the output_channels
   * elements \param[in] num_channels: Number of channels \param[in]
   * num_symbols: Number of symbols in the OFDM grid \param[in] num_sc: Number
   * of subcarriers in the OFDM grid
   */
  int channel_counter[num_channels];
  for (int i = 0; i < num_channels; i++) {
    channel_counter[i] = 0;
  }

  // input_signal is the input OFDM grid
  // output_channels is a 2D array containing
  // the physical channels
  // output_indexes is a 3D array containing
  // the positions of each sample of the
  // physical channels
  // ref is the reference grid which contains
  // the position of the physical channel samples
  // Loop over each symbol in the OFDM grid
  for (int symbol = 0; symbol < num_symbols; symbol++) {
    // Loop over each subcarrier of the OFDM grid
    // symbol and sc are pointing to a precise RE
    for (int sc = 0; sc < num_sc; sc++) {
      // Loop over each physical channel
      for (int channel = 0; channel < num_channels; channel++) {
        // If the current resource element belongs to the physical channel,
        // store the element and corresponding grid index
        if (ref[channel][symbol][sc] == 1) {
          // Add the current RE sample to the physical channel
          output_channels[channel][channel_counter[channel]] =
              input_signal[symbol][sc];
          // Save the symbol and subcarrier position
          // of the sample for future processing
          output_indexes[channel][0][channel_counter[channel]] = symbol;
          output_indexes[channel][1][channel_counter[channel]] = sc;
          // Increment the channel counter,
          // which counts the number of elements
          // in each physical channel.
          channel_counter[channel]++;
        }
      }
    }
  }
}

auto free5GRAN::phy::signal_processing::compute_freq_from_gscn(int gscn)
    -> double {
  /**
   * \fn compute_freq_from_gscn
   * \brief Computing frequency from received GSCN
   * \standard TS38.104 V15.2.0 Table 5.4.3.1-1
   *
   * \param[in] gscn: Input GSCN
   */
  int N;
  double freq;
  if (gscn < 7499) {
    int M;
    for (int i = 0; i < 3; i++) {
      M = 2 * i + 1;
      if ((gscn - (M - 3) / 2) % 3 == 0) {
        break;
      }
    }
    N = (gscn - (M - 3) / 2) / 3;
    freq = N * 1.2e6 + M * 50e3;
  } else if (gscn < 22256) {
    N = gscn - 7499;
    freq = 3e9 + N * 1.44e6;
  } else {
    N = gscn - 22256;
    freq = 24250.08e6 + N * 17.28e6;
  }
  return freq;
}

auto free5GRAN::phy::signal_processing::
    compute_pdcch_t0_ss_monitoring_occasions(int pdcch_config,
                                             int pbch_scs,
                                             int common_scs,
                                             int i)
        -> free5GRAN::pdcch_t0ss_monitoring_occasions {
  /**
   * \fn compute_pdcch_t0_ss_monitoring_occasions
   * \brief Compute PDCCH monitoring occasion informations
   * \standard TS38.213 V15.2.0 Section 13
   *
   * \param[in] pdcch_config: PDCCH config received from MIB
   * \param[in] pbch_scs: PBCH subcarrier spacing
   * \param[in] common_scs: BWP subcarrier spacing
   * \param[in] i: SSB index
   */
  free5GRAN::pdcch_t0ss_monitoring_occasions obj = {};
  int index1, index2;
  index1 = pdcch_config / 16;
  index2 = pdcch_config % 16;
  int* table1 = new int[4];
  float* table2;
  if (pbch_scs == 15e3 && common_scs == 15e3) {
    table1 = free5GRAN::TS_38_213_TABLE_13_1[index1];
  } else if (pbch_scs == 15e3 && common_scs == 30e3) {
    table1 = free5GRAN::TS_38_213_TABLE_13_2[index1];
  } else if (pbch_scs == 30e3 && common_scs == 15e3) {
    table1 = free5GRAN::TS_38_213_TABLE_13_3[index1];
  } else if (pbch_scs == 30e3 && common_scs == 30e3) {
    table1 = free5GRAN::TS_38_213_TABLE_13_4[index1];
  }
  table2 = free5GRAN::TS_38_213_TABLE_13_11[index2];

  obj.multiplexing_pattern = table1[0];
  obj.n_rb_coreset = table1[1];
  obj.n_symb_coreset = table1[2];
  obj.offset = table1[3];
  obj.O = table2[0];
  obj.num_ss_slots = table2[1];
  obj.M = table2[2];

  if (table2[3] == -1) {
    if (i % 2 == 0) {
      obj.first_symb_index = 0;
    } else {
      obj.first_symb_index = obj.n_symb_coreset;
    }
  } else {
    obj.first_symb_index = table2[3];
  }

  return obj;
}

void free5GRAN::phy::signal_processing::compute_rb_start_lrb_dci(
    int RIV,
    int n_size_bwp,
    int& lrb,
    int& rb_start) {
  /**
   * \fn compute_rb_start_lrb_dci
   * \brief Compute RB start and LRB from DCI
   * \standard TS38.214 V15.2.0 Section 5.1.2.2.2
   *
   * \param[in] RIV: Frequency domain resource allocation
   * \param[in] n_size_bwp: Active BWP size (in RB)
   * \param[out] lrb: Number of PDSCH RB
   * \param[out] rb_start: First PDSCH RB index in BWP
   */
  lrb = floor(RIV / n_size_bwp) + 1;
  rb_start = RIV - ((lrb - 1) * n_size_bwp);
  if (lrb > n_size_bwp - rb_start) {
    lrb = n_size_bwp - lrb + 2;
    rb_start = n_size_bwp - 1 - rb_start;
  }
}

void free5GRAN::phy::signal_processing::get_pdsch_dmrs_symbols(
    const string& type,
    int duration,
    int additionnal_position,
    int l0,
    int** output,
    int& size) {
  /**
   * \fn get_pdsch_dmrs_symbols
   * \brief Retrieve DMRS symbols
   * \standard TS38.211 V15.2.0 Table 7.4.1.1.2-3
   *
   * \param[in] additionnal_position: Number of additional positions
   * \param[in] l0: First DMRS symbol
   * \param[out] output: DMRS symbols array
   * \param[out] size: Number of DMRS symbols
   */
  if (duration < 8 || additionnal_position == 0) {
    size = 1;
    (*output) = new int[size]{l0};
  } else if (duration < 10) {
    size = 2;
    (*output) = new int[size]{l0, 7};
  } else if (additionnal_position == 1) {
    size = 2;
    if (duration < 13) {
      (*output) = new int[size]{l0, 9};
    } else {
      (*output) = new int[size]{l0, 11};
    }
  } else if (additionnal_position == 2) {
    size = 3;
    if (duration < 13) {
      (*output) = new int[size]{l0, 6, 9};
    } else {
      (*output) = new int[size]{l0, 7, 11};
    }
  } else if (additionnal_position == 3) {
    if (duration < 12) {
      size = 3;
      (*output) = new int[size]{l0, 6, 9};
    } else {
      size = 4;
      (*output) = new int[size]{l0, 5, 8, 11};
    }
  }
}

void free5GRAN::phy::signal_processing::compute_cp_lengths(
    int scs,
    int nfft,
    int is_extended_cp,
    int num_symb_per_subframes,
    int* cp_lengths,
    int* cum_sum_cp_lengths) {
  /**
   * \fn compute_cp_lengths
   * \brief Compute cyclic prefix size
   * \standard TS38.211 V15.2.0 Section 5.3
   *
   * \param[in] scs: Subcarrier spacing (in kHz)
   * \param[in] nfft: FFT/iFFT size
   * \param[in] is_extended_cp: True if current CP is extended CP
   * \param[in] num_symb_per_subframes: Number of symbols per subframe
   * \param[out] cp_lengths: Returned CP
   * \param[out] cum_sum_cp_lengths: Cumulative CP sum
   */
  int nom_cp = ((is_extended_cp) ? 512 : 144);
  int base_cp = nom_cp * nfft / 2048;
  cum_sum_cp_lengths[0] = 0;
  for (int i = 0; i < num_symb_per_subframes; i++) {
    if (i % (num_symb_per_subframes / 2) == 0) {
      cp_lengths[i] = (scs * nfft - num_symb_per_subframes * nfft -
                       (num_symb_per_subframes - 2) * base_cp) /
                      2;
    } else {
      cp_lengths[i] = base_cp;
    }
    if (i < num_symb_per_subframes - 1) {
      cum_sum_cp_lengths[i + 1] = cum_sum_cp_lengths[i] + cp_lengths[i] + nfft;
    }
  }
}

void free5GRAN::phy::signal_processing::compute_cp_lengths(
    int scs,
    int nfft,
    int is_extended_cp,
    int num_symb_per_subframes,
    vector<int>& cp_lengths,
    vector<int>& cum_sum_cp_lengths) {
  /**
   * \fn compute_cp_lengths
   * \brief Compute cyclic prefix size
   * \standard TS38.211 V15.2.0 Section 5.3
   *
   * \param[in] scs: Subcarrier spacing (in kHz)
   * \param[in] nfft: FFT/iFFT size
   * \param[in] is_extended_cp: True if current CP is extended CP
   * \param[in] num_symb_per_subframes: Number of symbols per subframe
   * \param[out] cp_lengths: Returned CP
   * \param[out] cum_sum_cp_lengths: Cumulative CP sum
   */
  int nom_cp = ((is_extended_cp) ? 512 : 144);
  int base_cp = nom_cp * nfft / 2048;
  cum_sum_cp_lengths[0] = 0;
  for (int i = 0; i < num_symb_per_subframes; i++) {
    if (i % (num_symb_per_subframes / 2) == 0) {
      cp_lengths[i] = (scs * nfft - num_symb_per_subframes * nfft -
                       (num_symb_per_subframes - 2) * base_cp) /
                      2;
    } else {
      cp_lengths[i] = base_cp;
    }
    if (i < num_symb_per_subframes - 1) {
      cum_sum_cp_lengths[i + 1] = cum_sum_cp_lengths[i] + cp_lengths[i] + nfft;
    }
  }
}

void free5GRAN::phy::signal_processing::compute_phase_decomp(
    int* cp_lengths,
    int* cum_sum_symb,
    float sampling_rate,
    float f0,
    int num_symb_per_subframes,
    complex<float>* phase_decomp_factor) {
  /**
   * \fn compute_phase_decomp
   * \brief Compute phase decompensation vector from frequency
   * \standard TS38.211 V15.2.0 Section 5.3
   *
   * \param[in] cp_lengths: CP lengths
   * \param[in] cum_sum_symb: CP lengths cumulative sum
   * \param[in] sampling_rate: RF device sample rate
   * \param[in] f0: Frequency in Hz
   * \param[in] num_symb_per_subframes: Number of symbols per subframes
   * \param[out] phase_decomp_factor: Phase de-compensator vector
   */
  for (int i = 0; i < num_symb_per_subframes; i++) {
    float t_start = cum_sum_symb[i] / sampling_rate;
    float t_cp = cp_lengths[i] / sampling_rate;
    phase_decomp_factor[i] =
        exp(complex<float>(0, -2 * f0 * M_PI) * (-t_start - t_cp));
  }
}

auto free5GRAN::phy::signal_processing::compute_nre(int num_symb_pdsch,
                                                    int num_dmrs_symb) -> int {
  /**
   * \fn compute_nre
   * \brief Compute number of PDSCH RE per RB
   * \param[in] num_symb_pdsch: Number of PDSCH symbols
   * \param[in] num_dmrs_symb: Number of DMRS symbols
   * \return number of PDSCH RE per RB
   */
  return 12 * (num_symb_pdsch - num_dmrs_symb);
}

void free5GRAN::phy::signal_processing::fft(
    vector<complex<float>> time_domain_signal,
    vector<vector<complex<float>>>& output_signal,
    int fft_size,
    int* cp_lengths,
    int* cum_sum_symb,
    int num_symbols,
    int num_sc_output,
    int first_symb_index,
    int offset) {
  /**
   * \fn fft
   * \brief Perform FFT on time domain signal to recover frequency domain signal
   * (= RE grid) \param[in] time_domain_signal: Input time domain signal
   * \param[out] output_signal: Output RE grid
   * \param[in] fft_size: FFT size (Number of samples per symbol, excluding CP)
   * \param[in] cp_lengths: Array of CP lengths for 1 subframe (= 1ms)
   * \param[in] cum_sum_symb: Cumulative sum of symbols length in one subframe
   * \param[in] num_symbols: Number of symbols in output RE grid (= Number of
   * rows of output_signal)
   * \param[in] num_sc_output: Number of subcarriers in
   * output RE grid (= Number of columns of output_signal).
   * \param[in] first_symb_index: Index of first symbol to be extracted in
   * frame. \param[in] offset: Number of amples to be left before extracting.
   * Can be used while extracting specific slots in a radio frame.
   */
  // Initializing fft parameters
  auto* fft_in = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * fft_size);
  auto* fft_out = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * fft_size);
  fftw_plan fft_plan =
      fftw_plan_dft_1d(fft_size, fft_in, fft_out, FFTW_FORWARD, FFTW_MEASURE);
  // Loop over all the symbols of the signal
  for (int symbol = 0; symbol < num_symbols; symbol++) {
    // Compute symbol index
    int symb_index = (first_symb_index + symbol) %
                     free5GRAN::NUMBER_SYMBOLS_PER_SLOT_NORMAL_CP;
    // Filling fft input signal with current symbol IQ
    // Perform cyclic prefix deletion by only extracting
    // the data part of a symbol
    // (take the fft_size elements of the symbol
    // shifted by the symbol CP - cp_lengths[symb_index] -)
    for (int i = 0; i < fft_size; i++) {
      fft_in[i][0] =
          real(time_domain_signal[i + offset + cum_sum_symb[symb_index] +
                                  cp_lengths[symb_index]]);
      fft_in[i][1] =
          imag(time_domain_signal[i + offset + cum_sum_symb[symb_index] +
                                  cp_lengths[symb_index]]);
    }
    // Execute the fft
    fftw_execute(fft_plan);
    // Recover RE grid from FFT output
    // In FFTW3, positive and negative
    // frequencies are twisted
    for (int i = 0; i < num_sc_output / 2; i++) {
      output_signal[symbol][num_sc_output / 2 + i] =
          complex<float>(fft_out[i][0], fft_out[i][1]);
      output_signal[symbol][num_sc_output / 2 - i - 1] = complex<float>(
          fft_out[fft_size - i - 1][0], fft_out[fft_size - i - 1][1]);
    }
  }
  fftw_destroy_plan(fft_plan);
  fftw_free(fft_in);
  fftw_free(fft_out);
}


auto free5GRAN::phy::signal_processing::synchronize_and_extract_pbch(
    vector<complex<float>> buffer,
    int& pss_start_index,
    float& received_power,
    free5GRAN::phy::bwp* bwp,
    int& pci,
    float& freq_offset,
    double sampling_rate,
    int& i_ssb,
    free5GRAN::ss_power_indicator& ss_pwr,
    free5GRAN::mib& mib_object,
    int l_max) -> int {
  /**
 * \fn extract_pbch
 * \brief Time resynchronization, frequency synchronization, PBCH extraction
 * and decoding. \details
 * - Getting 3ms signal from RF device
 * - PSS cross-correlation to retrieve N_ID_2
 * - SSS correlation to retrieve N_ID_1
 * - PCI computation based on N_ID_1 and N_ID_2
 * - Function ends if recomputed PCI differs from to initially computed one
 * - Fine frequency synchronization by correlating cyclic prefixes and
 * corresponding symbol part
 * - Signal extraction and FFT
 * - Resource element de-mapper
 * - Channel estimation based on different values of i_ssb
 * - Channel equalization based on best SNR value
 * - PBCH decoding
 * - BCH decoding
 * - MIB parsing
 */

  BOOST_LOG_TRIVIAL(trace) << "Extracting PBCH";
  // Get at least 30ms of signal (=3 frames, at least 2 complete ones)
  size_t num_samples = buffer.size();

  ofstream data;
  data.open("data.txt");
  for (int i = 0; i < num_samples; i++) {
    data << buffer[i];
    data << "\n";
  }
  data.close();

  auto now = chrono::high_resolution_clock::now();

  /*
   * Compute PBCH CP lengths for 1 subframe
   */
  /*
  int num_symbols_per_subframe_pbch =
      free5GRAN::NUMBER_SYMBOLS_PER_SLOT_NORMAL_CP * scs / 15e3;
  int cp_lengths_pbch[num_symbols_per_subframe_pbch];
  int cum_sum_pbch[num_symbols_per_subframe_pbch];
  free5GRAN::phy::signal_processing::compute_cp_lengths(
      (int)scs / 1e3, fft_size, 0, num_symbols_per_subframe_pbch,
      cp_lengths_pbch, cum_sum_pbch);
  common_cp_length = cp_lengths_pbch[1];
   */

  // Compute common symbol duration
  int symbol_duration = bwp->getFftSize() + bwp -> getCommonCpLength();

  int synchronisation_index;
  float peak_value;

  /*
   * Downsample signal for better performance
   */
  int downsampling_factor = bwp->getFftSize() / free5GRAN::PSS_SSS_FFT_SIZE;
  BOOST_LOG_TRIVIAL(trace) << "PSS synchronization downsampling factor: "
                           << downsampling_factor;
  int symbol_duration_downsampled = symbol_duration / downsampling_factor;
  vector<complex<float>> pss_signal_downsampled(num_samples /
                                                downsampling_factor);
  for (int i = 0; i < pss_signal_downsampled.size(); i++) {
    pss_signal_downsampled[i] = buffer[(size_t)downsampling_factor * i];
  }

  int n_id_1, n_id_2;
  // Search PSS on the downsampled signal
  free5GRAN::phy::synchronization::search_pss(
      n_id_2, synchronisation_index, peak_value,
      bwp -> getCommonCpLength() / downsampling_factor, pss_signal_downsampled,
      bwp -> getFftSize() / downsampling_factor);
  // Compute PSS start index in downsampled signal
  pss_start_index = downsampling_factor *
                    (synchronisation_index - symbol_duration_downsampled + 1);

  /*
   * Once synchronization is made on downsampled signal, it can be performed on
   * full signal for finer results
   */
  vector<complex<float>> fine_pss_signal(symbol_duration +
                                         (2 * downsampling_factor + 1));
  int count = 0;
  for (int i = pss_start_index - downsampling_factor;
       i < pss_start_index + symbol_duration + (downsampling_factor + 1); i++) {
    fine_pss_signal[count] = buffer[i];
    count++;
  }
  // Search PSS in initial signal
  free5GRAN::phy::synchronization::search_pss(
      n_id_2, synchronisation_index, peak_value, bwp -> getCommonCpLength(),
      fine_pss_signal, bwp -> getFftSize());
  // Compute PSS start index relatively to the initial signal
  pss_start_index = pss_start_index + (synchronisation_index - symbol_duration +
                                       1 - downsampling_factor);
  int buffer_pss_index = pss_start_index;
  // Compute SSS start index
  int sss_init_index = pss_start_index + 2 * symbol_duration + bwp -> getCommonCpLength();

  /*
   * Extracting the SSS signal based on sss_init_index and common_cp_length
   */
  vector<complex<float>> sss_signal(bwp -> getFftSize());
  for (int i = 0; i < bwp -> getFftSize(); i++) {
    sss_signal[i] = buffer[i + sss_init_index];
  }

  float peak_value_sss;
  /*
   * Get SSS correlation result
   */
  free5GRAN::phy::synchronization::get_sss(n_id_1, peak_value_sss,
                                           sss_signal, bwp -> getFftSize(), n_id_2);

  // Compute cell PCI
  pci = 3 * n_id_1 + n_id_2;

  /*
   * UE is now synchronized with the cell.
   * Trying to extract DMRS AND PBCH
   */

  // Extract SSB signal and compute received power;
  vector<complex<float>> ssb_signal(4 * symbol_duration);
  received_power = 0;
  for (int i = 0; i < free5GRAN::NUM_SYMBOLS_SSB * symbol_duration; i++) {
    ssb_signal[i] = buffer[i + buffer_pss_index];
    received_power += pow(abs(ssb_signal[i]), 2);
  }
  received_power /= 4 * symbol_duration;
  received_power = 10 * log10(received_power);

  /*
   * Fine frequency correlation
   * Getting phase offset between CP and corresponding part of the OFDM symbol
   * for each of the 4 symbols. phase_offset is the mean phase offset
   */
  free5GRAN::phy::signal_processing::compute_fine_frequency_offset(
      ssb_signal, symbol_duration, bwp -> getFftSize(), bwp -> getCommonCpLength(), bwp -> getScs(), freq_offset,
      free5GRAN::NUM_SYMBOLS_SSB);

  // Correcting signal based on frequency offset
  free5GRAN::phy::signal_processing::transpose_signal(
      &buffer, freq_offset, sampling_rate, buffer.size());

  /*
   * Extracting DMRS AND PBCH modulation samples
   * final_pbch_modulation_samples will contain the PBCH samples
   * after equalization
   * pbch_samples will contain temporary PBCH samples (before computing ssb
   * index) dmrs_samples will contain DMRS samples for each ssb index value
   * sss_samples will contain SSS samples after FFT
   */
  vector<complex<float>> final_pbch_modulation_samples(
      free5GRAN::SIZE_SSB_PBCH_SAMPLES);
  complex<float> pbch_samples[free5GRAN::SIZE_SSB_PBCH_SAMPLES];
  complex<float> dmrs_samples[free5GRAN::SIZE_SSB_DMRS_SAMPLES];
  complex<float> sss_samples[free5GRAN::SIZE_PSS_SSS_SIGNAL];
  /*
   * ref is the reference grid for resource element demapper
   * ref[0] -> indexes of PBCH resource elements
   * ref[1] -> indexes of DMRS resource elements
   */
  vector<vector<vector<int>>> ref(
      3, vector<vector<int>>(free5GRAN::SIZE_SSB_DMRS_SAMPLES,
                             vector<int>(free5GRAN::NUM_SC_SSB)));
  /*
   * channel_indexes[0] contains PBCH samples indexes
   * channel_indexes[1] contains DMRS samples indexes
   * channel_indexes[2] contains SSS samples indexes
   */
  vector<vector<vector<int>>> channel_indexes = {
      vector<vector<int>>(2, vector<int>(free5GRAN::SIZE_SSB_PBCH_SAMPLES)),
      vector<vector<int>>(2, vector<int>(free5GRAN::SIZE_SSB_DMRS_SAMPLES)),
      vector<vector<int>>(2, vector<int>(free5GRAN::SIZE_PSS_SSS_SIGNAL))};

  vector<vector<complex<float>>> ssb_samples(
      free5GRAN::NUM_SYMBOLS_SSB - 1,
      vector<complex<float>>(free5GRAN::NUM_SC_SSB));

  vector<int> cum_sum_fft(free5GRAN::NUM_SYMBOLS_SSB);
  for (int symbol = 0; symbol < free5GRAN::NUM_SYMBOLS_SSB; symbol++) {
    cum_sum_fft[symbol] = symbol * symbol_duration;
  }

  /*
   * Recover RE grid from time domain signal
   */
  bwp->fft(ssb_signal,ssb_samples,cum_sum_fft,  free5GRAN::NUM_SYMBOLS_SSB - 1, free5GRAN::NUM_SC_SSB,1,0);

  // Fill ref grid
  free5GRAN::phy::physical_channel::compute_pbch_indexes(ref, pci);
  /*
   * Channel demapping using computed ref grid
   */
  complex<float>* output_channels[] = {pbch_samples, dmrs_samples, sss_samples};
  free5GRAN::phy::signal_processing::channel_demapper(
      ssb_samples, ref, output_channels, channel_indexes, 3,
      free5GRAN::NUM_SYMBOL_PBCH_SSB, free5GRAN::NUM_SC_SSB);

  /*
   * Channel estimation and equalization
   * Creating coefficients arrays that will contain the channel coefficients
   * grid
   */
  vector<vector<vector<complex<float>>>> coefficients(
      free5GRAN::MAX_I_BAR_SSB,
      vector<vector<complex<float>>>(
          free5GRAN::NUM_SYMBOL_PBCH_SSB,
          vector<complex<float>>(free5GRAN::NUM_SC_SSB)));
  // dmrs_sequence will contain pilot values (computed by the UE)
  complex<float> dmrs_sequence[free5GRAN::SIZE_SSB_DMRS_SAMPLES];
  // snr contains the SNRs for all the possible SSB index values
  float snr[free5GRAN::MAX_I_BAR_SSB];

  /*
   * For each possible iBarSSB (SSB index) value, estimate the corresponding
   * transport_channel
   */
  for (int i = 0; i < free5GRAN::MAX_I_BAR_SSB; i++) {
    free5GRAN::utils::sequence_generator::generate_pbch_dmrs_sequence(
        pci, i, dmrs_sequence);
    free5GRAN::phy::signal_processing::channelEstimation(
        dmrs_samples, dmrs_sequence, channel_indexes[1], coefficients[i],
        snr[i], free5GRAN::NUM_SC_SSB, free5GRAN::NUM_SYMBOL_PBCH_SSB,
        free5GRAN::SIZE_SSB_DMRS_SAMPLES);
  }
  /*
   * Choose the iBarSSB value that maximizes the SNR
   */
  float max_snr = snr[0];
  int i_b_ssb = 0;
  for (int i = 1; i < free5GRAN::MAX_I_BAR_SSB; i++) {
    if (snr[i] > max_snr) {
      max_snr = snr[i];
      i_b_ssb = i;
    }
  }

  // Equalize physical channel
  for (int i = 0; i < free5GRAN::SIZE_SSB_PBCH_SAMPLES; i++) {
    final_pbch_modulation_samples[i] =
        (pbch_samples[i]) *
        conj(coefficients[i_b_ssb][channel_indexes[0][0][i]]
             [channel_indexes[0][1][i]]) /
        (float)pow(abs(coefficients[i_b_ssb][channel_indexes[0][0][i]]
                       [channel_indexes[0][1][i]]),
                   2);
  }

  // Compute SS-RSRP
  ss_pwr.ss_rsrp = 0;
  for (int i = 0; i < free5GRAN::SIZE_PSS_SSS_SIGNAL; i++) {
    ss_pwr.ss_rsrp += pow(abs(sss_samples[i]), 2);
  }
  for (int i = 0; i < free5GRAN::SIZE_SSB_DMRS_SAMPLES; i++) {
    ss_pwr.ss_rsrp += pow(abs(dmrs_samples[i]), 2);
  }
  ss_pwr.ss_rsrp /=
      (free5GRAN::SIZE_PSS_SSS_SIGNAL + free5GRAN::SIZE_SSB_DMRS_SAMPLES);

  // Compute SS-RSSI
  ss_pwr.ss_rssi = 0;
  for (int symb = 0; symb < free5GRAN::NUM_SYMBOLS_SSB - 1; symb++) {
    for (int sc = 0; sc < free5GRAN::NUM_SC_SSB; sc++) {
      ss_pwr.ss_rssi += pow(abs(ssb_samples[symb][sc]), 2);
    }
  }
  // 20 is the number of RB in SSB block
  int n_rb = 20;
  ss_pwr.ss_rssi /= n_rb;
  ss_pwr.ss_rsrq = 10 * log(n_rb * ss_pwr.ss_rsrp / ss_pwr.ss_rssi);
  // Converting RSRP and RSSI from W to dBm
  ss_pwr.ss_rsrp = 10 * log10(ss_pwr.ss_rsrp) + 30;
  ss_pwr.ss_rssi = 10 * log10(ss_pwr.ss_rssi) + 30;
  ss_pwr.ss_sinr = max_snr;

  if (l_max == 4) {
    i_ssb = i_b_ssb % 4;
  } else {
    i_ssb = i_b_ssb;
  }

  /*
   * Physical and transport channel decoding
   * MIB parsing
   */
  int bch_bits[free5GRAN::SIZE_SSB_PBCH_SAMPLES * 2];
  free5GRAN::phy::physical_channel::decode_pbch(final_pbch_modulation_samples,
                                                i_ssb, pci, bch_bits);
  int mib_bits[free5GRAN::BCH_PAYLOAD_SIZE];
  free5GRAN::phy::transport_channel::decode_bch(
      bch_bits, mib_object.crc_validated, mib_bits, pci);
  free5GRAN::utils::common_utils::parse_mib(mib_bits, mib_object);
  return 0;
}
void free5GRAN::phy::signal_processing::blind_search_pdcch(
    bool& validated,
    vector<complex<float>> frame_data,
    free5GRAN::phy::bwp* bwp,
    free5GRAN::coreset control_resource_set,
    int first_slot_index,
    int first_symbol_index,
    int& monitoring_slot,
    vector<int>& dci_decoded_bits,
    int& freq_domain_ra_size,
    int frame_size) {
  /**
   * \fn blind_search_pdcch
   * \brief PDCCH config extraction, PDCCH blind search and DCI decoding
   * \standard TS 38.213 13
   * \details
   * - Read PDCCH config from MIB
   * - Detect frame beginning
   * - Select frame containing PDCCH and PDSCH based of SFN
   * - Frequency calibration to retrieve center on CORESET0
   * - Compute CCE to REG mapping
   * - Blind search DCI decoding over different candidates:
   *  -# Select a candidate
   *  -# Perform resource element de-mapping and FFT
   *  -# Channel estimation & equalization
   *  -# PDCCH decoding
   *  -# DCI decoding
   *  -# If CRC is validated, candidate is selected and function ends
   *  -# Otherwise, function continues with another candidates
   *
   * \param[out] dci_found: returns true if blind decode succeeds.
   */

  /*
   * Initialize arrays
   */
  int num_sc_coreset_0 = 12 * control_resource_set.n_rb_coreset;

  /*
   * Compute CCE-to-REG Bundle mapping From TS38.211 7.3.2.2
   */
  int height_reg_rb =
      free5GRAN::NUMBER_REG_PER_CCE / control_resource_set.duration;
  // R: Interleaver depth: Number of section into which the CORESET is divided
  int R = control_resource_set.cce_REG_MappingType.interleaverSize;
  // C = N_RB_CORESET / (L * R)
  // with
  // L: REG Bundle size
  // N_RB_CORESET: Number of RB in Coreset
  int C = control_resource_set.n_rb_coreset / (height_reg_rb * R);
  int j;
  int reg_index[C * R];
  for (int c = 0; c < C; c++) {
    for (int r = 0; r < R; r++) {
      j = c * R + r;
      reg_index[j] =
          (r * C + c + control_resource_set.cce_REG_MappingType.shiftIndex) %
          (control_resource_set.n_rb_coreset / height_reg_rb);
    }
  }
  for (int i = 0; i < C * R; i++) {
    BOOST_LOG_TRIVIAL(trace)
        << "## CCE" + to_string(i) + ": REG" + to_string(reg_index[i]);
  }
  int K;
  /*
   * Number of bits for Frequency domain allocation in DCI TS 38 212 7.3.1.2.1
   */
  freq_domain_ra_size =
      ceil(log2(bwp->getNBwpSize() * (bwp->getNBwpSize() + 1) / 2));
  /*
   * K is the DCI payload size including CRC
   */
  K = freq_domain_ra_size + 4 + 1 + 5 + 2 + 1 + 15 + 24;

  float snr;
  validated = false;
  int agg_level, num_candidates;
  dci_decoded_bits.resize(K - 24);
  // dci_decoded_bits = new int[K - 24];
  //  global_sequence will contain the DMRS sequence for the whole coreset
  vector<vector<complex<float>>> global_sequence(
      control_resource_set.duration,
      vector<complex<float>>(control_resource_set.n_rb_coreset * 3));
  // ref will contain the position for PDCCH qnd DMRS
  vector<vector<vector<int>>> ref(
      2,
      vector<vector<int>>(control_resource_set.duration,
                          vector<int>(12 * control_resource_set.n_rb_coreset)));
  // coreset_0_samples will contain the RE grid for coreset 0
  vector<vector<complex<float>>> coreset_0_samples(
      control_resource_set.duration, vector<complex<float>>(num_sc_coreset_0));
  // coefficients will contain the channel coefficients for channel equalization
  vector<vector<complex<float>>> coefficients(
      control_resource_set.duration, vector<complex<float>>(num_sc_coreset_0));

  /*
   * PDCCH blind search.
   * The UE does not know, a priori, if there is PDCCH in the coreset and
   * where it is located. Blind search enables the UE to loop over all the
   * combinations to search for PDCCH. CRC enables the UE to validate a
   * candidate First, loop over every monitoring slot
   */
  BOOST_LOG_TRIVIAL(trace) << "### PDCCH BLIND SEARCH" << endl;
  for (monitoring_slot = 0; monitoring_slot < 2; monitoring_slot++) {
    // monitoring_slot = monitoring_slot;
    BOOST_LOG_TRIVIAL(trace)
        << "## MONITORING SLOT: " + to_string(monitoring_slot) << endl;
    // Symbol in slot where PDCCH could be located
    int symb_index =
        (first_symbol_index) % free5GRAN::NUMBER_SYMBOLS_PER_SLOT_NORMAL_CP;
    /*
     * Extract corresponding CORESET0 samples. CORESET0 number of symbols is
     * given by PDCCH config in MIB
     */
    BOOST_LOG_TRIVIAL(trace) << "PERFORMING FFT" << endl;
    BOOST_LOG_TRIVIAL(trace) << "FFT SIZE: " << bwp->getFftSize() << endl;
    BOOST_LOG_TRIVIAL(trace)
        << "NUM SLOTS PER FRAME: " << bwp->getNumSlotsPerFrame() << endl;
    // Recover RE grid from time domain signal
    bwp->fft(frame_data, coreset_0_samples, control_resource_set.duration,
             num_sc_coreset_0, first_symbol_index,
             (first_slot_index + monitoring_slot) * frame_size /
                 bwp->getNumSlotsPerFrame());
    for (int symb = 0; symb < control_resource_set.duration; symb++) {
      /*
       * Generate DMRS sequence for corresponding symbols
       */
      free5GRAN::utils::sequence_generator::generate_pdcch_dmrs_sequence(
          control_resource_set.pdcch_DMRS_ScrambilngID,
          first_slot_index + monitoring_slot, first_symbol_index + symb,
          global_sequence[symb], control_resource_set.n_rb_coreset * 3);
    }

    /*
     * Loop over possible aggregation level id (from 2 to 4 included)
     */
    for (int i = 2; i < 5; i++) {
      // comput aggregation level value
      agg_level = pow(2, i);
      // If agg level can fit with the CORESET size
      if (agg_level <= control_resource_set.n_rb_coreset / height_reg_rb) {
        // channel_indexes will contain PDCCH and DMRS
        // indexes for each sample
        // For PDCCH, for each 12 REs, there is 9 REs of PDCCH
        // and 3 REs of DMRS
        vector<vector<vector<int>>> channel_indexes = {
            vector<vector<int>>(2,
                                vector<int>((size_t)agg_level *
                                            free5GRAN::NUMBER_REG_PER_CCE * 9)),
            vector<vector<int>>(
                2, vector<int>((size_t)agg_level *
                               free5GRAN::NUMBER_REG_PER_CCE * 3))};
        // pdcch_samples will contain the PDCCH samples for the current
        // candidate
        vector<complex<float>> pdcch_samples((size_t)agg_level *
                                             free5GRAN::NUMBER_REG_PER_CCE * 9);
        // temp_pdcch_samples will contain temporary PDCCH samples
        // for channel de-mapper and channel estimation
        complex<float>
            temp_pdcch_samples[agg_level * free5GRAN::NUMBER_REG_PER_CCE * 9];
        // dmrs_samples will contain DMRS samples extracted from coreset
        complex<float>
            dmrs_samples[agg_level * free5GRAN::NUMBER_REG_PER_CCE * 3];
        // dmrs_sequence contains DMRS pilot value computed by UE
        complex<float>
            dmrs_sequence[agg_level * free5GRAN::NUMBER_REG_PER_CCE * 3];

        int reg_bundles[agg_level];
        int reg_bundles_ns[agg_level];
        int dci_bits[agg_level * free5GRAN::NUMBER_REG_PER_CCE * 9 * 2];

        BOOST_LOG_TRIVIAL(trace)
            << "## AGGREGATION LEVEL" + to_string(agg_level) << endl;
        // Compute number of candidates per aggregation level
        num_candidates =
            control_resource_set.n_rb_coreset / (agg_level * height_reg_rb);
        /*
         * Loop over candidates of current aggregation level
         */
        for (int p = 0; p < num_candidates; p++) {
          BOOST_LOG_TRIVIAL(trace) << "## CANDIDATE " + to_string(p) << endl;
          /*
           * Extract REG bundles for current candidate and aggregation level
           */
          for (int l = 0; l < agg_level; l++) {
            reg_bundles[l] = reg_index[l + p * agg_level];
            reg_bundles_ns[l] = reg_index[l + p * agg_level];
          }
          // Sort REG Bundles in increasing order
          sort(reg_bundles, reg_bundles + agg_level);
          /*
           * PDCCH samples extraction
           * Initialize ref to 0
           */
          for (int symbol = 0; symbol < control_resource_set.duration;
               symbol++) {
            for (int sc = 0; sc < 12 * control_resource_set.n_rb_coreset;
                 sc++) {
              ref[1][symbol][sc] = 0;
              ref[0][symbol][sc] = 0;
            }
          }
          /*
           * Computing PDCCH candidate position in RE grid
           */
          free5GRAN::phy::physical_channel::compute_pdcch_indexes(
              ref, control_resource_set.duration, agg_level, reg_bundles,
              height_reg_rb);
          /*
           * Channel de-mapping
           */
          complex<float>* output_channels[] = {temp_pdcch_samples,
                                               dmrs_samples};
          free5GRAN::phy::signal_processing::channel_demapper(
              coreset_0_samples, ref, output_channels, channel_indexes, 2,
              control_resource_set.duration,
              12 * control_resource_set.n_rb_coreset);
          /*
           * DMRS CCE-to-REG de-mapping for DMRS
           */
          for (int k = 0; k < agg_level; k++) {
            for (int reg = 0; reg < free5GRAN::NUMBER_REG_PER_CCE; reg++) {
              // Extract DMRS samples from global coreset DMRS sequence
              dmrs_sequence[((agg_level * free5GRAN::NUMBER_REG_PER_CCE * 3) /
                             control_resource_set.duration) *
                                (reg % control_resource_set.duration) +
                            k * height_reg_rb * 3 +
                            (reg / control_resource_set.duration) * 3] =
                  global_sequence[reg % control_resource_set.duration]
                                 [reg_bundles[k] * height_reg_rb * 3 +
                                  (reg / control_resource_set.duration) * 3];
              dmrs_sequence[((agg_level * free5GRAN::NUMBER_REG_PER_CCE * 3) /
                             control_resource_set.duration) *
                                (reg % control_resource_set.duration) +
                            k * height_reg_rb * 3 +
                            (reg / control_resource_set.duration) * 3 + 1] =
                  global_sequence[reg % control_resource_set.duration]
                                 [reg_bundles[k] * height_reg_rb * 3 +
                                  (reg / control_resource_set.duration) * 3 +
                                  1];
              dmrs_sequence[((agg_level * free5GRAN::NUMBER_REG_PER_CCE * 3) /
                             control_resource_set.duration) *
                                (reg % control_resource_set.duration) +
                            k * height_reg_rb * 3 +
                            (reg / control_resource_set.duration) * 3 + 2] =
                  global_sequence[reg % control_resource_set.duration]
                                 [reg_bundles[k] * height_reg_rb * 3 +
                                  (reg / control_resource_set.duration) * 3 +
                                  2];
            }
          }
          /*
           * Channel estimation
           */
          free5GRAN::phy::signal_processing::channelEstimation(
              dmrs_samples, dmrs_sequence, channel_indexes[1], coefficients,
              snr, 12 * control_resource_set.n_rb_coreset,
              control_resource_set.duration,
              agg_level * free5GRAN::NUMBER_REG_PER_CCE * 3);
          /*
           * Channel equalization
           */
          ofstream data_pdcch;
          data_pdcch.open("pdcch_constellation.txt");
          for (int sc = 0; sc < agg_level * free5GRAN::NUMBER_REG_PER_CCE * 9;
               sc++) {
            pdcch_samples[sc] =
                (temp_pdcch_samples[sc]) *
                conj(coefficients[channel_indexes[0][0][sc]]
                                 [channel_indexes[0][1][sc]]) /
                (float)pow(abs(coefficients[channel_indexes[0][0][sc]]
                                           [channel_indexes[0][1][sc]]),
                           2);
            data_pdcch << pdcch_samples[sc];
            data_pdcch << "\n";
          }
          data_pdcch.close();
          /*
          ofstream data;
          data.open("frame_buff_data.txt");
          for (int i = 0; i < frame_size; i++) {
            data << frame_data[i];
            data << "\n";
          }
          data.close();
           */
          /*
           * PDCCH and DCI decoding
           */
          free5GRAN::phy::physical_channel::decode_pdcch(
              pdcch_samples, dci_bits, agg_level, reg_bundles_ns, reg_bundles,
              control_resource_set.pdcch_DMRS_ScrambilngID);
          free5GRAN::phy::transport_channel::decode_dci(
              dci_bits, agg_level * free5GRAN::NUMBER_REG_PER_CCE * 9 * 2, K,
              free5GRAN::SI_RNTI, validated, dci_decoded_bits);

          /*
           * If DCI CRC is validated, candidate is validated, blind search ends
           */
          if (validated) {
            data_pdcch.open("output_files/pdcch_constellation.txt");
            for (int sc = 0; sc < agg_level * free5GRAN::NUMBER_REG_PER_CCE * 9;
                 sc++) {
              data_pdcch << pdcch_samples[sc];
              data_pdcch << "\n";
            }
            data_pdcch.close();
            goto dci_found_and_validated;
          }
        }
      } else {
        break;
      }
    }
  }

dci_found_and_validated:
  return;
}
void free5GRAN::phy::signal_processing::extract_pdsch(
    vector<complex<float>> frame_data,
    free5GRAN::phy::bwp* bwp,
    int slot_number,
    vector<int>& output_bits,
    free5GRAN::mib mib_object,
    bool& validated,
    dci_1_0_si_rnti dci_1_0_si_rnti_object,
    int frame_size,
    double sampling_rate,
    int pci) {
  /**
   * \fn extract_pdsch
   * \brief PDSCH extraction, PDSCH decoding, DL-SCH decoding and SIB1 parsing
   * \details
   * - Parameters extraction from DCI and standard
   * - Phase de-compensation. Looping over different possible phase
   * compensation:
   *  -# Signal extraction, FFT and resource element de-mapper
   *  -# Channel estimation & equalization
   *  -# PDSCH decoding
   *  -# DL-SCH decoding
   *  -# If CRC is validated, phase de-compensation is validated and functions
   * continues. Otherwise, another phase de-compensation is tried.
   * - SIB1 parsing using ASN1C
   */

  BOOST_LOG_TRIVIAL(trace) << "#### DECODING PDSCH";
  /*
   * Extracting PDSCH time and frequency position TS 38 214 5.1
   */
  int lrb, rb_start, k0, S, L, mod_order, code_rate, l0;
  free5GRAN::phy::signal_processing::compute_rb_start_lrb_dci(
      dci_1_0_si_rnti_object.RIV, bwp->getNBwpSize(), lrb, rb_start);
  k0 = free5GRAN::TS_38_214_TABLE_5_1_2_1_1_2[dci_1_0_si_rnti_object.TD_ra]
                                             [mib_object.dmrs_type_a_position -
                                              2][1];
  S = free5GRAN::TS_38_214_TABLE_5_1_2_1_1_2[dci_1_0_si_rnti_object.TD_ra]
                                            [mib_object.dmrs_type_a_position -
                                             2][2];
  L = free5GRAN::TS_38_214_TABLE_5_1_2_1_1_2[dci_1_0_si_rnti_object.TD_ra]
                                            [mib_object.dmrs_type_a_position -
                                             2][3];
  string mapping_type =
      ((free5GRAN::TS_38_214_TABLE_5_1_2_1_1_2[dci_1_0_si_rnti_object.TD_ra]
                                              [mib_object.dmrs_type_a_position -
                                               2][0] == 0)
           ? "A"
           : "B");
  // Compute modulation order and Coding rate
  mod_order =
      free5GRAN::TS_38_214_TABLE_5_1_3_1_1[dci_1_0_si_rnti_object.mcs][0];
  code_rate =
      free5GRAN::TS_38_214_TABLE_5_1_3_1_1[dci_1_0_si_rnti_object.mcs][1];
  BOOST_LOG_TRIVIAL(trace) << "## Frequency domain RA: RB Start " +
                                  to_string(rb_start) + " and LRB " +
                                  to_string(lrb);
  BOOST_LOG_TRIVIAL(trace) << "## Time domain RA: K0 " + to_string(k0) +
                                  ", S " + to_string(S) + " and L " +
                                  to_string(L) + " (mapping type " +
                                  mapping_type + ")";
  BOOST_LOG_TRIVIAL(trace) << "## MCS: Order " + to_string(mod_order) +
                                  " and code rate " + to_string(code_rate);
  BOOST_LOG_TRIVIAL(trace) << "## Slot number " + to_string(slot_number + k0);

  /*
   * Compute number of additionnal DMRS positions TS 38 214 5.1
   */
  int additionnal_position;
  if (mapping_type == "A") {
    additionnal_position = 2;
  } else {
    if (L == 2 || L == 4) {
      additionnal_position = 0;
    } else if (L == 7) {
      additionnal_position = 1;
    } else {
      additionnal_position = 0;
    }
  }

  int *dmrs_symbols, num_symbols_dmrs;
  /*
   * Get PDSCH DMRS symbols indexes
   */
  free5GRAN::phy::signal_processing::get_pdsch_dmrs_symbols(
      mapping_type, L + S, additionnal_position,
      mib_object.dmrs_type_a_position, &dmrs_symbols, num_symbols_dmrs);

  float snr;

  // dmrs_sequence will contain the pilot values computed by the UE
  complex<float> dmrs_sequence[6 * lrb * num_symbols_dmrs];

  int count_dmrs_symbol = 0;
  // Compute CP lengths for PDSCH
  int num_symbols_per_subframe_pdsch =
      free5GRAN::NUMBER_SYMBOLS_PER_SLOT_NORMAL_CP * mib_object.scs / 15;
  int cp_lengths_pdsch[num_symbols_per_subframe_pdsch];
  int cum_sum_pdsch[num_symbols_per_subframe_pdsch];

  // pdsch_ofdm_samples will contain PDSCH RE grid
  // pdsch_samples will contain PDSCH samples after channel de-mapper
  vector<vector<complex<float>>> pdsch_ofdm_samples(
      L, vector<complex<float>>(12 * bwp->getNBwpSize())),
      pdsch_samples(L, vector<complex<float>>(12 * lrb));
  // ref will contain reference grid for channel de-mapper
  vector<vector<vector<int>>> ref(
      2, vector<vector<int>>(L, vector<int>(12 * lrb)));
  // channel_indexes will contain channels samples indexes in RE grid
  vector<vector<vector<int>>> channel_indexes = {
      vector<vector<int>>(
          2, vector<int>((size_t)12 * lrb * (L - num_symbols_dmrs))),
      vector<vector<int>>(2, vector<int>((size_t)6 * lrb * num_symbols_dmrs))};
  // coefficients will contain channel coefficients for equalization
  vector<vector<complex<float>>> coefficients(L,
                                              vector<complex<float>>(12 * lrb));
  // temp_dmrs_sequence will contain temporary DMRS sequence
  // pdsch_samples_only will contain PDSCH samples before equalization
  // dmrs_samples_only will contain received DMRS samples
  complex<float> temp_dmrs_sequence[6 * bwp->getNBwpSize()],
      pdsch_samples_only[12 * lrb * (L - num_symbols_dmrs)],
      dmrs_samples_only[6 * lrb * num_symbols_dmrs];

  /*
   * Compute PDSCH CP lengths (same as PDCCH, as it is the same BWP)
   */
  free5GRAN::phy::signal_processing::compute_cp_lengths(
      mib_object.scs, bwp->getFftSize(), false, num_symbols_per_subframe_pdsch,
      cp_lengths_pdsch, cum_sum_pdsch);

  /*
   * Recover RE grid from time domain signal
   */
  /*
  free5GRAN::phy::signal_processing::fft(
      frame_data, pdsch_ofdm_samples, bwp->getFftSize(), cp_lengths_pdsch,
      cum_sum_pdsch, L, 12 * bwp->getNBwpSize(), S,
      (slot_number + k0) * frame_size / bwp->getNumSlotsPerFrame());*/

  bwp->fft(frame_data, pdsch_ofdm_samples, L, 12 * bwp->getNBwpSize(), S,
           (slot_number + k0) * frame_size / bwp->getNumSlotsPerFrame());

  bool dmrs_symbol_array[L];
  /*
   * PDSCH extraction
   */
  for (int symb = 0; symb < L; symb++) {
    bool dmrs_symbol = false;
    /*
     * Check if studied symbol is a DMRS
     */
    for (int j = 0; j < num_symbols_dmrs; j++) {
      if (symb + S == dmrs_symbols[j]) {
        dmrs_symbol = true;
        break;
      }
    }
    dmrs_symbol_array[symb] = dmrs_symbol;

    if (dmrs_symbol) {
      /*
       * Get DMRS sequence
       */
      free5GRAN::utils::sequence_generator::generate_pdsch_dmrs_sequence(
          free5GRAN::NUMBER_SYMBOLS_PER_SLOT_NORMAL_CP, slot_number + k0,
          symb + S, 0, pci, temp_dmrs_sequence, 6 * bwp->getNBwpSize());
      for (int i = 0; i < 6 * lrb; i++) {
        dmrs_sequence[count_dmrs_symbol * 6 * lrb + i] =
            temp_dmrs_sequence[rb_start * 6 + i];
      }
      count_dmrs_symbol += 1;
    }

    for (int i = 0; i < 12 * lrb; i++) {
      pdsch_samples[symb][i] = pdsch_ofdm_samples[symb][12 * rb_start + i];
    }
  }
  free5GRAN::phy::physical_channel::compute_pdsch_indexes(
      ref, dmrs_symbol_array, L, lrb);
  /*
   * Channel de-mapping
   */
  complex<float>* output_channels[] = {pdsch_samples_only, dmrs_samples_only};
  free5GRAN::phy::signal_processing::channel_demapper(
      pdsch_samples, ref, output_channels, channel_indexes, 2, L, 12 * lrb);
  float f0 = 0;
  /*
   * Phase decompensator. As phase compensation is not known a priori, we have
   * to loop over different possibles phase compensation for decoding
   */
  for (int phase_decomp_index = 0; phase_decomp_index < 50;
       phase_decomp_index++) {
    /*
     * Compute phase decomp value
     */
    f0 += (phase_decomp_index % 2) * pow(2, bwp->getMu()) * 1e3;
    float phase_offset = (phase_decomp_index % 2) ? f0 : -f0;
    BOOST_LOG_TRIVIAL(trace) << "PHASE DECOMP " << phase_offset;
    complex<float> phase_decomp[num_symbols_per_subframe_pdsch];
    /*
     * Compute phase decompensation value for each symbol in  a subframe
     */
    free5GRAN::phy::signal_processing::compute_phase_decomp(
        cp_lengths_pdsch, cum_sum_pdsch, sampling_rate, phase_offset,
        num_symbols_per_subframe_pdsch, phase_decomp);
    /*
     * Phase de-compensation
     */
    for (int samp = 0; samp < 12 * lrb * (L - num_symbols_dmrs); samp++) {
      pdsch_samples_only[samp] =
          pdsch_samples_only[samp] *
          phase_decomp[((slot_number + k0) %
                        (bwp->getNumSlotsPerFrame() / 10)) *
                           free5GRAN::NUMBER_SYMBOLS_PER_SLOT_NORMAL_CP +
                       S + channel_indexes[0][0][samp]];
    }
    for (int samp = 0; samp < 6 * lrb * num_symbols_dmrs; samp++) {
      dmrs_samples_only[samp] =
          dmrs_samples_only[samp] *
          phase_decomp[((slot_number + k0) %
                        (bwp->getNumSlotsPerFrame() / 10)) *
                           free5GRAN::NUMBER_SYMBOLS_PER_SLOT_NORMAL_CP +
                       S + channel_indexes[1][0][samp]];
    }
    /*
     * Channel estimation
     */
    free5GRAN::phy::signal_processing::channelEstimation(
        dmrs_samples_only, dmrs_sequence, channel_indexes[1], coefficients, snr,
        12 * lrb, L, 6 * lrb * num_symbols_dmrs);
    /*
     * Channel equalization
     */
    vector<complex<float>> pdsch_samples_vector((size_t)12 * lrb *
                                                (L - num_symbols_dmrs));
    for (int sc = 0; sc < 12 * lrb * (L - num_symbols_dmrs); sc++) {
      pdsch_samples_vector[sc] =
          (pdsch_samples_only[sc]) *
          conj(coefficients[channel_indexes[0][0][sc]]
                           [channel_indexes[0][1][sc]]) /
          (float)pow(abs(coefficients[channel_indexes[0][0][sc]]
                                     [channel_indexes[0][1][sc]]),
                     2);
    }

    ofstream data_pdsch;
    data_pdsch.open("output_files/pdsch_constellation.txt");
    for (int i = 0; i < 12 * lrb * (L - num_symbols_dmrs); i++) {
      data_pdsch << pdsch_samples_vector[i];
      data_pdsch << "\n";
    }
    data_pdsch.close();

    /*
     * PDSCH and DL-SCH decoding
     */
    double dl_sch_bits[2 * pdsch_samples_vector.size()];
    free5GRAN::phy::physical_channel::decode_pdsch(pdsch_samples_vector,
                                                   dl_sch_bits, pci);
    int n_re =
        free5GRAN::phy::signal_processing::compute_nre(L, num_symbols_dmrs);

    output_bits = free5GRAN::phy::transport_channel::decode_dl_sch(
        dl_sch_bits, n_re, (float)code_rate / (float)1024, lrb,
        2 * pdsch_samples_vector.size(), validated, dci_1_0_si_rnti_object);
    /*
     * If DL-SCH CRC is validated, Phase decompensation is validated
     */
    if (validated)
      break;
  }
}
