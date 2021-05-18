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

#include "bwp.h"
#include <iostream>
#include "../../variables/common_variables/common_variables.h"
#include "../libphy/libphy.h"
using namespace std;

auto free5GRAN::phy::bwp::getId() const -> int {
  return id;
}
auto free5GRAN::phy::bwp::getScs() const -> int {
  return scs;
}
auto free5GRAN::phy::bwp::getNBwpSize() const -> int {
  return n_bwp_size;
}
auto free5GRAN::phy::bwp::getMu() const -> int {
  return mu;
}
auto free5GRAN::phy::bwp::getNumSlotsPerFrame() const -> int {
  return num_slots_per_frame;
}
auto free5GRAN::phy::bwp::getNumSymbolsPerSubframe() const -> int {
  return num_symbols_per_subframe;
}
auto free5GRAN::phy::bwp::getFftSize() const -> int {
  return fft_size;
}
auto free5GRAN::phy::bwp::getCpLengths() const -> const vector<int>& {
  return cp_lengths;
}
auto free5GRAN::phy::bwp::getCumSumCpLengths() const -> const vector<int>& {
  return cum_sum_cp_lengths;
}
auto free5GRAN::phy::bwp::getCommonCpLength() const -> int {
  return common_cp_length;
}

free5GRAN::phy::bwp::bwp(int id,
                         double scs,
                         int size_rb,
                         double sampling_rate) {
  this->id = id;
  this->scs = scs;
  this->n_bwp_size = size_rb;
  this->mu = (int)log2(scs / 15e3);
  this->num_slots_per_frame = (int)10 * scs / 15e3;
  this->num_symbols_per_subframe =
      free5GRAN::NUMBER_SYMBOLS_PER_SLOT_NORMAL_CP * (int)(scs / 15e3);

  this->cp_lengths.resize(this->num_symbols_per_subframe);
  this->cum_sum_cp_lengths.resize(this->num_symbols_per_subframe);

  this->fft_size = (int)sampling_rate / scs;

  free5GRAN::phy::signal_processing::compute_cp_lengths(
      (int)scs / 1e3, this->fft_size, false, this->num_symbols_per_subframe,
      this->cp_lengths, this->cum_sum_cp_lengths);

  this->common_cp_length = cp_lengths[1];

  this->fft_in =
      (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * this->fft_size);

  this->fft_out =
      (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * this->fft_size);

  if (this->scs == 15e3) {
    fft_plan = free5GRAN::FFT_PLAN_15_KHZ;
  } else if (this->scs == 30e3) {
    fft_plan = free5GRAN::FFT_PLAN_30_KHZ;
  } else {
    fft_plan = free5GRAN::FFT_PLAN_60_KHZ;
  }
}
free5GRAN::phy::bwp::bwp(int id, double scs, double sampling_rate) {
  this->id = id;
  this->scs = scs;
  this->mu = (int)log2(scs / 15e3);
  this->num_slots_per_frame = (int)10 * scs / 15e3;
  this->num_symbols_per_subframe =
      free5GRAN::NUMBER_SYMBOLS_PER_SLOT_NORMAL_CP * (int)(scs / 15e3);

  this->cp_lengths.resize(this->num_symbols_per_subframe);
  this->cum_sum_cp_lengths.resize(this->num_symbols_per_subframe);

  this->fft_size = (int)sampling_rate / scs;

  free5GRAN::phy::signal_processing::compute_cp_lengths(
      (int)scs / 1e3, this->fft_size, false, this->num_symbols_per_subframe,
      this->cp_lengths, this->cum_sum_cp_lengths);

  this->common_cp_length = cp_lengths[1];

  this->fft_in =
      (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * this->fft_size);

  this->fft_out =
      (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * this->fft_size);
  if (this->scs == 15e3) {
    fft_plan = free5GRAN::FFT_PLAN_15_KHZ;
  } else if (this->scs == 30e3) {
    fft_plan = free5GRAN::FFT_PLAN_30_KHZ;
  } else {
    fft_plan = free5GRAN::FFT_PLAN_60_KHZ;
  }
}
void free5GRAN::phy::bwp::fft(vector<complex<float>> time_domain_signal,
                              vector<vector<complex<float>>>& output_signal,
                              vector<int>& cum_sum_symb,
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
    fftw_execute_dft(fft_plan, this->fft_in, this->fft_out);
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
}

void free5GRAN::phy::bwp::fft(vector<complex<float>> time_domain_signal,
                              vector<vector<complex<float>>>& output_signal,
                              int num_symbols,
                              int num_sc_output,
                              int first_symb_index,
                              int offset) {
  this->fft(time_domain_signal, output_signal, this->cum_sum_cp_lengths,
            num_symbols, num_sc_output, first_symb_index, offset);
}

free5GRAN::phy::bwp::~bwp() {
  fftw_free(fft_in);
  fftw_free(fft_out);
}
void free5GRAN::phy::bwp::setNBwpSize(int nBwpSize) {
  n_bwp_size = nBwpSize;
}
