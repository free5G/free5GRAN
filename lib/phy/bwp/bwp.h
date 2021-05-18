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

#ifndef FREE5GRAN_BWP_H
#define FREE5GRAN_BWP_H

#include <fftw3.h>
#include <complex>
#include <vector>

using namespace std;
namespace free5GRAN::phy {
class bwp {
 private:
  int id, scs, n_bwp_size, mu, num_slots_per_frame, num_symbols_per_subframe,
      fft_size, common_cp_length;
  vector<int> cp_lengths, cum_sum_cp_lengths;
  fftw_complex *fft_in, *fft_out;
  fftw_plan fft_plan;

 public:
  bwp(int id, double scs, int size_rb, double sampling_rate);
  bwp(int id, double scs, double sampling_rate);
  [[nodiscard]] auto getId() const -> int;
  [[nodiscard]] auto getScs() const -> int;
  [[nodiscard]] auto getNBwpSize() const -> int;
  [[nodiscard]] auto getMu() const -> int;
  [[nodiscard]] auto getNumSlotsPerFrame() const -> int;
  [[nodiscard]] auto getNumSymbolsPerSubframe() const -> int;
  [[nodiscard]] auto getFftSize() const -> int;
  [[nodiscard]] auto getCpLengths() const -> const vector<int>&;
  [[nodiscard]] auto getCumSumCpLengths() const -> const vector<int>&;
  [[nodiscard]] auto getCommonCpLength() const -> int;
  void fft(vector<complex<float>> time_domain_signal,
           vector<vector<complex<float>>>& output_signal,
           vector<int>& cum_sum_symb,
           int num_symbols,
           int num_sc_output,
           int first_symb_index,
           int offset);
  void fft(vector<complex<float>> time_domain_signal,
           vector<vector<complex<float>>>& output_signal,
           int num_symbols,
           int num_sc_output,
           int first_symb_index,
           int offset);
  void setNBwpSize(int nBwpSize);
  ~bwp();
};
}  // namespace free5GRAN::phy

#endif  // FREE5GRAN_BWP_H
