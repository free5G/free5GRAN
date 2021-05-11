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

#include <complex>
#include <vector>

#include "../../phy/bwp/bwp.h"
#include "../../variables/common_structures/common_structures.h"
using namespace std;

namespace free5GRAN::phy::signal_processing {

void channelEstimation(complex<float>* pilots,
                       complex<float>* reference_pilot,
                       vector<vector<int>>& pilot_indexes,
                       vector<vector<complex<float>>>& coefficients,
                       float& snr,
                       int num_sc,
                       int num_symbols,
                       int pilot_size);

void hard_demodulation(vector<complex<float>> signal,
                       int* bits,
                       int signal_length,
                       int modulation_scheme);

void soft_demodulation(vector<complex<float>> signal,
                       double* soft_bits,
                       int signal_length,
                       int modulation_scheme);

void compute_fine_frequency_offset(vector<complex<float>> input_signal,
                                   int symbol_duration,
                                   int fft_size,
                                   int cp_length,
                                   int scs,
                                   float& output,
                                   int num_symbols);

void transpose_signal(vector<complex<float>>* input_signal,
                      float freq_offset,
                      int sample_rate,
                      int input_length);

void channel_demapper(vector<vector<complex<float>>>& input_signal,
                      vector<vector<vector<int>>>& ref,
                      complex<float>** output_channels,
                      vector<vector<vector<int>>>& output_indexes,
                      int num_channels,
                      int num_symbols,
                      int num_sc);

auto compute_freq_from_gscn(int gscn) -> double;

auto compute_pdcch_t0_ss_monitoring_occasions(int pdcch_config,
                                              int pbch_scs,
                                              int common_scs,
                                              int i)
    -> free5GRAN::pdcch_t0ss_monitoring_occasions;

void compute_rb_start_lrb_dci(int RIV, int n_size_bwp, int& lrb, int& rb_start);

void get_pdsch_dmrs_symbols(const string& type,
                            int duration,
                            int additionnal_position,
                            int l0,
                            int** output,
                            int& size);

void compute_cp_lengths(int scs,
                        int nfft,
                        int is_extended_cp,
                        int num_symb_per_subframes,
                        int* cp_lengths,
                        int* cum_sum_cp_lengths);
void compute_cp_lengths(int scs,
                        int nfft,
                        int is_extended_cp,
                        int num_symb_per_subframes,
                        vector<int>& cp_lengths,
                        vector<int>& cum_sum_cp_lengths);

void compute_phase_decomp(int* cp_lengths,
                          int* cum_sum_symb,
                          float sampling_rate,
                          float f0,
                          int num_symb_per_subframes,
                          complex<float>* phase_decomp_factor);

auto compute_nre(int num_symb_pdsch, int num_dmrs_symb) -> int;

void fft(vector<complex<float>> time_domain_signal,
         vector<vector<complex<float>>>& output_signal,
         int fft_size,
         int* cp_lengths,
         int* cum_sum_symb,
         int num_symbols,
         int num_sc_output,
         int first_symb_index,
         int offset);

auto synchronize_and_extract_pbch(vector<complex<float>> buffer,
                                  int& pss_start_index,
                                  float& received_power,
                                  free5GRAN::phy::bwp* bwp,
                                  int& pci,
                                  float& freq_offset,
                                  double sampling_rate,
                                  int& i_ssb,
                                  free5GRAN::ss_power_indicator& ss_pwr,
                                  free5GRAN::mib& mib_object,
                                  int l_max) -> int;

void blind_search_pdcch(bool& validated,
                        vector<complex<float>> frame_data,
                        free5GRAN::phy::bwp* bwp,
                        free5GRAN::coreset control_resource_set,
                        int first_slot_index,
                        int first_symbol_index,
                        int& monitoring_slot,
                        vector<int>& dci_decoded_bits,
                        int& freq_domain_ra_size,
                        int frame_size);

void extract_pdsch(vector<complex<float>> frame_data,
                   free5GRAN::phy::bwp* bwp,
                   int slot_number,
                   vector<int>& output_bits,
                   free5GRAN::mib mib_object,
                   bool& validated,
                   dci_1_0_si_rnti dci_1_0_si_rnti_object,
                   int frame_size,
                   double sampling_rate,
                   int pci);

}  // namespace free5GRAN::phy::signal_processing