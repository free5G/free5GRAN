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
#include "../../variables/common_structures/common_structures.h"
using namespace std;


namespace free5GRAN {
    namespace phy {
        namespace signal_processing {

            void channelEstimation(complex<float> * pilots, complex<float> * reference_pilot, vector<vector<int>> &pilot_indexes, vector<vector<complex<float>>> &coefficients, float &snr, int num_sc, int num_symbols, int pilot_size);

            void hard_demodulation(vector<complex<float>> signal, int * bits, int signal_length, int modulation_scheme);

            void soft_demodulation(vector<complex<float>> signal, double * soft_bits, int signal_length, int modulation_scheme);

            void compute_fine_frequency_offset(vector<complex<float>> input_signal, int symbol_duration, int fft_size, int cp_length, int scs, float &output, int num_symbols);

            void transpose_signal(vector<complex<float>> *input_signal, float freq_offset, int sample_rate, int input_length);

            void channel_demapper(vector<vector<complex<float>>> &input_signal, vector<vector<vector<int>>> &ref, complex<float> **output_channels, vector<vector<vector<int>>> &output_indexes, int num_channels, int num_symbols, int num_sc);

            double compute_freq_from_gscn(int gscn);

            free5GRAN::pdcch_t0ss_monitoring_occasions compute_pdcch_t0_ss_monitoring_occasions(int pdcch_config, int pbch_scs, int common_scs, int i);

            void compute_rb_start_lrb_dci(int RIV, int n_size_bwp, int &lrb, int &rb_start);

            void get_pdsch_dmrs_symbols(string type, int duration, int additionnal_position, int l0, int **output, int &size);

            void compute_cp_lengths(int scs, int nfft, int is_extended_cp, int num_symb_per_subframes, int *cp_lengths, int *cum_sum_cp_lengths);

            void compute_phase_decomp(int* cp_lengths, int* cum_sum_symb, float sampling_rate, float f0, int num_symb_per_subframes, complex<float>* phase_decomp_factor);

            int compute_nre(int num_symb_pdsch, int num_dmrs_symb);

            void fft(vector<complex<float>> time_domain_signal, vector<vector<complex<float>>> &output_signal, int fft_size, int *cp_lengths, int *cum_sum_symb, int num_symbols, int num_sc_output, int first_symb_index, int offset);

            void get_candidates_frames_indexes(vector<vector<int>> &frame_indexes, int *frame_numbers, int sfn, int index_first_pss, int num_samples_before_pss, int frame_size);




            /** FROM HERE, IT'S ADDITION FROM BENOIT. BE CAREFUL WHEN MERGING */

            // To be deleted void modulation(int *bits, int bit_sequence_length, int modulation_scheme, std::complex<float> *pbch_symbols);
            void modulation(vector<int> bits, int bit_sequence_length, int modulation_scheme, vector<complex<float>> &pbch_symbols_vector);
            void build_reference_grid(int num_channels, int num_sc_ssb, int num_symbols_ssb, int pci, vector<vector<vector<int>>> &ref);
            // To be deleted void map_ssb(std::complex<float> **input_channels, int ***ref, vector<vector<complex<float>>> &output_channels, int num_channels, int num_symbols, int num_sc);
            // To be deleted void map_ssb(std::complex<float> **input_channels, vector<vector<vector<int>>> ref, vector<vector<complex<float>>> &output_channels, int num_channels, int num_symbols, int num_sc);
            void map_ssb(vector<vector<complex<float>>> input_channels, vector<vector<vector<int>>> ref, vector<vector<complex<float>>> &output_channels, int num_channels, int num_symbols, int num_sc);
            void channel_mapper(vector<vector<complex<float>>> input_channel, vector<vector<complex<float>>> &output_channel, int num_symbols_ssb, int index_symbol_ssb, int num_SSB_in_this_frame, int num_sc_input, int ifft_size);
            void generate_freq_domain_frame(vector<complex<float>> pbch_symbols_vector, int pci, int index_symbol_ssb, int num_SSB_in_this_frame, int i_b_ssb, vector<vector<complex<float>>> &freq_domain_frame);
            void ifft(vector<vector<complex<float>>> freq_domain_frame, int *cp_lengths_one_frame, vector<int> data_symbols, int num_symbols_frame, float scaling_factor, vector<complex<float>> &one_frame_vector);
            }
        }
}