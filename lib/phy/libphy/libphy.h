/*
 * Copyright 2020 Telecom Paris

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

            void modulation(int *bits, int bit_sequence_length, int modulation_scheme, std::complex<float> *pbch_symbols);

            void build_reference_grid(int num_channels, int num_sc_ssb, int num_symbols_ssb, int pci, int ***ref); //Nbs d'entrée variables

            void channel_mapper(std::complex<float> **input_channels, int ***ref, std::complex<float> ** output_channels, int num_channels, int num_symbols, int num_sc);

            void increase_size_ssb(std::complex<float> ** input_channel, std::complex<float> ** output_channel, int num_symbols, int num_sc_input, int num_sc_output);

            void reverse_ssb(std::complex<float> ** input_ssb, std::complex<float> ** output_reversed_ssb, int num_symbols, int num_sc);

            void ifft(std::complex<float> ** in_freq_domain_channel, std::complex<float> ** out_time_domain_channel, int fft_size, int dividing_factor, int sc_number);

            void adding_cp(std::complex<float> ** input_channel, int num_symbols, int num_sc_in, int cp_lengths, std::complex<float> ** output_channel_with_cp);

            void generate_time_domain_ssb(std::complex<float> * pbch_symbols, int pci, int i_b_ssb, int dividing_factor, int ifft_size, std::complex<float> ** SSB_signal_time_domain);
        }
    }
}