/**
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

 * \author Télécom Paris, P5G Lab ; Benoit Oehmicen & Aymeric de Javel
 * \version 0.2
 * \date February 2021
 */

#ifndef FREE5GRAN_PHY_H
#define FREE5GRAN_PHY_H

#include "../../lib/variables/common_structures/common_structures.h"
#include "../../lib/phy/transport_channel/transport_channel.h"
#include "../../lib/variables/common_variables/common_variables.h"
#include "../rf/rf.h"
#include_next <math.h>
#include <fftw3.h>
#include <vector>
#include <complex>
#include <fstream>

class phy {

private:
    free5GRAN::mib mib_object{};
    int *cp_lengths_one_frame;
    int *cum_sum_cp_lengths;
    int ifft_size;
    int num_samples_in_frame;
    std::vector<std::complex<float>> buffer_null;
    std::vector<std::complex<float>> buffer_generated1_private;
    std::vector<std::complex<float>> buffer_generated2_private;




public:
    phy(free5GRAN::mib mib_object, int *cp_lengths_one_frame, int *cum_sum_cp_lengths, int ifft_size, int num_samples_in_frame);
    void generate_frame(int num_SSB_in_this_frame, int num_symbols_frame, int sfn,int pci, int i_b_ssb, float scaling_factor, std::vector<std::complex<float>> &one_frame);
    void compute_num_SSB_in_frame(float ssb_period, int sfn, int &num_SSB_in_frame);
    void continuous_buffer_generation();

    //----------- From here, DCI / PDCCH

    void UE_decode_polar_dci(vector<complex<float>> pdcch_symbols, int K, int N, int E, int length_crc, int pci,int agg_level, int polar_decoded_size, int freq_domain_ra_size, int *rnti, bool &validated, free5GRAN::dci_1_0_si_rnti &dci_object);
    void UE_decode_coreset(vector<vector<complex<float>>> masked_coreset_grid, int K, int N, int E, int length_crc, int pci, int agg_level, int polar_decoded_size, int freq_domain_ra_size, int *rnti, bool &validated, int slot_number, int symbol_number, int num_rb_coreset, free5GRAN::dci_1_0_si_rnti &dci_object);
};
void send_buffer_multithread(rf rf_variable_2, vector<complex<float>> * buff_generated1, vector<complex<float>> * buff_generated2);

#endif //FREE5GRAN_PHY_H
