/**
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

 * \author Télécom Paris, P5G Lab ; Benoit Oehmicen & Aymeric de Javel
 * \version 0.2
 * \date January 2021
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
    int bch_crc[24]; // erreur ?
    bool crc_validated;
    //int *pi_seq; TO BE DELETED
    int i_ssb;
    int i_b_ssb;
    int l_max;
    //std::vector<std::complex<float>> pbch_symbols;
    //std::vector<std::complex<float>> pbch_symbols;
    //std::vector<std::complex<float>> pbch_symbols2;

public:

    phy();

    void encode_bch(free5GRAN::mib, int* rate_matched_bch);

    static void convert_decimal_to_binary(int size, int decimal, int* table_output);

    // void encode_mib(free5GRAN::mib, int* mib_bits); TO BE DELETED
    // void bch_interleaving(int* mib_bits, int* mib_bits_interleaved); TO BE DELETED
    // void scrambling_bch(int v, int pci, int* mib_bits_interleaved, int* bch_payload); TO BE DELETED
    void adding_crc(int* bch_payload, int*bch_payload_with_crc);
    // void polar_encode_bch(int N, int* input_bits, int* output_encoded_bits); TO BE DELETED
    void rate_matching(int* polar_encoded_bch, int* rate_matched_bch);

    void encode_pbch(int gscn, int pci, int i_b_ssb, int* rate_matched_bch, int* encoded_pbch);
    void modulation(int* bits, int bit_sequence_length, int modulation_scheme, std::complex<float> *pbch_symbols);
    void generate_dmrs_of_pbch(int pci, int i_b_ssb, std::complex<float> *dmrs_symbols);
    int * convert_pci_into_nid2_and_nid1(int pci);

    void construct_reference_grid(int num_channels, int num_sc_ssb, int num_symbols_ssb, int pci, int ***ref);
    void channel_mapper(std::complex<float> **input_channels, int ***ref, std::complex<float> ** output_channels, int num_channels, int num_symbols, int num_sc);
    void increase_size_ssb(std::complex<float> ** input_channel, std::complex<float> ** output_channel, int num_symbols, int num_sc_input, int num_sc_output);
    void reverse_ssb(std::complex<float> ** input_ssb, std::complex<float> ** output_reversed_ssb, int num_symbols, int num_sc);
    void ifft(std::complex<float> ** in_freq_domain_channel, std::complex<float> ** out_time_domain_channel, int fft_size, int sc_number);
    void compute_cp_lengths(int scs, int nfft, int is_extended_cp, int num_symb_per_subframes, int *cp_lengths, int *cum_sum_cp_lengths);
    void adding_cp(std::complex<float> ** input_channel, int num_symbols, int num_sc_in, int cp_lengths, std::complex<float> ** output_channel_with_cp);

    static void display_table(int *table, int size, char* table_name);
    static void display_complex_float(std::complex<float>* vector_to_display, int vector_size, char* vector_name);
    static void display_vector(std::vector<std::complex<float>> vector_to_display, int vector_size, char* vector_name);
    static void display_complex_double(std::complex<double> *vector_to_display, int vector_size, char* vector_name);
    static void display_signal_float(std::complex<float> ** signal_to_display, int num_symbols, int num_sc, char* signal_name);

    void encode_bch(int * mib_bits, int pci, int N, int *rate_matched_bch);
    void encode_pbch_and_modulation(int * rate_matched_bch, int pci, int gscn, int i_b_ssb, std::complex<float> * pbch_symbols2);
    void generate_SSB_time(std::complex<float> * pbch_symbols2, int pci, int i_b_ssb, free5GRAN::mib mib_object, std::complex<float> ** SSB_signal_time_domain_CP);

    std::vector<std::complex<float>> AY_extract_pbch(std::complex<float> ** input_SSB, int pci);
    int * AY_decode_pbch(int pci, std::vector<std::complex<float>> pbch_symbols);
    void AY_decode_bch(int* bch_bits, int pci, int* mib_bits);
    void AY_decode_mib(int* mib_bits, free5GRAN::mib &mib_object);

};


#endif //FREE5GRAN_PHY_H
