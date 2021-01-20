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

#include "phy.h"
#include <iostream>
#include "../../lib/utils/sequence_generator/sequence_generator.h"
#include "../../lib/phy/libphy/libphy.h"
#include <fftw3.h>
#include <complex>
#include <vector>
#include "../../lib/utils/sequence_generator/sequence_generator.h"
#include "../../lib/variables/common_variables/common_variables.h"
#include "../../lib/utils/common_utils/common_utils.h"
#include "../../lib/phy/libphy/libphy.h"
#include "../../lib/phy/transport_channel/transport_channel.h"
#include_next <math.h>


bool display_variable = true; /** indicates if you want to display all variables in the Console */

phy::phy() {

}


/** FROM HERE, THE CODE AIMS TO DECODE INFORMATIONS AND NOT TO ENCODE IT ANYMORE (AYMERIC's CODE) */
/** IT IS ONLY HERE TO VERIFY THAT WHAT WE ENCODE IS POSSIBLE TO BE DECODED CORRECTLY */


std::vector<std::complex<float>> phy::AY_extract_pbch(std::complex<float> ** input_SSB, int pci){


    int fft_size = 256;
    int cp_length = 18;
    int symbol_duration = fft_size + cp_length; // = 274
    float max_snr;
    int l_max = 4;  //TO BE VERIFIED !!


    std::vector<std::complex<float>> pbch_modulation_symbols(free5GRAN::SIZE_SSB_PBCH_SYMBOLS);
    std::vector<std::complex<float>> final_pbch_modulation_symbols(free5GRAN::SIZE_SSB_PBCH_SYMBOLS);

    // Initializing fft parameters
    fftw_complex *fft_in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * fft_size);
    fftw_complex *fft_out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * fft_size);

    fftw_plan fft_plan = fftw_plan_dft_1d(fft_size, fft_in, fft_out, FFTW_FORWARD, FFTW_MEASURE);


    //Extracting DMRS AND PBCH modulation symbols


    std::vector<std::complex<float>> temp_mod_symbols, temp_mod_symbols2,temp_mod_symbols_dmrs;
    std::complex<float> *pbch_symbols_ay, *dmrs_symbols;
    pbch_symbols_ay = new std::complex<float>[free5GRAN::SIZE_SSB_PBCH_SYMBOLS];
    dmrs_symbols = new std::complex<float>[free5GRAN::SIZE_SSB_DMRS_SYMBOLS];

    //ref[0] -> indexes of PBCH resource elements
    //ref[1] -> indexes of DMRS resource elements


    int ** dmrs_indexes, ** pbch_indexes, ***ref;
    ref = new int **[2];
    ref[0] = new int *[free5GRAN::NUM_SYMBOL_PBCH_SSB]; // = [3]
    ref[1] = new int *[free5GRAN::NUM_SYMBOL_PBCH_SSB]; // = [3]
    dmrs_indexes = new int *[2];
    dmrs_indexes[0] = new int[free5GRAN::SIZE_SSB_DMRS_SYMBOLS]; // = [144]
    dmrs_indexes[1] = new int[free5GRAN::SIZE_SSB_DMRS_SYMBOLS]; // = [144]
    pbch_indexes = new int *[2];
    pbch_indexes[0] = new int[free5GRAN::SIZE_SSB_PBCH_SYMBOLS]; // = [432]
    pbch_indexes[1] = new int[free5GRAN::SIZE_SSB_PBCH_SYMBOLS]; // = [432]


    //Converting input_SSB (double) into vector_input_SSB (float)
    std::vector<std::complex<float>> vector_input_SSB(4 * symbol_duration);
    for (int symbol = 0; symbol < 4; symbol++){
        for (int sc = 0; sc < symbol_duration; sc++){
            vector_input_SSB[symbol * symbol_duration + sc] = input_SSB[symbol][sc];
        }
    }

    std::complex<float> **ssb_symbols = new std::complex<float> *;
    *ssb_symbols = new std::complex<float> [free5GRAN::NUM_SYMBOLS_SSB - 1]; // = [3]
    //Looping over 3 OFDM symbols containing DMRS and PBCH

    /** Below, some modifications to pass from table to vector */

    vector<vector<complex<float>>> ssb_symbols_vector(free5GRAN::NUM_SYMBOLS_SSB - 1, vector<complex<float>>(free5GRAN::NUM_SC_SSB));

    vector<vector<vector<int>>> ref_vector(2, vector<vector<int>>(free5GRAN::SIZE_SSB_DMRS_SYMBOLS, vector<int>(free5GRAN::NUM_SC_SSB)));

    int agg_level;

    vector<vector<vector<int>>> channel_indexes = {vector<vector<int>>(2, vector<int>(agg_level * free5GRAN::NUMBER_REG_PER_CCE * 9)), vector<vector<int>>(2, vector<int>(agg_level * free5GRAN::NUMBER_REG_PER_CCE * 3))};

    int lrb, L;

    vector<vector<complex<float>>> coefficients_vector(L, vector<complex<float>>(12 * lrb));

    /** ################################# */


    for (int symbol = 1; symbol < free5GRAN::NUM_SYMBOLS_SSB; symbol ++){ // loop symbol = 1, symbol = 2 and symbol = 3
        ssb_symbols[symbol-1] = new std::complex<float>[free5GRAN::NUM_SC_SSB]; // = [240]
        ref[0][symbol - 1] = new int [free5GRAN::NUM_SC_SSB]; // = [240]
        ref[1][symbol - 1] = new int [free5GRAN::NUM_SC_SSB]; // = [240]
        // Filling fft input signal with current symbol IQ
        for (int i = 0; i < fft_size; i++){
            fft_in[i][0] = std::real(vector_input_SSB[i + symbol * symbol_duration + cp_length]);
            fft_in[i][1] = std::imag(vector_input_SSB[i + symbol * symbol_duration + cp_length]);
        }
        // Execute the fft
        fftw_execute(fft_plan);
        // Building the RE grid from 0 to 239 from the 256 sized fft output
        for (int i = 0; i < free5GRAN::NUM_SC_SSB / 2; i++){ // loop from i = 0 to 119
            ssb_symbols[symbol-1][free5GRAN::NUM_SC_SSB / 2 + i ] = std::complex<float>(fft_out[i][0],fft_out[i][1]);
            ssb_symbols[symbol-1][free5GRAN::NUM_SC_SSB / 2 - i - 1] = std::complex<float>(fft_out[fft_size - i - 1][0],fft_out[fft_size - i - 1][1]);
        }
        // Creating Resource element de-mapper reference grid
        for (int i = 0; i < free5GRAN::NUM_SC_SSB; i++){
            if (symbol == 1 || symbol == 3){
                if (pci % 4 != i % 4){
                    ref[0][symbol - 1][i] = 1;
                    ref[1][symbol - 1][i] = 0;
                }else{
                    ref[0][symbol - 1][i] = 0;
                    ref[1][symbol - 1][i] = 1;
                }
            }else if (symbol == 2){
                if (pci % 4 != i % 4 && (i < free5GRAN::INTERVAL_SSB_NO_PBCH_DMRS[0] || i > free5GRAN::INTERVAL_SSB_NO_PBCH_DMRS[1])){
                    ref[0][symbol - 1][i] = 1;
                    ref[1][symbol - 1][i] = 0;
                } else if (i < free5GRAN::INTERVAL_SSB_NO_PBCH_DMRS[0] || i > free5GRAN::INTERVAL_SSB_NO_PBCH_DMRS[1]){
                    ref[0][symbol - 1][i] = 0;
                    ref[1][symbol - 1][i] = 1;
                }else{
                    ref[0][symbol - 1][i] = 0;
                    ref[1][symbol - 1][i] = 0;
                }
            }
        }
    }


    //free5GRAN::phy::signal_processing::channel_demapper(ssb_symbols, ref, new int[2]{free5GRAN::SIZE_SSB_PBCH_SYMBOLS,free5GRAN::SIZE_SSB_DMRS_SYMBOLS}, new std::complex<float>*[2] {pbch_symbols_ay,dmrs_symbols}, new int **[2] {pbch_indexes,dmrs_indexes}, 2, free5GRAN::NUM_SYMBOL_PBCH_SSB, free5GRAN::NUM_SC_SSB);
    free5GRAN::phy::signal_processing::channel_demapper(ssb_symbols_vector, ref_vector,new std::complex<float>*[2] {pbch_symbols_ay,dmrs_symbols}, channel_indexes, 2, free5GRAN::NUM_SYMBOL_PBCH_SSB, free5GRAN::NUM_SC_SSB);

    //Channel estimation and equalization
    //Creating coefficients arrays


    std::complex<float> **coefficients[free5GRAN::MAX_I_BAR_SSB];
    for (int p = 0; p < free5GRAN::MAX_I_BAR_SSB; p ++){
        coefficients[p] = new std::complex<float> * [free5GRAN::NUM_SYMBOL_PBCH_SSB];
        for (int i = 0 ; i < free5GRAN::NUM_SYMBOL_PBCH_SSB; i ++){
            coefficients[p][i] = new std::complex<float> [free5GRAN::NUM_SC_SSB];
        }
    }

    std::complex<float> * dmrs_sequence;
    float snr[free5GRAN::MAX_I_BAR_SSB]; // = [8]


//For each possible iBarSSB value, estimate the corresponding channel


    for (int i = 0; i < free5GRAN::MAX_I_BAR_SSB; i ++){
        dmrs_sequence = new std::complex<float>[free5GRAN::SIZE_SSB_DMRS_SYMBOLS];
        free5GRAN::utils::sequence_generator::generate_pbch_dmrs_sequence(pci,i,dmrs_sequence);
        free5GRAN::phy::signal_processing::channelEstimation(dmrs_symbols, dmrs_sequence, channel_indexes[1],coefficients_vector, snr[i], free5GRAN::NUM_SC_SSB, free5GRAN::NUM_SYMBOL_PBCH_SSB , free5GRAN::SIZE_SSB_DMRS_SYMBOLS);
    }

//Choose the iBarSSB value that maximizes the SNR


    max_snr = snr[0];
    int i_b_ssb = 0;
    for (int i = 1; i < free5GRAN::MAX_I_BAR_SSB ; i ++){
        if (snr[i] > max_snr){
            max_snr = snr[i];
            i_b_ssb = i;
        }
    }

// Equalize the channel
    std::vector<std::complex<float>> pbch_symbols = {};

    for (int i = 0; i < free5GRAN::SIZE_SSB_PBCH_SYMBOLS; i ++){ // = [432]

//pbch_symbols.push_back({8,7});
//pbch_symbols[i] = {8,7};
        pbch_symbols.push_back(  (pbch_symbols_ay[i]) * std::conj(coefficients[i_b_ssb][pbch_indexes[0][i]][pbch_indexes[1][i]]) / (float) pow(std::abs(coefficients[i_b_ssb][pbch_indexes[0][i]][pbch_indexes[1][i]]),2)   );
    }
//pbch_symbols.push_back({3,3});
/*
*/
    return pbch_symbols;


    /*
    this->i_b_ssb = i_b_ssb;
    //this-> i_ssb = i_b_ssb%4 ;
    if (l_max == 4){
        this-> i_ssb = i_b_ssb % 4;
    }else {
        this-> i_ssb = i_b_ssb;
    }
    this->pci = pci;

    /*
     * Setting pbch symbols to pbch object and running decoding process

    this->pbch_symbols_ay = final_pbch_modulation_symbols;
    //return 0;
     */
}





int * phy::AY_decode_pbch(int pci, std::vector<std::complex<float>> pbch_symbols) {
    /*
    std::ofstream out_file, out_bits, out_scrambled;
    out_file.open("out_pbch.txt");
    out_bits.open("out_bits.txt");
    out_scrambled.open("out_scrambled.txt");

    for (auto modulation_symbol : pbch_symbols2){
        out_file << modulation_symbol;
        out_file << "\n";
    }
    out_file.close();
    */
    int * bch_bits5 = new int[free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2];
    int * pbch_bits;
    pbch_bits = new int[free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2];
    /*
     * Demodulate PBCH Signal
     */
    free5GRAN::phy::signal_processing::hard_demodulation(pbch_symbols, pbch_bits, free5GRAN::SIZE_SSB_PBCH_SYMBOLS, 1);

    if (display_variable){
        free5GRAN::utils::common_utils::display_table(pbch_bits, free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2, "AY_pbch_bits");}

    // Generate de-scrambling sequence
    //c_sequence seq_object = c_sequence(pci, free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2 * (1 + i_b_ssb) );
    //int * c_seq = seq_object.get_sequence();
    int * c_seq = new int[free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2 * (1 + i_ssb)];
    free5GRAN::utils::sequence_generator::generate_c_sequence(pci, free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2 * (1 + i_ssb), c_seq, 0);

    //int * scrambled_bits;
    //scrambled_bits = new int[free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2];

    /*
     * De-scramble pbch_bits to scrambled_bits
     */
    free5GRAN::utils::common_utils::scramble(pbch_bits, c_seq, bch_bits5, free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2, i_ssb * free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2);
    if (display_variable){
        free5GRAN::utils::common_utils::display_table(bch_bits5, 864, "AY_bch_bits5 from phy");}
    return bch_bits5;
    /*
    for (int i = 0; i < free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2; i ++){
        out_bits << pbch_bits[i];
        out_bits << "\n";
        out_scrambled << bch_bits[i];
        out_scrambled << "\n";
    }
    out_bits.close();
    out_scrambled.close();
*/
}





void phy::AY_decode_bch(int* bch_bits, int pci, int* mib_bits) {

    char* table_name8 = "AY_bch_bits";
    if (display_variable){
        free5GRAN::utils::common_utils::display_table(bch_bits, 864, table_name8);}

    int n = free5GRAN::phy::transport_channel::compute_N_polar_code(free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2,free5GRAN::SIZE_PBCH_POLAR_DECODED,9);
    int N = pow(2,n);
    if (display_variable){std::cout << "N_AY =" << N << std::endl;}
    int * rate_recovered_bits, *polar_decoded_bits, *remainder, *bch_payload, *bch_crc_recomputed, *bch_crc, *crc_masq;
    rate_recovered_bits = new int[N];
    polar_decoded_bits = new int[free5GRAN::SIZE_PBCH_POLAR_DECODED];
    remainder = new int[free5GRAN::BCH_CRC_LENGTH + 1];
    bch_payload = new int[free5GRAN::BCH_PAYLOAD_SIZE];
    bch_crc_recomputed = new int[free5GRAN::BCH_CRC_LENGTH];
    bch_crc = new int[free5GRAN::BCH_CRC_LENGTH];
    crc_masq = new int[free5GRAN::BCH_CRC_LENGTH];
    // Rate recover bch_bits to rate_recovered_bits
    free5GRAN::phy::transport_channel::rate_recover(bch_bits,rate_recovered_bits,0,free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2,N, 0); /** 0 corresponds to 'K' but was choosen randomly */
    char* table_name9 = "AY_rate_recovered_bits";
    if (display_variable){
        free5GRAN::utils::common_utils::display_table(rate_recovered_bits, 512, table_name9);}

    // Polar decode rate_recovered_bits to polar_decoded_bits
    free5GRAN::phy::transport_channel::polar_decode(rate_recovered_bits,polar_decoded_bits,N,free5GRAN::SIZE_PBCH_POLAR_DECODED,9,1,0,0, 0); /** The last 0 corresponds to 'E' but is choosen randomly */

    // Validate polar_decoded_bits CRC (compute the remainder and check that t is equal to 0)
    free5GRAN::phy::transport_channel::crc_validate(polar_decoded_bits, free5GRAN::G_CRC_24_C,remainder,free5GRAN::SIZE_PBCH_POLAR_DECODED,free5GRAN::BCH_CRC_LENGTH + 1);
    crc_validated = true;
    for (int i = 1; i < free5GRAN::BCH_CRC_LENGTH + 1; i ++){
        if (remainder[i] ==1){
            crc_validated = false;
            std::cout << "CRC IS NOT VALIDATED" << std::endl;
            break;
        }
    }
    // Split polar_decoded_bits into bch_payload and bch_crc
    for (int i = 0; i < free5GRAN::BCH_PAYLOAD_SIZE; i ++){
        bch_payload[i] = polar_decoded_bits[i];
    }
    for (int i = 0; i < free5GRAN::BCH_CRC_LENGTH; i ++){
        bch_crc[i] = polar_decoded_bits[free5GRAN::BCH_PAYLOAD_SIZE + i];
    }

    if (display_variable) {
        char *table_name11 = "AY_bch_payload";
        free5GRAN::utils::common_utils::display_table(bch_payload, 32, table_name11);
    }

    // Re-compute the bch_payload CRC
    free5GRAN::phy::transport_channel::compute_crc(bch_payload, free5GRAN::G_CRC_24_C, bch_crc_recomputed, 32, 25);


    // XOR recomputed CRC with received CRC to determine CRC masq
    for (int i = 0; i < free5GRAN::BCH_CRC_LENGTH; i ++){
        crc_masq[i] = bch_crc[i] ^ bch_crc_recomputed[i];
    }

    /*
     * PBCH payload recovering (TS38.212 7.1.1)
     */
    int A = free5GRAN::BCH_PAYLOAD_SIZE;
    int A_bar = free5GRAN::BCH_PAYLOAD_SIZE - 8;
    int M = A - 3;
    int * s_sequence, *c_seq, *bch_descrambled;
    int sfn_bits[4][2] = {
            {0,0},
            {0,1},
            {1,0},
            {1,1}
    };
    // Find the correct value of v
    for (int v = 0 ; v < 4; v ++){
        // Generate de-scrambling sequence
        c_seq = new int[free5GRAN::BCH_PAYLOAD_SIZE + v * M];
        free5GRAN::utils::sequence_generator::generate_c_sequence(pci, free5GRAN::BCH_PAYLOAD_SIZE + v * M, c_seq, 0);

        s_sequence = new int[A];
        int j = 0;
        // Generate s sequence
        for (int i = 0; i < A; i ++){
            if ( i == 0 || i == 6 || i == 24){
                s_sequence[i] = 0;
            }else {
                s_sequence[i] = c_seq[j + v * M];
                j ++;
            }
        }

        bch_descrambled = new int[free5GRAN::BCH_PAYLOAD_SIZE];
        // De-scramble bch_payload to bch_descrambled
        free5GRAN::utils::common_utils::scramble(bch_payload, s_sequence, bch_descrambled, free5GRAN::BCH_PAYLOAD_SIZE, 0);

        // BCH payload de-interleaving

        //mib_bits = new int[free5GRAN::BCH_PAYLOAD_SIZE];
        int j_sfn = 0;
        int j_hrf = 10;
        int j_ssb = 11;
        int j_other = 14;
        for (int i = 0; i < 32; i ++){
            if (i == 24 || i == 25 || i == 26 || i ==27|| i==1||i==2||i==3||i==4||i==5||i==6){
                mib_bits[i] = bch_descrambled[free5GRAN::PBCH_PAYLOAD_INTERLEAVER[j_sfn]];
                j_sfn ++;
            }else if (i == 28){
                mib_bits[i] = bch_descrambled[free5GRAN::PBCH_PAYLOAD_INTERLEAVER[j_hrf]];
            }else if ( i >= A_bar + 5 && i <= A_bar + 7){
                mib_bits[i] = bch_descrambled[free5GRAN::PBCH_PAYLOAD_INTERLEAVER[j_ssb]];
                j_ssb ++;
            }else {
                mib_bits[i] = bch_descrambled[free5GRAN::PBCH_PAYLOAD_INTERLEAVER[j_other]];
                j_other ++;
            }
        }


        // If final bits correspond to 3rd and 2nd LSB of SFN, correct v was found
        if (sfn_bits[v][0] == mib_bits[25] && sfn_bits[v][1] == mib_bits[26]){
            //if(v==3){

            break;
        }

    }



}

void phy::AY_decode_mib(int* mib_bits, free5GRAN::mib &mib_object) {
    /**
     * Getting MIB informations
    */
    // Recovering SFN from BCH payload
    int sfn_array[10] = {mib_bits[1], mib_bits[2], mib_bits[3], mib_bits[4], mib_bits[5], mib_bits[6],mib_bits[24], mib_bits[25], mib_bits[26], mib_bits[27]};
    mib_object.sfn = 0;
    for (int i = 0 ; i < 10; i ++){
        mib_object.sfn += sfn_array[i] * pow(2, 10 - 1 - i);
    }

    int available_scs[2] = {15, 30};
    // Recovering PDCCH config identifier
    int pdcch_config_array[8];
    for (int i = 0 ; i < 8; i ++){
        pdcch_config_array[i] = mib_bits[13 + i];
    }
    mib_object.pdcch_config = 0;
    for (int i = 0 ; i < 8; i ++){
        mib_object.pdcch_config += pdcch_config_array[i] * pow(2, 8 - 1 - i);
    }
    // Recovering cell barred information
    mib_object.cell_barred = mib_bits[21];
    mib_object.scs = available_scs[mib_bits[7]];

    int k_ssb_array[5] = {mib_bits[29], mib_bits[8], mib_bits[9], mib_bits[10], mib_bits[11]};
    mib_object.k_ssb = 0;
    for (int i = 0 ; i < 5; i ++){
        mib_object.k_ssb += k_ssb_array[i] * pow(2, 5 - 1 - i);
    }
    mib_object.dmrs_type_a_position = 2 + mib_bits[12];
    mib_object.intra_freq_reselection = mib_bits[22];
}



