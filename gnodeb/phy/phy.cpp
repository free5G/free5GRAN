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
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/file.hpp>




bool display_variable = true; /** indicates if you want to display all variables in the Console */


/**
void phy::compute_cp_lengths(int scs, int nfft, int is_extended_cp, int num_symb_per_subframes, int *cp_lengths, int *cum_sum_cp_lengths){

     *
     * \fn compute_cp_lengths (int scs, int nfft, int is_extended_cp, int num_symb_per_subframes, int *cp_lengths, int *cum_sum_cp_lengths)
     * \brief Compute cyclic prefix size
     * \standard TS38.211 V15.2.0 Section 5.3
     * \param[in] scs: Subcarrier spacing
     * \param[in] nfft: FFT/iFFT size
     * \param[in] is_extended_cp: True if current CP is extended CP
     * \param[in] num_symb_per_subframes: Number of symbols per subframe
     * \param[out] cp_lengths: Returned CP
     * \param[out] cum_sum_cp_lengths: Cumulative CP sum

    int nom_cp = ((is_extended_cp) ? 512 :  144);
    int base_cp = nom_cp * nfft / 2048;
    cum_sum_cp_lengths[0] = 0;
    for (int i = 0; i < num_symb_per_subframes; i ++){
        if (i % (num_symb_per_subframes / 2) == 0){
            cp_lengths[i] = (scs * nfft - num_symb_per_subframes * nfft - (num_symb_per_subframes - 2) * base_cp) / 2;
        }else {
            cp_lengths[i] = base_cp;
        }
        if (i < num_symb_per_subframes - 1){
            cum_sum_cp_lengths[i + 1] = cum_sum_cp_lengths[i] + cp_lengths[i] + nfft;
        }
    }
    BOOST_LOG_TRIVIAL(info) << "function compute_cp_length done.";
    free5GRAN::utils::common_utils::display_table(cp_lengths, num_symb_per_subframes, "cp_lengths");
}
*/



/** TO BE DELETED. THIS FUNCTION IS NOW IN THE LIB
void phy::adding_cp(std::complex<float> ** input_channel, int num_symbols, int num_sc_in, int cp_lengths, std::complex<float> ** output_channel_with_cp){

        * \fn phy * adding_cp (std::complex<float> ** input_channel, int num_symbols, int num_sc_in, int cp_lengths, std::complex<float> ** output_channel_with_cp)
        * \brief This function aims to add the CP (Cyclic Prefix) to the SSB signal (time_domain).
        * \standard !! TS TO BE ADDED !!
        * \param[in] std::complex<float> ** input_channel. In our Case, it is the SSB (time domain) (4*256 symbols)
        * \param[in] int num_symbols. Number of symbol in the input_channel. In our case it's 4.
        * \param[in] int num_sc_in. Number of elements in each symbols. In our case, it is 256/
        * \param[in] int cp_lengths. Number of symbols the Cyclic Prefix should contain
        * \param[out] std::complex<float> ** output_channel_with_cp. In our case, it is the SSB with CP.


    /** Loop over all symbols
    for (int symbol = 0; symbol < num_symbols; symbol++ ) {
        /** Loop over all subcarriers
        for (int sc_out = 0; sc_out < num_sc_in + cp_lengths; sc_out++) {
            if (sc_out < cp_lengths) {
                output_channel_with_cp[symbol][sc_out] = input_channel[symbol][num_sc_in - cp_lengths + sc_out];
            }else{
                output_channel_with_cp[symbol][sc_out] = input_channel[symbol][sc_out - cp_lengths];
            }
        }
    }


    /** Filling a txt file to verify our ssb_time_cp
    std::ofstream out_file_ben;
    out_file_ben.open("time_ssb_ben_cp.txt");
    for (int symbol = 0; symbol < free5GRAN::NUM_SYMBOLS_SSB; symbol ++){
        for (int sc = 0; sc < free5GRAN::SIZE_IFFT_SSB + cp_lengths; sc++) {
            out_file_ben << output_channel_with_cp[symbol][sc];
            out_file_ben << "\n";
        }
    }
    out_file_ben.close();
    BOOST_LOG_TRIVIAL(info) << "function adding_cp done. It will give "+std::to_string(free5GRAN::NUM_SYMBOLS_SSB)+ " * "+std::to_string(free5GRAN::SIZE_IFFT_SSB + cp_lengths)+ " complex symbols";
}
*/






//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------

/** Bellow are some little functions that are used in the functions above */


/** TO BE DELETED. THIS FUNCTION IS NOW INCLUDED
int * phy::convert_pci_into_nid2_and_nid1 (int pci) {


* \fn ph_ben * convert_pci_into_nid2_and_nid1 (int pci)
* \brief This function takes the Physical Cell Id (pci) as enter and return the array n_id which contains n_id_1 and n_id_2.
* \standard TS38.211 V15.2.0 Section 7.4.2.1
*
* \param[in] pci Physical Cell Id

    int n_id_1;
    int n_id_2;
    n_id_2 = pci % 3;
    n_id_1 = (pci - n_id_2)/3;

    int * n_id = new int[2];
    n_id[0] = n_id_1;
    n_id[1] = n_id_2;
    return n_id;
}
*/

/** TO BE DELETED
void phy::display_signal_float(std::complex<float> ** signal_to_display, int num_symbols, int num_sc, char* signal_name){
    for (int symbols = 0; symbols < num_symbols; symbols++){
        std::cout<<""<<std::endl;
        std::cout<<""<<std::endl;
        std::cout<<signal_name<< "of symbol "<<symbols<<" = "<<std::ends;
        free5GRAN::utils::common_utils::display_complex_float(signal_to_display[symbols], num_sc, "");
    }
}
*/

/** TO BE DELETED
void phy::display_vector(std::vector<std::complex<float>> vector_to_display, int vector_size, char* vector_name){

    * \fn ph_ben * display_vector (std::vector<std::complex<float>> *vector_to_display, int vector_size, char* vector_name)
* \brief This function aims to display a vector in the console, using the command std::cout.
*
* \param[in] vector_to_display
* \param[in] vector_size number of element in the vector
* \param[in] vector_name name to display

    for (int i=0; i<vector_size; i++){
        if (i==0){
            std::cout <<""<< std::endl;
            std::cout<< vector_name << " (of size "<< vector_size<<") = "<<std::ends;
        }
        if (i% 10 == 0){         /** 10 here means that every 10 elements displayed, a line break is done
            std::cout <<""<< std::endl;
        }
        std::cout<<vector_to_display[i]<<"  "<< std::ends;
    }

}
*/
/** TO BE DELETED
void phy::display_complex_double(std::complex<double> *vector_to_display, int vector_size, char* vector_name){

    /**
* \fn ph_ben * display_complex_double (std::complex<double> *vector_to_display, int vector_size, char* vector_name)
* \brief This function aims to display a vector in the console, using the command std::cout.
*
* \param[in] vector_to_display
* \param[in] vector_size number of element in the vector
* \param[in] vector_name name to display

    for (int i=0; i<vector_size; i++){
        if (i % 10 == 0){        /** 10 here means that every 10 elements displayed, a line break is done
            std::cout <<""<< std::endl;
        }
        if (i == 0){
            std::cout << +vector_name << ": "<<std::ends;
        }
        std::cout<<vector_to_display[i] <<"   "<< std::ends;
    }
}
*/


/** // TO BE DELETED
void phy::display_complex_float(std::complex<float> *vector_to_display, int vector_size, char* vector_name){


* \fn ph_ben * display_complex_float (std::complex<float> *vector_to_display, int vector_size, char* vector_name)
* \brief This function aims to display a vector in the console, using the command std::cout.
*
* \param[in] vector_to_display
* \param[in] vector_size number of element in the vector
* \param[in] vector_name name to display

    for (int i=0; i<vector_size; i++){
        if (i % 1000 == 0){        /** 10 here means that every 10 elements displayed, a line break is done
            std::cout <<""<< std::endl;
        }
        if (i == 0){
            std::cout << +vector_name << ": "<<std::ends;
        }
        std::cout<<vector_to_display[i] <<"   "<< std::ends;
    }
}
 */

/** // TO BE DELETED
void phy::display_table(int* table_to_display, int size, char* table_name) {


   * \fn ph_ben * display_table (int* table_to_display, int size, char* table_name)
   * \brief This function aims to display a table in the console, using the command std::cout.
   *
   * \param[in] table_to_display
   * \param[in] size number of element in the table
   * \param[in] table_name name to display


    for (int i = 0; i<size; i++) {
        if (i % 70== 0){        /** 70 here means that every 70 elements displayed, a line break is done
            std::cout <<""<< std::endl;
        }
        if (i == 0){
            std::cout << +table_name << ": "<<std::ends;
        }
        std::cout<<table_to_display[i] <<" "<< std::ends;

    }
    std::cout <<""<< std::endl;
}
*/

/** // TO BE DELETED ???
void phy::convert_decimal_to_binary(int size, int decimal, int* table_output) {

      * \fn ph_ben * convert_decimal_to_binary (int size, int decimal, int* table_output)
      * \brief This function aims to convert an integer number into a binary bit sequence.
      *
      * \param[in] size Indicates the number of bits in the output sequence. Please verify that decimal <= 2^size.
      * \param[in] decimal the number to convert into a bit sequence.
      * \param[out] table_output the output bits sequence. Will contain only 1 and 0.


    for (int i = size; i >= 0; i--) {
        if (decimal >= std::pow(2, i-1)) {
            table_output[size-i] = 1;
            decimal = decimal - std::pow(2, i-1);
        } else {
            table_output[size-i] = 0;
        }
    }
}
*/

//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------

/** In this section, big functions are created using the little functions above. */
/** This aims to reduce the length of main_ben.cpp */


/** TO BE DELETED. THIS FUNCTION IS NOW IN THE LIB, transport_channel
void phy::encode_bch(int * mib_bits, int pci, int N, int * rate_matched_bch){

      * \fn ph_ben * encode_bch(int * mib_bits, int pci, int N, int * rate_matched_bch)
      * \brief This function aims to transform the mib_bits into a rate_matched_bch bits sequence.
      * \details Here are the steps of this function: INTERLEAVING, SCRAMBLING, ADDING CRC, POLAR ENCODING and RATE MATCHING.
      * \standard TS38.212 V15.2.0 Section 7.1
      * \standard TS38.212 V15.2.0 Section 5
      *
      * \param[in] mib_bits. In our case, it is a 32 long bits sequence.
      * \param[in] pci. Physical Cell ID.
      * \param[in] N. It is the length of the BCH payload after polar encode. It's a power of 2. In our case, it's 512.
      * \param[out] rate_matched_bch. The output bits sequence. 864 bits long in our case.


    /** INTERLEAVING -> Generating mib_bits_interleaved (32 bits long in our case) from mib_bits. TS38.212 V15.2.0 Section 7.1.1
    int mib_bits_interleaved[free5GRAN::BCH_PAYLOAD_SIZE];
    free5GRAN::phy::transport_channel::bch_payload_integration(mib_bits, mib_bits_interleaved);

    if(display_variable){
        display_table(mib_bits_interleaved, free5GRAN::BCH_PAYLOAD_SIZE, "mib_bits_interleaved from phy");}

    /** SCRAMBLING -> Generating bch_payload (32 bits long in our case) from mib_bits_interleaved. TS38.212 V15.2.0 Section 7.1.2
    int bch_payload[free5GRAN::BCH_PAYLOAD_SIZE];
    int v = mib_bits[25] * 2 + mib_bits[26]; /** v depends on the SFN value (3rd LSB of SFN and 2nd LSB of SFN).
    if (display_variable){std::cout <<"v (depends on SFN, 3rd and 2nd LSB "<<v<< std::endl;}
    free5GRAN::phy::transport_channel::scrambling_bch(v, pci, mib_bits_interleaved, bch_payload);

    if (display_variable){
        display_table(bch_payload, free5GRAN::BCH_PAYLOAD_SIZE, "bch_payload from phy");}

    /** CRC -> Generating bch_payload_with_crc (56 bits long in our case) from bch_payload. TS38.212 V15.2.0 Section 5.1
    int bch_payload_with_crc[free5GRAN::SIZE_PBCH_POLAR_DECODED];
    adding_crc(bch_payload, bch_payload_with_crc);
    if (display_variable){
        display_table(bch_payload_with_crc, free5GRAN::SIZE_PBCH_POLAR_DECODED, "bch_payload_with_crc from phy");}

    /** POLAR ENCODING -> Generating polar encoded_bch (512 bits long in our case) from bch_payload_with_crc. TS38.212 V15.2.0 Section 5.3.1
    int *polar_encoded_bch = new int[N];
    free5GRAN::phy::transport_channel::polar_encoding(N, bch_payload_with_crc, polar_encoded_bch);

    if (display_variable){
        display_table(polar_encoded_bch, N, "polar_encoded_bch from phy");}

    /** RATE MATCHING -> Generating rate_matching_bch (864 bits long in our case) from encoded_bch. TS38.212 V15.2.0 Section 5.4.1
    free5GRAN::phy::transport_channel::rate_matching_polar_coding(polar_encoded_bch, rate_matched_bch);
    if (display_variable){
        display_table(rate_matched_bch, free5GRAN::SIZE_SSB_PBCH_SYMBOLS*2, "rate_matched_bch from phy");}

}
 */

/** TO BE DELETED. THIS FUNCTION IS NOW IN THE LIB/PHYSICAL_CHANNEL. NEW NAME : PBCH_ENCODING

void phy::encode_pbch_and_modulation(int * rate_matched_bch, int pci, int gscn, int i_b_ssb, std::complex<float> * pbch_symbols2){

  * \fn ph_ben * encode_pbch_and_modulation (int * rate_matched_bch, int pci, int gscn, int i_b_ssb, std::complex<float> * pbch_symbols2)
  * \brief This function aims to transform a rate_matched_bch bits sequence into a pbch symbols sequence.
  * \details The 2 main steps are ENCODING and MODULATION.
  * \standard TS38.211 V15.2.0 Section 7.3.3.1
  * \standard TS38.211 V15.2.0 Section 5.1.3
  *
  * \param[in] rate_matched_bch. In our case, it is a 864 long bits sequence.
  * \param[in] pci. Physical Cell ID.
  * \param[in] gscn. Global Synchronization Channel Number.
  * \param[in] i_b_ssb. It is the SSB index. Should be between 0 and 7.
  * \param[out] pbch_symbols2. The output symbols sequence. 432 symbols long in our case.
  */

    /** ENCODING -> Generating encoded_pbch (864 bits long in our case) from rate_matching_bch. TS38.211 V15.2.0 Section 7.3.3.1
    int *encoded_pbch = new int[free5GRAN::SIZE_SSB_PBCH_SYMBOLS*2];
    encode_pbch(gscn, pci, i_b_ssb, rate_matched_bch, encoded_pbch);
    if (display_variable){
        display_table(encoded_pbch, free5GRAN::SIZE_SSB_PBCH_SYMBOLS*2, "encoded_pbch from phy");}

    /** MODULATION -> Generating pbch_symbols2 (432 symbols long in our case) from encoded_pbch, using BPSK or QPSK. TS38.211 V15.2.0 Section 5.1.3

    //modulation(encoded_pbch, free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2, 1, pbch_symbols2);
    free5GRAN::phy::signal_processing::modulation(encoded_pbch, free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2, 1, pbch_symbols2);
    if (display_variable){
        display_complex_float(pbch_symbols2, free5GRAN::SIZE_SSB_PBCH_SYMBOLS, "pbch_symbols2 from phy");}

}
*/


/** TO BE DELETED. THIS FUNCTION IS NOW IN THE LIB
void phy::generate_SSB_time(std::complex<float> * pbch_symbols2, int pci, int i_b_ssb, free5GRAN::mib mib_object, std::complex<float> ** SSB_signal_time_domain){

 * \fn ph_ben * generate_SSB_time (std::complex<float> * pbch_symbols2, int pci, int i_b_ssb, free5GRAN::mib mib_object, std::complex<float> ** SSB_signal_time_domain)
 * \brief This function aims to generate from a pbch sequence a SSB (Synchronization Signal Block), without Cyclic Prefix, in time domain.
 * \standard TS38.211 V15.2.0 Section 7.4
 * \param[in] pbch_symbols2. In our case, it is a 432 symbols sequence.
 * \param[in] pci. Physical Cell ID.
 * \param[in] i_b_ssb. It is the SSB index. Should be between 0 and 7.
 * \param[in] mib_object. The Master Information Block.
 * \param[out] SSB_signal_time_domain. In our case, it is a signal composed of 4*256 elements.



    /** DMRS -> Generating dmrs_symbols (144 symbols long in our case) from pci and i_b_ssb. TS38.211 V15.2.0 Section 7.4.1.4.1
    std::complex<float> *dmrs_symbols;
    dmrs_symbols = new std::complex<float>[free5GRAN::SIZE_SSB_DMRS_SYMBOLS];
    free5GRAN::utils::sequence_generator::generate_pbch_dmrs_sequence(pci, i_b_ssb, dmrs_symbols);
    //generate_dmrs_of_pbch(pci, i_b_ssb, dmrs_symbols); TO BE DELETED
    if (display_variable){
        display_complex_float(dmrs_symbols, free5GRAN::SIZE_SSB_DMRS_SYMBOLS,
                              "dmrs_symbols from phy");}

    /** N_ID_1 & N_ID_2 -> Computing n_id_1 and n_id_2 from the pci. TS38.211 V15.2.0 Section 7.4.2.1
    int * n_id = new int[2];
    n_id = convert_pci_into_nid2_and_nid1(pci);
    int n_id_1 = n_id[0];
    int n_id_2 = n_id[1];
    if (display_variable) {std::cout<<"n_id_1 = "<< n_id_1 <<"; n_id_2 = "<< n_id_2 <<std::endl;}

    /** PSS -> Computing pss_sequence_symbols (127 symbols long in our case) from n_id_2. TS38.211 V15.2.0 Section 7.4.2.2.1
    int * pss_sequence_symbols= new int[free5GRAN::SIZE_PSS_SSS_SIGNAL];
    free5GRAN::utils::sequence_generator::generate_pss_sequence(n_id_2, pss_sequence_symbols);
    if (display_variable){
        display_table(pss_sequence_symbols, free5GRAN::SIZE_PSS_SSS_SIGNAL, "pss_sequence_symbols");}

    /** CONVERTING PSS -> Converting PSS sequence element from int to complex<float> (Imaginary part = 0)
    std::complex<float> *pss_complex_symbols;
    pss_complex_symbols = new std::complex<float>[free5GRAN::SIZE_PSS_SSS_SIGNAL];
    for (int i=0; i<free5GRAN::SIZE_PSS_SSS_SIGNAL; i++){
        pss_complex_symbols[i] = {static_cast<float>(pss_sequence_symbols[i]), 0};
    }
    if (display_variable){
        display_complex_float(pss_complex_symbols, free5GRAN::SIZE_PSS_SSS_SIGNAL, "pss_complex_symbols");}


    /** SSS -> Computing sss_sequence_symbols (127 symbols long in our case) from n_id_1. TS38.211 V15.2.0 Section 7.4.2.3.1
    int * sss_sequence_symbols= new int[free5GRAN::SIZE_PSS_SSS_SIGNAL];
    free5GRAN::utils::sequence_generator::generate_sss_sequence(n_id_1, n_id_2, sss_sequence_symbols);
    if (display_variable){
        display_table(sss_sequence_symbols, free5GRAN::SIZE_PSS_SSS_SIGNAL, "sss_sequence_symbols");}

    /** CONVERTING SSS -> Converting SSS sequence element from int to complex<float> (Imaginary part = 0)
    std::complex<float> *sss_complex_symbols;
    sss_complex_symbols = new std::complex<float>[free5GRAN::SIZE_PSS_SSS_SIGNAL];
    for (int i=0; i<free5GRAN::SIZE_PSS_SSS_SIGNAL; i++){
        sss_complex_symbols[i] = {static_cast<float>(sss_sequence_symbols[i]), 0};
    }
    if (display_variable){
        display_complex_float(sss_complex_symbols, free5GRAN::SIZE_PSS_SSS_SIGNAL, "sss_complex_symbols");}



    /** REFERENCE GRID -> Building reference grid ref to then fill the SSB correctly, according to TS38.211 V15.2.0 Section 7.4.3
    int *** ref;
    ref = new int **[4]; /** There are 4 channels
    ref[0] = new int *[free5GRAN::NUM_SYMBOLS_SSB];
    ref[1] = new int *[free5GRAN::NUM_SYMBOLS_SSB];
    ref[2] = new int *[free5GRAN::NUM_SYMBOLS_SSB];
    ref[3] = new int *[free5GRAN::NUM_SYMBOLS_SSB];

    free5GRAN::phy::signal_processing::build_reference_grid(4,free5GRAN::NUM_SC_SSB, free5GRAN::NUM_SYMBOLS_SSB, pci, ref);

    /** DISPLAY ref

    if (display_variable) {
        for (int channel = 0; channel < 4; channel++) {
            std::cout << "" << std::endl;
            for (int symbol = 0; symbol < free5GRAN::NUM_SYMBOLS_SSB; symbol++) {
                std::cout << "" << std::endl;
                std::cout << "ref channel " << channel << " symbol " << symbol << " = " << std::ends;
                for (int sc = 0; sc < free5GRAN::NUM_SC_SSB; sc++) {
                    if (sc % 50 == 0) {
                        std::cout << " ____ " << std::ends;
                    }
                    std::cout << ref[channel][symbol][sc] << " " << std::ends;
                }
            }
        }
    }

    /** CHANNEL MAPPING --> Fill the SSB with PSS, SSS, PBCH and DMRS, using ref and according to TS38.211 V15.2.0 Section 7.4.3
    std::complex<float> ** SSB_signal_freq_domain;
    SSB_signal_freq_domain = new std::complex<float> *[free5GRAN::NUM_SYMBOLS_SSB];
    SSB_signal_freq_domain[0] = new std::complex<float> [free5GRAN::NUM_SC_SSB];
    SSB_signal_freq_domain[1] = new std::complex<float> [free5GRAN::NUM_SC_SSB];
    SSB_signal_freq_domain[2] = new std::complex<float> [free5GRAN::NUM_SC_SSB];
    SSB_signal_freq_domain[3] = new std::complex<float> [free5GRAN::NUM_SC_SSB];

    //channel_mapper(new std::complex<float>*[4]{pss_complex_symbols, sss_complex_symbols, pbch_symbols2, dmrs_symbols}, ref, SSB_signal_freq_domain, 4, free5GRAN::NUM_SYMBOLS_SSB, free5GRAN::NUM_SC_SSB);
    free5GRAN::phy::signal_processing::channel_mapper(new std::complex<float>*[4]{pss_complex_symbols, sss_complex_symbols, pbch_symbols2, dmrs_symbols}, ref, SSB_signal_freq_domain, 4, free5GRAN::NUM_SYMBOLS_SSB, free5GRAN::NUM_SC_SSB);
    if (display_variable){
        display_signal_float(SSB_signal_freq_domain, free5GRAN::NUM_SYMBOLS_SSB, free5GRAN::NUM_SC_SSB, "SSB_signal_freq_domain from phy");}


    /** SSB FROM 240 TO 256 SYMBOLS
    std::complex<float> ** SSB_signal_extended;
    SSB_signal_extended = new std::complex<float> *[free5GRAN::NUM_SYMBOLS_SSB];
    SSB_signal_extended[0] = new std::complex<float> [free5GRAN::SIZE_IFFT_SSB];
    SSB_signal_extended[1] = new std::complex<float> [free5GRAN::SIZE_IFFT_SSB];
    SSB_signal_extended[2] = new std::complex<float> [free5GRAN::SIZE_IFFT_SSB];
    SSB_signal_extended[3] = new std::complex<float> [free5GRAN::SIZE_IFFT_SSB];

    //increase_size_ssb(SSB_signal_freq_domain, SSB_signal_extended, free5GRAN::NUM_SYMBOLS_SSB, free5GRAN::NUM_SC_SSB, free5GRAN::SIZE_IFFT_SSB);
    free5GRAN::phy::signal_processing::increase_size_ssb(SSB_signal_freq_domain, SSB_signal_extended, free5GRAN::NUM_SYMBOLS_SSB, free5GRAN::NUM_SC_SSB, free5GRAN::SIZE_IFFT_SSB);
    if (display_variable){
        display_signal_float(SSB_signal_extended, free5GRAN::NUM_SYMBOLS_SSB, free5GRAN::SIZE_IFFT_SSB, "SSB_signal_extended from phy");}

    /** REVERSE SSB
    std::complex<float> ** SSB_signal_extended_reversed;
    SSB_signal_extended_reversed = new std::complex<float> *[free5GRAN::NUM_SYMBOLS_SSB];
    SSB_signal_extended_reversed[0] = new std::complex<float> [free5GRAN::SIZE_IFFT_SSB];
    SSB_signal_extended_reversed[1] = new std::complex<float> [free5GRAN::SIZE_IFFT_SSB];
    SSB_signal_extended_reversed[2] = new std::complex<float> [free5GRAN::SIZE_IFFT_SSB];
    SSB_signal_extended_reversed[3] = new std::complex<float> [free5GRAN::SIZE_IFFT_SSB];

    //reverse_ssb(SSB_signal_extended, SSB_signal_extended_reversed, free5GRAN::NUM_SYMBOLS_SSB, free5GRAN::SIZE_IFFT_SSB);
    free5GRAN::phy::signal_processing::reverse_ssb(SSB_signal_extended, SSB_signal_extended_reversed, free5GRAN::NUM_SYMBOLS_SSB, free5GRAN::SIZE_IFFT_SSB);
    if (display_variable){
        display_signal_float(SSB_signal_extended_reversed, free5GRAN::NUM_SYMBOLS_SSB, free5GRAN::SIZE_IFFT_SSB, "SSB_signal_extended_reversed from phy");}


    /**IFFT --> SSB from frequency domain to time domain
    //ifft(SSB_signal_extended_reversed, SSB_signal_time_domain, free5GRAN::SIZE_IFFT_SSB, free5GRAN::SIZE_IFFT_SSB);
    free5GRAN::phy::signal_processing::ifft(SSB_signal_extended_reversed, SSB_signal_time_domain, free5GRAN::SIZE_IFFT_SSB, free5GRAN::SIZE_IFFT_SSB);
    if (display_variable){
        display_signal_float(SSB_signal_time_domain, free5GRAN::NUM_SYMBOLS_SSB, free5GRAN::SIZE_IFFT_SSB, "SSB_signal_time_domain from phy");}

}
*/

//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------







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

phy::phy() {

}

