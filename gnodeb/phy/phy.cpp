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
#include "../../lib/phy/physical_channel/physical_channel.h"
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include_next <math.h>


void phy::generate_frame_10ms(free5GRAN::mib mib_object, usrp_info2 usrp_info_object, int sfn, double ssb_period,int pci, int N, int gscn, int i_b_ssb, float scaling_factor, std::vector<std::complex<float>> &buff_main_10ms_5){
//std::vector<std::complex<float>> phy::generate_frame_10ms(free5GRAN::mib mib_object, usrp_info2 usrp_info_object, int sfn, double ssb_period,int pci, int N, int gscn, int i_b_ssb, float scaling_factor){

    mib_object.sfn = sfn;
    BOOST_LOG_TRIVIAL(warning) << "SFN = " + std::to_string(sfn);

    /** MIB GENERATION -> Generate mib_bits sequence (32 bits long in our case) from mib_object. TS38.331 V15.11.0 Section 6.2.2*/
    int mib_bits[free5GRAN::BCH_PAYLOAD_SIZE];
    free5GRAN::utils::common_utils::encode_mib(mib_object, mib_bits);
    BOOST_LOG_TRIVIAL(info) << "MIB GENERATION from generate_frame_10ms";

    /** ENCODE BCH -> Generate rate_matched_bch (864 bits in our case) from mib_bits. TS38.212 V15.2.0 Section 5 */
    int *rate_matched_bch = new int[free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2];
    free5GRAN::phy::transport_channel::bch_encoding(mib_bits, pci, N, rate_matched_bch);
    BOOST_LOG_TRIVIAL(info) << "ENCODE BCH from generate_frame_10ms";

    /** ENCODE PBCH -> Generate pbch_symbols (432 symbols in our case) from rate_matched_bch. TS38.212 V15.2.0 Section 7.3.3.1 and 5.1.3 */
    std::complex<float> *pbch_symbols;
    pbch_symbols = new std::complex<float>[free5GRAN::SIZE_SSB_PBCH_SYMBOLS];
    free5GRAN::phy::physical_channel::pbch_encoding(rate_matched_bch, pci, gscn, i_b_ssb, pbch_symbols);
    BOOST_LOG_TRIVIAL(info) << "ENCODE PBCH from generate_frame_10ms";

    /** GENERATE SSB -> Generate SSB_signal_time_domain (4 * 256 symbols in our case) from pbch_symbols. TS38.211 V15.2.0 Section 7.4 */
    std::complex<float> **SSB_signal_time_domain;
    SSB_signal_time_domain = new std::complex<float> *[free5GRAN::NUM_SYMBOLS_SSB];
    for (int symbol = 0; symbol < free5GRAN::NUM_SYMBOLS_SSB; symbol++) {
        SSB_signal_time_domain[symbol] = new std::complex<float>[free5GRAN::SIZE_IFFT_SSB];
    }

    free5GRAN::phy::signal_processing::generate_time_domain_ssb(pbch_symbols, pci, i_b_ssb, scaling_factor,
                                                                free5GRAN::SIZE_IFFT_SSB, SSB_signal_time_domain);
    BOOST_LOG_TRIVIAL(info) << "GENERATE SSB from generate_frame_10ms";

    /** COMPUTE CP. TS38.211 V15.2.0 Section 5.3 */
    int Num_symbols_per_subframe;
    if (mib_object.scs == 15000) {
        Num_symbols_per_subframe = 14;
    } else if (mib_object.scs == 30000) {
        Num_symbols_per_subframe = 28;
    }

    //std::cout << "Num_symbols_per_subframe = " << Num_symbols_per_subframe << std::endl;

    int cp_lengths[Num_symbols_per_subframe], cum_sum_cp_lengths[Num_symbols_per_subframe];

    free5GRAN::phy::signal_processing::compute_cp_lengths(mib_object.scs / 1000, free5GRAN::SIZE_IFFT_SSB, 0,
                                                          Num_symbols_per_subframe, &cp_lengths[0],
                                                          &cum_sum_cp_lengths[0]);


    BOOST_LOG_TRIVIAL(info) << "COMPUTE CP LENGTH from generate_frame_10ms";
    BOOST_LOG_TRIVIAL(info) << "From generate_frame_10ms cp_lengths[1] (which will be used) = " + std::to_string(cp_lengths[1]);

    /** ADDING CP TO SSB -> Generate SSB_signal_time_domain_CP from SSB_signal_time_domain TS TO BE ADDED */
    std::complex<float> **SSB_signal_time_domain_CP;
    SSB_signal_time_domain_CP = new std::complex<float> *[free5GRAN::NUM_SYMBOLS_SSB];
    for (int symbol = 0; symbol < free5GRAN::NUM_SYMBOLS_SSB; symbol++) {
        SSB_signal_time_domain_CP[symbol] = new std::complex<float>[free5GRAN::SIZE_IFFT_SSB + cp_lengths[1]];
    }

    free5GRAN::phy::signal_processing::adding_cp(SSB_signal_time_domain, free5GRAN::NUM_SYMBOLS_SSB,
                                                 free5GRAN::SIZE_IFFT_SSB, cp_lengths[1],
                                                 SSB_signal_time_domain_CP);

    BOOST_LOG_TRIVIAL(info) << "ADD CP (Cyclic Prefix) to the SSB (time domain) from generate_frame_10ms";


    int Num_samples_per_symbol_SSB = free5GRAN::SIZE_IFFT_SSB + cp_lengths[1];
    float sample_duration = 1 / usrp_info_object.sampling_rate;

    float SSB_duration = free5GRAN::NUM_SYMBOLS_SSB * Num_samples_per_symbol_SSB * sample_duration;
    //std::cout << "SSB_duration = " << SSB_duration << std::endl;
    //std::cout << "sampling_rate = " << usrp_info_object.sampling_rate << std::endl;

    int Num_sample_per_frame =
            (1e-2) *
            usrp_info_object.sampling_rate; //This corresponds to divide the frame duration (10 ms) by the sample_duration.
    //std::cout << "Num_sample_per_frame = " << (Num_sample_per_frame) << std::endl;

    int Num_symbols_per_frame = Num_symbols_per_subframe * 10;
    //std::cout << "Num_symbols_per_frame = " << Num_symbols_per_frame << std::endl;

    int ssb_period_in_samples = ssb_period * usrp_info_object.sampling_rate;
    //std::cout << "ssb_period_in_samples = " << ssb_period_in_samples << std::endl;

    int index_first_ssb_in_frame = free5GRAN::BAND_N_78.ssb_symbols[i_b_ssb];
    //std::cout << "index_first_ssb_in_frame = " << index_first_ssb_in_frame << std::endl;


    //Initialize the cp_length for each symbols of a frame
    int cp_lengths_one_frame[Num_symbols_per_frame];
    for (int sub_frame = 0; sub_frame < 10; sub_frame++) {
        for (int symbol = 0; symbol < Num_symbols_per_subframe; symbol++) {
            cp_lengths_one_frame[Num_symbols_per_subframe * sub_frame + symbol] = cp_lengths[symbol];
        }
    }

    if (free5GRAN::display_variables) {
        free5GRAN::utils::common_utils::display_table(cp_lengths_one_frame, Num_symbols_per_frame,
                                                      "cp_lengths_one_frame from generate_frame_10ms");
    }
    //Initialize the symbol_size for each symbols of a frame

    int symbols_size_one_frame[Num_symbols_per_frame];
    for (int symbol = 0; symbol < Num_symbols_per_frame; symbol++) {
        symbols_size_one_frame[symbol] = free5GRAN::SIZE_IFFT_SSB + cp_lengths_one_frame[symbol];
    }

    if (free5GRAN::display_variables) {
        free5GRAN::utils::common_utils::display_table(symbols_size_one_frame, Num_symbols_per_frame,
                                                      "symbols_size_one_frame from generate_frame_10ms");
        std::cout << "" << std::endl;
    }

    //Initialise one_frame with the right number of sample for each symbols
    std::complex<float> **one_frame;
    one_frame = new std::complex<float> *[Num_symbols_per_frame];
    for (int symbol = 0; symbol < Num_symbols_per_frame; symbol++) {
        one_frame[symbol] = new std::complex<float>[symbols_size_one_frame[symbol]];
    }

    //Fill one_frame with 0 values everywhere
    for (int symbol = 0; symbol < Num_symbols_per_frame; symbol++) {
        for (int sample = 0; sample < symbols_size_one_frame[symbol]; sample++) {
            one_frame[symbol][sample] = {0, 0};
        }
    }

    //Fill one_frame with SSB at the right place
    int index_ssb_in_frame = index_first_ssb_in_frame;
    for (int symbol_SSB_index = 0; symbol_SSB_index < free5GRAN::NUM_SYMBOLS_SSB; symbol_SSB_index++) {
        one_frame[index_ssb_in_frame] = SSB_signal_time_domain_CP[symbol_SSB_index];
        index_ssb_in_frame++;
    }

    //If ssb_period = 0.005 sec, fill one_frame with SSB a second time
    if (ssb_period == 0.005) {
        int index_second_ssb_in_frame = index_first_ssb_in_frame + Num_symbols_per_frame / 2;
        int index_ssb_in_frame = index_second_ssb_in_frame;
        std::cout << "index_second_ssb_in_frame = " << index_second_ssb_in_frame << std::endl;
        for (int symbol_SSB_index = 0; symbol_SSB_index < free5GRAN::NUM_SYMBOLS_SSB; symbol_SSB_index++) {
            one_frame[index_ssb_in_frame] = SSB_signal_time_domain_CP[symbol_SSB_index];
            index_ssb_in_frame++;
        }
    }


    if (free5GRAN::display_variables) {
        /** Display one_frame */
        for (int symbol = 0; symbol < Num_symbols_per_frame; symbol++) {
            std::cout << "symbol " << symbol << " of ";
            free5GRAN::utils::common_utils::display_complex_float(one_frame[symbol], symbols_size_one_frame[symbol],
                                                                  "one_frame = ");
            std::cout << "" << std::endl;
        }
    }



    //Fill a buffer buff_main_10ms_5 with one_frame
    buff_main_10ms_5.clear();
    //std::vector<std::complex<float>> buff_main_10ms_5;

    int Num_samples_in_frame = 0; //Just here to verify.
    for (int symbol = 0; symbol < Num_symbols_per_frame; symbol++) {
        for (int sample = 0; sample < symbols_size_one_frame[symbol]; sample++) {
            //buff_main_10ms_5->push_back(one_frame[symbol][sample]);

            //buff_main_10ms_5.push_back(one_frame[symbol][sample]);
            buff_main_10ms_5[(symbol*Num_symbols_per_frame) + sample] = one_frame[symbol][sample];
            Num_samples_in_frame++;
        }
    }
    //std::cout<<"Num_samples_in_frame in phy = "<<Num_samples_in_frame<<std::endl;
    //std::cout<<"Num_symbols_per_frame in phy = "<<Num_symbols_per_frame<<std::endl;


    if (free5GRAN::display_variables) {
        //Display buff_main_10ms
        //free5GRAN::utils::common_utils::display_vector(buff_main_10ms_5, Num_sample_per_frame, "buff_main_10ms");
    }
    //free5GRAN::utils::common_utils::display_vector(buff_main_10ms_5, Num_sample_per_frame, "buff_main_10ms_5 from generate_frame_10ms");
    //return buff_main_10ms_5;
}





