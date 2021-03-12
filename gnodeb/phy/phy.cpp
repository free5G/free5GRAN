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

 * \author Télécom Paris, P5G Lab ; Benoit Oehmichen & Aymeric de Javel
 * \version 0.2
 * \date February 2021
 */

#include "phy.h"
#include <iostream>
#include "../../lib/utils/sequence_generator/sequence_generator.h"
#include "../../lib/phy/libphy/libphy.h"
#include <complex>
#include <vector>
#include "../../lib/utils/sequence_generator/sequence_generator.h"
#include "../../lib/variables/common_variables/common_variables.h"
#include "../../lib/utils/common_utils/common_utils.h"
#include "../../lib/variables/common_structures/common_structures.h"
#include "../../lib/phy/libphy/libphy.h"
#include "../../lib/phy/transport_channel/transport_channel.h"
#include "../../lib/phy/physical_channel/physical_channel.h"
#include <boost/format.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <thread>
#include <semaphore.h>


phy::phy(free5GRAN::mib mib_object3, int *cp_lengths_one_frame3, int *cum_sum_cp_lengths, int ifft_size, int num_samples_in_frame) {

    this->mib_object = mib_object3;
    this->cp_lengths_one_frame = cp_lengths_one_frame3;
    this->cum_sum_cp_lengths = cum_sum_cp_lengths;
    this->ifft_size = ifft_size;
    this->num_samples_in_frame = num_samples_in_frame;


    /** Resize the 3 buffers */
    free5GRAN::buffer_generated1.resize(num_samples_in_frame,  {0.0, 0.0});
    free5GRAN::buffer_generated2.resize(num_samples_in_frame,  {0.0, 0.0});
    free5GRAN::buffer_null.resize(num_samples_in_frame,  {0.0, 0.0});

    /** Resize some vectors used in function ifft */
    free5GRAN::freq_domain_reversed_frame.resize(free5GRAN::num_symbols_frame, std::vector<std::complex<float>>(free5GRAN::SIZE_IFFT_SSB, {0.0, 0.0}));
    free5GRAN::time_domain_frame.resize(free5GRAN::num_symbols_frame, std::vector<std::complex<float>>(free5GRAN::SIZE_IFFT_SSB, {0.0, 0.0}));
}


void phy::generate_frame(int num_SSB_in_this_frame, int num_symbols_frame, int sfn, int pci, int i_b_ssb, float scaling_factor, std::vector<std::complex<float>> &one_frame_vector) {
    /**
    * \fn generate_frame(free5GRAN::mib mib_object, int num_SSB_in_this_frame, int num_symbols_frame, int *cp_lengths_one_frame, int sfn, double ssb_period,int pci, int N, int gscn, int i_b_ssb, float scaling_factor, std::vector<std::complex<float>> &one_frame_vector)
    * \brief From mib_object and many other parameters, generates a frame of 10 ms containing SSB.
    * \details
               * -step 1: MIB GENERATION. Generate mib_bits from mib_object
               * -step 2: ENCODE BCH. Generate bch bits sequence.
               * -step 3: ENCODE PBCH. Generate pbch symbols
               * -step 4: GENERATE FREQUENCY DOMAIN FRAME. Generate a frequency domain frame with SSB placed in it
               * -step 5: IFFT. Perform ifft for each symbols to get the final 10ms time_domain frame.
    * \standard !! TS to be added !!
    * \param[in] num_SSB_in_this_frame. Number of SSB that the frame will contain. Is calculated in function of ssb_period and sfn. Should be equal to 0, 1 or 2
    * \param[in] num_symbols_frame. Number of symbols that the frame will contain (eg 140 or 280).
    * \param[in] sfn. Sequence Frame Number. Varies between 0 and 1023
    * \param[in] pci. Physical Cell ID. Should be between 0 and 1007.
    * \param[in] i_b_ssb. SSB index (between 0 and 7). Indicates the position of SSB in the frame.
    * \param[in] scaling_factor. Multiplication factor applied to each values before performing ifft
    * \param[out] &one_frame_vector. One dimension vector containing, in time domain, 10 ms of signal with SSB included in it.
    */


    mib_object.sfn = sfn;

    /** Step 1: MIB GENERATION -> Generate mib_bits sequence (32 bits long in our case) from mib_object. TS38.331 V15.11.0 Section 6.2.2*/
    int mib_bits[free5GRAN::BCH_PAYLOAD_SIZE];
    free5GRAN::utils::common_utils::encode_mib(mib_object, mib_bits);
    BOOST_LOG_TRIVIAL(info) << "MIB GENERATION from generate_frame done";

    /** Step 2: ENCODE BCH -> Generate rate_matched_bch (864 bits in our case) from mib_bits. TS38.212 V15.2.0 Section 5 */
    vector<int> rate_matched_bch_vector(free5GRAN::SIZE_SSB_PBCH_SYMBOLS*2, 0);
    free5GRAN::phy::transport_channel::bch_encoding(mib_bits, pci, rate_matched_bch_vector);
    BOOST_LOG_TRIVIAL(info) << "ENCODE BCH from generate_frame";

    /** Step 3: ENCODE PBCH -> Generate pbch_symbols (432 symbols in our case) from rate_matched_bch. TS38.212 V15.2.0 Section 7.3.3.1 and 5.1.3 */
    vector<complex<float>> pbch_symbols_vector(free5GRAN::SIZE_SSB_PBCH_SYMBOLS);
    free5GRAN::phy::physical_channel::pbch_encoding(rate_matched_bch_vector, pci, i_b_ssb, pbch_symbols_vector);
    BOOST_LOG_TRIVIAL(info) << "ENCODE PBCH from generate_frame";

    /** Step 4: GENERATE FREQUENCY DOMAIN FRAME -> Generate freq_domain_frame from pbch_symbols. TS38.211 V15.2.0 Section 7.4 */
    /** Calculate the position of ssb block in a frame */
    int index_symbol_ssb = free5GRAN::BAND_N_78.ssb_symbols[free5GRAN::gnodeB_config_globale.i_b_ssb];

    vector<vector<complex<float>>> ONEframe_SSB_freq(free5GRAN::num_symbols_frame, vector<complex<float>>(free5GRAN::SIZE_IFFT_SSB));


    free5GRAN::phy::signal_processing::generate_freq_domain_frame(pbch_symbols_vector, pci, index_symbol_ssb,
                                                                  num_SSB_in_this_frame, i_b_ssb,ONEframe_SSB_freq);
    BOOST_LOG_TRIVIAL(info) << "GENERATE ONEframe_SSB_freq";


    /** Step 5: IFFT. Perform ifft for each symbols to get the final 10ms time_domain frame */

    /** data_symbols will indicates which symbols of the frame is not nul.
     * If data_symbols[symbol] = 1, a signal processing will be apply to this symbol (reverse, scaling factor and ifft)
     * If data_symbols[symbol] = 0, nothing will be done for this symbol
    */
    vector<int> data_symbols(num_symbols_frame, 0);
    int count = index_symbol_ssb;
    for (int symbol = 0; symbol < free5GRAN::NUM_SYMBOLS_SSB; symbol++){
        /** for each  4 symbols containing SSB, data_symbols[symbol] = 1 */
        data_symbols[count] = 1;
        count ++;
    }

    /** If frame contains a second SSB, 4 more data_symbols */
    if (num_SSB_in_this_frame == 2){
        int count2 = index_symbol_ssb + (num_symbols_frame/2);
        for (int symbol = 0; symbol < free5GRAN::NUM_SYMBOLS_SSB; symbol++){
            data_symbols[count2] = 1;
            count2++;
        }
    }

    /** ifft -> This function are in 4 STEP: Place SSB in an empty frame ; reverse symbols ; ifft for each symbols ; adding CP for each symbols */
    free5GRAN::phy::signal_processing::ifft(ONEframe_SSB_freq, cp_lengths_one_frame, data_symbols, num_symbols_frame, scaling_factor,
                                            one_frame_vector);
    BOOST_LOG_TRIVIAL(info) << "function ifft done";
}







void phy::compute_num_SSB_in_frame(float ssb_period, int sfn, int &num_SSB_in_frame){
    /**
   * \fn compute_num_SSB_in_frame(float ssb_period, int sfn, int &num_SSB_in_frame)
   * \brief Calculates the number of SSB (0, 1 or 2) that a radioframe will contain in function of ssb_period and sfn
   * \param[in] ssb_period. ssb period in second. Should be between 0.005 and 0.160
   * \param[in] sfn. Sequence Frame Number. Varies from 0 to 1023
   * \param[out] &num_SSB_in_frame. Indicates the number of SSB block that the next frame should contain (0, 1 or 2).
   */

    if (ssb_period == float(0.005)){
        num_SSB_in_frame = 2;
    }else{
        float ssb_period_symbol = ssb_period / 0.01;
        int ssb_period_symbol_int = ssb_period_symbol;
        if (sfn % ssb_period_symbol_int == 0){
            num_SSB_in_frame = 1;
        }else{
            num_SSB_in_frame = 0;
        }
    }
}




void phy::continuous_buffer_generation() {

    /** Initialize variables to measure time in loop 'while true' */
    int sfn = 0, duration_sum_generate = 0, i = 0;
    auto start_generate1 = chrono::high_resolution_clock::now(), stop_generate1 = chrono::high_resolution_clock::now();
    auto start_generate2 = chrono::high_resolution_clock::now(), stop_generate2 = chrono::high_resolution_clock::now();
    auto duration_generate1 = chrono::duration_cast<chrono::microseconds>(stop_generate1 - start_generate1);
    auto duration_generate2 = chrono::duration_cast<chrono::microseconds>(stop_generate2 - start_generate2);
    int duration_generate_int1, duration_generate_int2;
    int number_calculate_mean = 400; /** indicates the number of iterations of 'while true' before display the mean durations */
    int num_SSB_in_next_frame;


        while (true) {

            /** GENERATE BUFFER 1 */
            /** Calculate the number of ssb block that the next frame will contain. To be optimize */
            compute_num_SSB_in_frame(free5GRAN::gnodeB_config_globale.ssb_period, sfn, num_SSB_in_next_frame);

            BOOST_LOG_TRIVIAL(warning) << "SFN = " + std::to_string(sfn);
            BOOST_LOG_TRIVIAL(warning) << "num_SSB in next frame = " + std::to_string(num_SSB_in_next_frame);

            sem_wait(&free5GRAN::semaphore_common1); // Waiting until buffer_generated1 finish to be sent

            /** If the frame has to contain 1 or more SSB, we generate it */
            start_generate1 = chrono::high_resolution_clock::now();
            if (num_SSB_in_next_frame != 0) {
                /** generate buffer_generated1 */
                //this->mib_object
                phy::generate_frame(num_SSB_in_next_frame, free5GRAN::num_symbols_frame,
                                            sfn,
                                            free5GRAN::gnodeB_config_globale.pci,
                                            free5GRAN::gnodeB_config_globale.i_b_ssb,
                                            free5GRAN::gnodeB_config_globale.scaling_factor, free5GRAN::buffer_generated1);
                BOOST_LOG_TRIVIAL(warning) << "Buffer 1 has been generated";
            }else{
                //free5GRAN::buffer_generated1 = phy_object.buffer_null;
                free5GRAN::buffer_generated1 = free5GRAN::buffer_null;
                BOOST_LOG_TRIVIAL(warning) << "Buffer 1 has been fill with buffer_null";
            }

            sfn = (sfn + 1) % 1024;
            stop_generate1 = chrono::high_resolution_clock::now();


            /** GENERATE BUFFER 2 */
            /** Calculate the number of ssb block that the next frame will contain */
            compute_num_SSB_in_frame(free5GRAN::gnodeB_config_globale.ssb_period, sfn, num_SSB_in_next_frame);
            BOOST_LOG_TRIVIAL(warning) << "SFN = " + std::to_string(sfn);
            BOOST_LOG_TRIVIAL(warning) << "num_SSB in next frame = " + std::to_string(num_SSB_in_next_frame);

            sem_wait(&free5GRAN::semaphore_common2); // Waiting until buffer_generated2 finish to be sent

            start_generate2 = chrono::high_resolution_clock::now();
            /** If the frame has to contain 1 or more SSB, we generate it */
            if (num_SSB_in_next_frame != 0) {
                /** generate buffer_generated2 */
                phy::generate_frame(num_SSB_in_next_frame, free5GRAN::num_symbols_frame,
                                            sfn,
                                            free5GRAN::gnodeB_config_globale.pci,
                                            free5GRAN::gnodeB_config_globale.i_b_ssb,
                                            free5GRAN::gnodeB_config_globale.scaling_factor, free5GRAN::buffer_generated2);
                BOOST_LOG_TRIVIAL(warning) << "Buffer 2 has been generated";
            }else{
                //free5GRAN::buffer_generated2 = phy_object.buffer_null;
                free5GRAN::buffer_generated2 = free5GRAN::buffer_null;
                BOOST_LOG_TRIVIAL(warning) << "Buffer 2 has been fill with buffer_null";
            }

            sfn = (sfn + 1) % 1024;
            stop_generate2 = chrono::high_resolution_clock::now();

            /** Calculate the mean duration of the number_calculate_mean first call */
            if (i < number_calculate_mean) {
                duration_generate1 = chrono::duration_cast<chrono::microseconds>(stop_generate1 - start_generate1);
                duration_generate_int1 = duration_generate1.count();
                duration_generate2 = chrono::duration_cast<chrono::microseconds>(stop_generate2 - start_generate2);
                duration_generate_int2 = duration_generate2.count();
                duration_sum_generate = duration_sum_generate + duration_generate_int1 + duration_generate_int2;
            }
            /** Display the mean duration */
            if (i == number_calculate_mean + 1) {
                float mean_duration_generate = (duration_sum_generate) / number_calculate_mean / 2;
                cout << "duration of generate (mean of "<<number_calculate_mean<<" last) = " << mean_duration_generate / 1000 << " ms" << endl;
                duration_sum_generate = 0;
            }
            i = (i + 1) % 3000;
        }
}

















/** ################################ DCI - PDCCH ################################ */






void phy::UE_decode_polar_dci(vector<complex<float>> pdcch_symbols, int K, int N, int E, int length_crc, int pci, int agg_level, int polar_decoded_size, int freq_domain_ra_size, int *rnti, bool &validated, free5GRAN::dci_1_0_si_rnti &dci_object){

    /*
     * Demodulate PBCH Signal
     */

    std::cout<<"agg_level from UE_decode = "<<agg_level<<std::endl;
    int pdcch_bits[E];
    free5GRAN::phy::signal_processing::hard_demodulation(pdcch_symbols,pdcch_bits,agg_level * 6 * 9,1);
    free5GRAN::utils::common_utils::display_table(pdcch_bits, E, "UE_pdcch_bits (just after demodulation)");

    /*
        * De-scramble pbch_bits to scrambled_bits
        */
    int c_seq[agg_level * 6 * 9 * 2];
    free5GRAN::utils::sequence_generator::generate_c_sequence((long) pci % (long)pow(2,31), agg_level * 6 * 9 * 2, c_seq,0);

    int rate_matched_dci_table[E];
    free5GRAN::utils::common_utils::scramble(pdcch_bits, c_seq, rate_matched_dci_table, agg_level * 6 * 9 * 2, 0);




    int rate_recovered[N], polar_decoded[K], remainder[length_crc +1], descrambled[polar_decoded_size + length_crc];
    //int descrambled[K + 24];

    int A = K-length_crc;



    /*
     * rate recovering
     */
    free5GRAN::utils::common_utils::display_table(rate_matched_dci_table, E, "UE_rate_matched_dci_table");
    free5GRAN::phy::transport_channel::rate_recover(rate_matched_dci_table, rate_recovered, 0, E, N, K);

    free5GRAN::utils::common_utils::display_table(rate_recovered, N, "UE rate_recoverd");

    /*
     * Polar decoding
     */

    //free5GRAN::phy::transport_channel::polar_decode(rate_matched_dci_table,polar_decoded,N,K,9,1,0,0, E);
    free5GRAN::phy::transport_channel::polar_decode(rate_recovered, polar_decoded, N, polar_decoded_size, 9, 1, 0, 0, E);

    free5GRAN::utils::common_utils::display_table(polar_decoded, K, "UE_polar_decoded");

    /*
     * RNTI de-masking and CRC validation
     */

    for (int i = 0; i < length_crc; i ++){
        descrambled[i] = 1;
    }
    std::cout<<"FROM EU, A = "<<A<<" & K = "<<K<<std::endl;
    for (int i = 0; i < K; i ++){
        if (i < A+8){
            descrambled[i + length_crc] = polar_decoded[i];
        }
        else {
            descrambled[i + length_crc] = (polar_decoded[i] + rnti[i - A - 8]) % 2;
        }
    }

    free5GRAN::utils::common_utils::display_table(descrambled, K + length_crc, "UE_descrambled");

    //free5GRAN::phy::transport_channel::crc_validate(polar_decoded, free5GRAN::G_CRC_24_C, remainder, K, 25);
    free5GRAN::phy::transport_channel::crc_validate(descrambled, free5GRAN::G_CRC_24_C, remainder, K+length_crc, length_crc+1);
    validated = true;
    free5GRAN::utils::common_utils::display_table(remainder, length_crc, "remainder from UE decode");
    for (int i = 0; i < length_crc+1; i ++){
        if (remainder[i] ==1){
            validated = false;
            break;
        }
    }
    BOOST_LOG_TRIVIAL(trace) << "## CRC " << ((validated) ? "validated" :  "not validated");

    int decoded_dci_bits[K-24];
    for (int i = 0; i < K-24; i ++){
        decoded_dci_bits[i] = polar_decoded[i];
    }
    free5GRAN::utils::common_utils::display_table(decoded_dci_bits, K-24, "UE decoded_dci_bits");
    if (validated) {
    //if (true){
        std::cout<<"\nCRC VALIDARED"<<std::endl;
        dci_object.RIV = 0;
        for (int i = 0 ; i < freq_domain_ra_size; i ++){
            dci_object.RIV += decoded_dci_bits[i] * pow(2, freq_domain_ra_size - i - 1);
        }
        std::cout<<"dci_object.RIV from UE decode = "<<dci_object.RIV<<std::endl;
        dci_object.TD_ra = 0;
        for (int i = 0; i < 4; i ++){
            dci_object.TD_ra += decoded_dci_bits[i + freq_domain_ra_size] * pow(2, 4 - i - 1);
        }

        dci_object.vrb_prb_interleaving = decoded_dci_bits[freq_domain_ra_size + 4];

        dci_object.mcs = 0;
        for (int i = 0; i < 5; i ++){
            dci_object.mcs += decoded_dci_bits[i + freq_domain_ra_size + 4 + 1] * pow(2, 5 - i - 1);
        }

        dci_object.rv = 0;
        for (int i = 0; i < 2; i ++){
            dci_object.rv += decoded_dci_bits[i + freq_domain_ra_size + 4 + 1 + 5] * pow(2, 2 - i - 1);
        }

        dci_object.si = decoded_dci_bits[freq_domain_ra_size + 4 + 1 + 5 + 2];
    }/***/
}




