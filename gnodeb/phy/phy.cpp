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
#include "../../lib/phy/libphy/libphy.h"
#include "../../lib/phy/transport_channel/transport_channel.h"
#include "../../lib/phy/physical_channel/physical_channel.h"
#include <boost/format.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <thread>
#include <boost/log/utility/setup/common_attributes.hpp>



void phy::generate_frame(free5GRAN::mib mib_object, int num_SSB_in_this_frame, int num_symbols_frame, int *cp_lengths_one_frame, int sfn, int pci, int N, int i_b_ssb, float scaling_factor, std::vector<std::complex<float>> &one_frame_vector) {
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
    * \param[in] mib_object object MIB created in common_structures.h, including cell_barred, k_ssb, pddchc_config...
    * \param[in] num_SSB_in_this_frame. Number of SSB that the frame will contain. Is calculated in function of ssb_period and sfn. Should be equal to 0, 1 or 2
    * \param[in] num_symbols_frame. Number of symbols that the frame will contain (eg 140 or 280).
    * \param[in] *cp_lengths_one_frame. Cyclic Prefix length for each symbol of a frame
    * \param[in] sfn. Sequence Frame Number. Varies between 0 and 1023
    * \param[in] pci. Physical Cell ID. Should be between 0 and 1007.
    * \param[in] N. Length of BCH after polar encoding.
    * \param[in] i_b_ssb. SSB index (between 0 and 7). Indicates the position of SSB in the frame.
    * \param[in] scaling_factor. Multiplication factor applied to each values before performing ifft
    * \param[out] &one_frame_vector. One dimension vector containing, in time domain, 10 ms of signal with SSB included in it.
    */


    mib_object.sfn = sfn;
    BOOST_LOG_TRIVIAL(warning) << "SFN (from generate_frame) = " + std::to_string(sfn);

    /** Step 1: MIB GENERATION -> Generate mib_bits sequence (32 bits long in our case) from mib_object. TS38.331 V15.11.0 Section 6.2.2*/
    int mib_bits[free5GRAN::BCH_PAYLOAD_SIZE];
    free5GRAN::utils::common_utils::encode_mib(mib_object, mib_bits);
    BOOST_LOG_TRIVIAL(info) << "MIB GENERATION from generate_frame done";

    /** Step 2: ENCODE BCH -> Generate rate_matched_bch (864 bits in our case) from mib_bits. TS38.212 V15.2.0 Section 5 */
    vector<int> rate_matched_bch_vector(free5GRAN::SIZE_SSB_PBCH_SYMBOLS*2, 0);
    free5GRAN::phy::transport_channel::bch_encoding(mib_bits, pci, N, rate_matched_bch_vector);
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






void phy::compute_num_sample_per_frame(free5GRAN::mib mib_object, int &Num_samples_in_frame) {

    /**
   * \fn compute_num_sample_per_frame(free5GRAN::mib mib_object, int &Num_samples_in_frame)
   * \brief Calculates the number of samples (IQ) that a 10 ms radio-frame will contain.
   * \standard !! TS TO BE ADDED !!
   * \param[in] mib_object. parameter SCS will be used
   * \param[out] &Num_samples_in_frame
   */

    /** Calculate number of symbols per subframe */
    int Num_symbols_per_subframe;
    if (mib_object.scs == 15000) {
        Num_symbols_per_subframe = 14;
    } else if (mib_object.scs == 30000) {
        Num_symbols_per_subframe = 28;
    }

    int Num_symbols_per_frame = Num_symbols_per_subframe * 10;

    /** Calculate cp_length for a subframe */
    int cp_lengths[Num_symbols_per_subframe], cum_sum_cp_lengths[Num_symbols_per_subframe];
    free5GRAN::phy::signal_processing::compute_cp_lengths(mib_object.scs / 1000, free5GRAN::SIZE_IFFT_SSB, 0,
                                                          Num_symbols_per_subframe, &cp_lengths[0],
                                                          &cum_sum_cp_lengths[0]);

    /** Calculate cp_length for each symbols of a frame */
    int cp_lengths_one_frame[Num_symbols_per_frame];
    for (int sub_frame = 0; sub_frame < 10; sub_frame++) {
        for (int symbol = 0; symbol < Num_symbols_per_subframe; symbol++) {
            cp_lengths_one_frame[Num_symbols_per_subframe * sub_frame + symbol] = cp_lengths[symbol];
        }
    }

    /** Calculate size of each symbol in a frame */
    int symbols_size_one_frame[Num_symbols_per_frame];
    for (int symbol = 0; symbol < Num_symbols_per_frame; symbol++) {
        symbols_size_one_frame[symbol] = free5GRAN::SIZE_IFFT_SSB + cp_lengths_one_frame[symbol];
    }

    /** Calculate Num_samples_in_frame */
    Num_samples_in_frame = 0;
    for (int symbol = 0; symbol < Num_symbols_per_frame; symbol++) {
        Num_samples_in_frame = Num_samples_in_frame + symbols_size_one_frame[symbol];
    }
}





void phy::reduce_main(bool run_with_usrp, bool run_one_time_ssb, char *argv[]) {

    phy phy_variable;
    const char *config_file;
    if (run_with_usrp == true) {
        config_file = argv[1];
    }
    if (run_with_usrp == false)
    {
        config_file = ("../config/ssb_emission.cfg");
    }

    /** Read Config File with function read_config_gNodeB */
    free5GRAN::utils::common_utils::read_config_gNodeB(config_file);

    /** Initialize log file with log_level */
    init_logging(free5GRAN::gnodeB_config_globale.log_level);

    /** Initialize mib_object and usrp_info_object */
    free5GRAN::mib mib_object = free5GRAN::gnodeB_config_globale.mib_object;
    free5GRAN::usrp_info usrp_info_object = free5GRAN::gnodeB_config_globale.usrp_info_object;

    /** Calculate scs (sub-carrier spacing) in function of center_frequency. scs is stored on MIB on 1 bit */
    /** Calculation according to !! TS TO BE ADDED !! */
    if (usrp_info_object.center_frequency < 3000e6) {
        mib_object.scs = 15e3; /** in Hz */
    } else {
        mib_object.scs = 30e3; /** in Hz */
    }

    /** Calculate sampling_rate */
    usrp_info_object.sampling_rate = free5GRAN::SIZE_IFFT_SSB * mib_object.scs;
    usrp_info_object.bandwidth = usrp_info_object.sampling_rate;

    /** Generate N which is the length of BCH payload after polar encode */
    int n = free5GRAN::phy::transport_channel::compute_N_polar_code(free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2,
                                                                    free5GRAN::SIZE_PBCH_POLAR_DECODED, 9);
    int N = pow(2, n);


    BOOST_LOG_TRIVIAL(info) << "pddchc_config = " + std::to_string(mib_object.pdcch_config);
    BOOST_LOG_TRIVIAL(info) << "k_ssb = " + std::to_string(mib_object.k_ssb);
    BOOST_LOG_TRIVIAL(info) << "scs = " + std::to_string(mib_object.scs);
    BOOST_LOG_TRIVIAL(info) << "dmrs_type_a_position = " + std::to_string(mib_object.dmrs_type_a_position);
    BOOST_LOG_TRIVIAL(info) << "intra_freq_reselection = " + std::to_string(mib_object.intra_freq_reselection);
    BOOST_LOG_TRIVIAL(info) << "cell_barred = " + std::to_string(mib_object.cell_barred);
    BOOST_LOG_TRIVIAL(info) << "pci = " + std::to_string(free5GRAN::gnodeB_config_globale.pci);
    BOOST_LOG_TRIVIAL(info) << "i_b_ssb = " + std::to_string(free5GRAN::gnodeB_config_globale.i_b_ssb);
    BOOST_LOG_TRIVIAL(info) << "ssb_perdiod (seconds) = " + std::to_string(free5GRAN::gnodeB_config_globale.ssb_period);
    BOOST_LOG_TRIVIAL(info) << "n = " + std::to_string(n);
    BOOST_LOG_TRIVIAL(info) << "N (length of BCH payload after polar encode) = " + std::to_string(N);
    BOOST_LOG_TRIVIAL(info) << "sampling rate for USRP = " + std::to_string(usrp_info_object.sampling_rate);
    BOOST_LOG_TRIVIAL(info) << "USRP serial = " + usrp_info_object.device_args;
    BOOST_LOG_TRIVIAL(info) << "USRP subdev = " + usrp_info_object.subdev;
    BOOST_LOG_TRIVIAL(info) << "USRP ant = " + usrp_info_object.ant;
    BOOST_LOG_TRIVIAL(info) << "USRP ref2 = " + usrp_info_object.ref2;
    BOOST_LOG_TRIVIAL(info) << "usrp_info_object.sampling_rate = " + std::to_string(usrp_info_object.sampling_rate);

    /** Calculate number of sample that a frame (10 ms) will contain (= sampling_rate / 100) */
    int num_samples_in_frame;
    phy_variable.compute_num_sample_per_frame(mib_object, num_samples_in_frame);

    /** Calculate number of symbols that a frame (10 ms) will contain */
    int Num_symbols_per_subframe;
    if (mib_object.scs == 15000) {
        Num_symbols_per_subframe = 14;
    } else if (mib_object.scs == 30000) {
        Num_symbols_per_subframe = 28;
    }
    free5GRAN::num_symbols_frame = Num_symbols_per_subframe * 10;

    /** Display some useful information in consol */
    std::cout << "\n###### RADIO" << std::endl;
    std::cout << "num_samples_in_frame = " << num_samples_in_frame << std::endl;
    std::cout << "num_symbols_in_frame = " << free5GRAN::num_symbols_frame << std::endl;
    std::cout << "# Frequency: " << usrp_info_object.center_frequency /1e6<< " MHz" << std::endl;
    std::cout << "###################### Ifft Size: " << free5GRAN::SIZE_IFFT_SSB << std::endl;
    std::cout << "# ssb_period: " << free5GRAN::gnodeB_config_globale.ssb_period << " second" << std::endl;
    std::cout << "num_samples_in_frame = " << num_samples_in_frame << std::endl;
    std::cout << "\n###### CELL" << std::endl;
    std::cout << "# PCI: " << free5GRAN::gnodeB_config_globale.pci << std::endl;
    std::cout << "# I_B_SSB: " << free5GRAN::gnodeB_config_globale.i_b_ssb << std::endl;
    std::cout << "\n###### MIB" << std::endl;
    std::cout << "# Frame number: varies cyclically between 0 and 1023" << std::endl;
    std::cout << "# PDCCH configuration: " << mib_object.pdcch_config << std::endl;
    std::cout << "###################### SCS: " << mib_object.scs/1e3 <<" kHz"<<std::endl;
    std::cout << "# cell_barred: " << mib_object.cell_barred << std::endl;
    std::cout << "# DMRS type A position: " << mib_object.dmrs_type_a_position << std::endl;
    std::cout << "# k SSB: " << mib_object.k_ssb << std::endl;
    std::cout << "# Intra freq reselection: " << mib_object.intra_freq_reselection << std::endl;
    std::cout << "\n###### USRP" << std::endl;
    std::cout << "# Sampling rate: " << usrp_info_object.sampling_rate/1e6 << " MHz" << std::endl;
    std::cout << "# Bandwidth: " << usrp_info_object.bandwidth/1e6 << " MHz" << std::endl;
    std::cout << "# Emission Gain: " << usrp_info_object.gain << " dB\n" << std::endl;
    std::cout << "# Scaling factor = " << free5GRAN::gnodeB_config_globale.scaling_factor<< std::endl;

    /** Resize some vectors used in function ifft */
    free5GRAN::freq_domain_reversed_frame.resize(free5GRAN::num_symbols_frame, std::vector<std::complex<float>>(free5GRAN::SIZE_IFFT_SSB, {0.0, 0.0}));
    free5GRAN::time_domain_frame.resize(free5GRAN::num_symbols_frame, std::vector<std::complex<float>>(free5GRAN::SIZE_IFFT_SSB, {0.0, 0.0}));


    /** Calculate cp_length */
    int cp_lengths_one_subframe[Num_symbols_per_subframe], cum_sum_cp_lengths[Num_symbols_per_subframe];
    free5GRAN::phy::signal_processing::compute_cp_lengths(mib_object.scs / 1000, free5GRAN::SIZE_IFFT_SSB, 0,
                                                          Num_symbols_per_subframe, &cp_lengths_one_subframe[0],
                                                          &cum_sum_cp_lengths[0]);

    /** Initialize cp_length for each symbols of a frame */
    int cp_lengths_one_frame[free5GRAN::num_symbols_frame];
    for (int sub_frame = 0; sub_frame < 10; sub_frame++) {
        for (int symbol = 0; symbol < Num_symbols_per_subframe; symbol++) {
            cp_lengths_one_frame[Num_symbols_per_subframe * sub_frame + symbol] = cp_lengths_one_subframe[symbol];
        }
    }

    std::cout<<"Size of symbol normal CP = "<<cp_lengths_one_subframe[1] + free5GRAN::SIZE_IFFT_SSB<<"  && Size of symbol long CP = "<<cp_lengths_one_subframe[0] + free5GRAN::SIZE_IFFT_SSB<<std::endl;


    if (run_with_usrp == false && run_one_time_ssb == true) {
        /** Run generate_frame one time for testing */

        int sfn = 555;
        std::vector<std::complex<float>> buff_main_10ms(num_samples_in_frame);

        phy_variable.generate_frame(mib_object, 1, free5GRAN::num_symbols_frame, cp_lengths_one_frame, sfn, free5GRAN::gnodeB_config_globale.pci, N,
                                    free5GRAN::gnodeB_config_globale.i_b_ssb,
                                    free5GRAN::gnodeB_config_globale.scaling_factor, buff_main_10ms);
        free5GRAN::utils::common_utils::display_vector(buff_main_10ms, free5GRAN::num_symbols_frame, "\n\nbuff_main_10ms from main");
    }


    /** Sending buffer MULTITHREADING */
    if (run_with_usrp == true){
        /** Initialize the 2 buffers. One will be generated while the other will be send */
        std::vector<std::complex<float>> buffer_generated(num_samples_in_frame);
        std::vector<std::complex<float>> buffer_to_send(num_samples_in_frame); // to be deleted ?
        std::vector<std::complex<float>> buffer_null(num_samples_in_frame, 0);
        free5GRAN::buffer_to_send.resize(num_samples_in_frame);

        /** Determine number of SSB block in each frame */
        int ssb_period_symbol_int = 0, num_SSB_in_this_frame = 0;
        float ssb_period_symbol = 0.0;
        if (free5GRAN::gnodeB_config_globale.ssb_period == float(0.005)){
            num_SSB_in_this_frame = 2;
            ssb_period_symbol = 0.5;
        }else{
            ssb_period_symbol = free5GRAN::gnodeB_config_globale.ssb_period / 0.01;
            ssb_period_symbol_int = ssb_period_symbol;
        }

        /** Initialize variables to measure time in loop 'while true' */
        int sfn = 0, duration_sum_num_SSB = 0, duration_sum_generate = 0, duration_sum_copy = 0, i = 0;
        auto start_num_SSB = chrono::high_resolution_clock::now(), stop_num_SSB = chrono::high_resolution_clock::now();
        auto start_generate = chrono::high_resolution_clock::now(), stop_generate = chrono::high_resolution_clock::now();
        auto start_copy = chrono::high_resolution_clock::now(), stop_copy = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(stop_generate - start_generate);
        auto duration_num_SSB = chrono::duration_cast<chrono::microseconds>(stop_generate - start_generate);
        auto duration_generate = chrono::duration_cast<chrono::microseconds>(stop_generate - start_generate);
        auto duration_copy = chrono::duration_cast<chrono::microseconds>(stop_generate - start_generate);
        int duration_num_SSB_int, duration_generate_int, duration_copy_int;
        int number_calculate_mean = 400; /** indicates the number of iterations of 'while true' before display the mean durations */


        /** Initialize the rf (USRP B210) parameters */
        rf rf_variable_2(usrp_info_object.sampling_rate, usrp_info_object.center_frequency,
                         usrp_info_object.gain, usrp_info_object.bandwidth, usrp_info_object.subdev,
                         usrp_info_object.ant, usrp_info_object.ref2, usrp_info_object.device_args);
        BOOST_LOG_TRIVIAL(info) << "Initialize the rf parameters done";

        /** launch thread sending which will run continuously */
        thread sending(send_buffer_multithread, rf_variable_2, &free5GRAN::buffer_to_send);


        std::cout << "\nGenerating Frame indefinitely..."<<std::endl;
        while (true) {
            BOOST_LOG_TRIVIAL(warning) << "SFN = " + std::to_string(sfn);

            /** Calculate the number of ssb block that the next frame will contain. To be optimize */
            start_num_SSB = chrono::high_resolution_clock::now();
            if (num_SSB_in_this_frame != 2) {
                if (sfn % ssb_period_symbol_int == 0) {
                    num_SSB_in_this_frame = 1;
                } else {
                    num_SSB_in_this_frame = 0;
                }
            }
            stop_num_SSB = chrono::high_resolution_clock::now();

            /** If the frame has to contain 1 or more SSB, we generate it */
            if (num_SSB_in_this_frame == 1 || num_SSB_in_this_frame == 2) {
                start_generate = chrono::high_resolution_clock::now();
                phy_variable.generate_frame(mib_object, num_SSB_in_this_frame, free5GRAN::num_symbols_frame, cp_lengths_one_frame,
                                            sfn,
                                            free5GRAN::gnodeB_config_globale.pci, N,
                                            free5GRAN::gnodeB_config_globale.i_b_ssb,
                                            free5GRAN::gnodeB_config_globale.scaling_factor, buffer_generated);
                stop_generate = chrono::high_resolution_clock::now();
                BOOST_LOG_TRIVIAL(warning) << "function generate_frame done";

                /** Copy buffer_generated into buffer_to_send */
                free5GRAN::mtx_common.lock(); /** mutex is lock to avoid thread 'sending' to run during copy */
                start_copy = chrono::high_resolution_clock::now();
                free5GRAN::buffer_to_send = buffer_generated;
                stop_copy = chrono::high_resolution_clock::now();
                free5GRAN::mtx_common.unlock(); /** mutex is unlock to let thread 'sending' begin to send a frame */
                BOOST_LOG_TRIVIAL(warning) << "Copy buffer_generated to buffer_to_send done";
            }
            /** If the frame doesn't have to contain a SSB, we simply copy an empty buffer */
            if (num_SSB_in_this_frame == 0){
                start_generate = chrono::high_resolution_clock::now();
                stop_generate = chrono::high_resolution_clock::now();

                free5GRAN::mtx_common.lock(); /** mutex is lock to avoid thread 'sending' to run during copy */
                start_copy = chrono::high_resolution_clock::now();
                free5GRAN::buffer_to_send = buffer_null;
                stop_copy = chrono::high_resolution_clock::now();
                free5GRAN::mtx_common.unlock(); /** mutex is unlock to let thread 'sending' begin to send a frame */
                BOOST_LOG_TRIVIAL(warning) << "Copy buffer_null to buffer_to_send done";
            }


            /** Calculate the mean duration of the number_calculate_mean first call */
            if (i < number_calculate_mean) {
                duration_num_SSB = chrono::duration_cast<chrono::microseconds>(stop_num_SSB - start_num_SSB);
                duration_num_SSB_int = duration_num_SSB.count();
                duration_sum_num_SSB = duration_sum_num_SSB + duration_num_SSB_int;

                duration_generate = chrono::duration_cast<chrono::microseconds>(stop_generate - start_generate);
                duration_generate_int = duration_generate.count();
                duration_sum_generate = duration_sum_generate + duration_generate_int;

                duration_copy = chrono::duration_cast<chrono::microseconds>(stop_copy - start_copy);
                duration_copy_int = duration_copy.count();
                duration_sum_copy = duration_sum_copy + duration_copy_int;
            }
            /** Display the mean duration */
            if (i == number_calculate_mean + 1) {
                float mean_duration_num_SSB = (duration_sum_num_SSB) / number_calculate_mean, mean_duration_generate = (duration_sum_generate) / number_calculate_mean, mean_duration_copy = (duration_sum_copy) / number_calculate_mean;
                cout << "duration of num_SSB_in_frame (mean of "<<number_calculate_mean<<" last) = " << mean_duration_num_SSB / 1000 << " ms" << endl;
                cout << "duration of generate (mean of "<<number_calculate_mean<<" last) = " << mean_duration_generate / 1000 << " ms" << endl;
                cout << "duration of copy (mean of "<<number_calculate_mean<<" last) = " << mean_duration_copy / 1000 << " ms" << endl;
                duration_sum_num_SSB = 0, duration_sum_generate = 0, duration_sum_copy = 0;
            }
            i = (i + 1) % 3000;
            sfn = (sfn + 1) % 1024; /** sfn (Sequence Frame Number) varies cyclically between 0 and 1023 */
        }
    }
}







/** Initialize a logging file. !! TO BE PUT IN ANOTHER FILE !!*/
void phy::init_logging(std::string level)
{
    boost::log::register_simple_formatter_factory<boost::log::trivial::severity_level, char>("Severity");
    boost::log::add_file_log
            (
                    boost::log::keywords::file_name = "free5GRAN_gNodeB.log",
                    boost::log::keywords::format = "[%TimeStamp%] [%ThreadID%] [%Severity%] %Message%"
            );

    if (level == "trace"){
        boost::log::core::get()->set_filter
                (
                        boost::log::trivial::severity >= boost::log::trivial::trace
                );
    }else if (level == "debug"){
        boost::log::core::get()->set_filter
                (
                        boost::log::trivial::severity >= boost::log::trivial::debug
                );
    }else if (level == "info"){
        boost::log::core::get()->set_filter
                (
                        boost::log::trivial::severity >= boost::log::trivial::info
                );
    }else if (level == "warning"){
        boost::log::core::get()->set_filter
                (
                        boost::log::trivial::severity >= boost::log::trivial::warning
                );
    }else if (level == "error"){
        boost::log::core::get()->set_filter
                (
                        boost::log::trivial::severity >= boost::log::trivial::error
                );
    }else {
        boost::log::core::get()->set_filter
                (
                        boost::log::trivial::severity >= boost::log::trivial::fatal
                );
    }
    boost::log::add_common_attributes();
}









//------------------------------------------------------------------------------------------------------
/** ################################ DCI - PDCCH ################################ */

void phy::encode_dci(free5GRAN::dci_1_0_si_rnti dci_object, int *dci_bits, int freq_domain_ra_size){
    int index_bit_dci = 0;
    int RIV_binary[freq_domain_ra_size];
    free5GRAN::utils::common_utils::convert_decimal_to_binary(sizeof(RIV_binary)/sizeof(*RIV_binary), dci_object.RIV, RIV_binary);
    for (int bit = 0; bit < freq_domain_ra_size; bit++){
        dci_bits[index_bit_dci] = RIV_binary[bit];
        index_bit_dci ++;
    }

    int TD_ra_binary[4];
    free5GRAN::utils::common_utils::convert_decimal_to_binary(sizeof(TD_ra_binary)/sizeof(*TD_ra_binary), dci_object.TD_ra, TD_ra_binary);
    for (int bit = 0; bit < 4; bit++){
        dci_bits[index_bit_dci] = TD_ra_binary[bit];
        index_bit_dci ++;
    }

    dci_bits[index_bit_dci] = dci_object.vrb_prb_interleaving;
    index_bit_dci ++;

    int mcs_binary[5];
    free5GRAN::utils::common_utils::convert_decimal_to_binary(sizeof(mcs_binary)/sizeof(*mcs_binary), dci_object.mcs, mcs_binary);
    for (int bit = 0; bit < 5; bit++){
        dci_bits[index_bit_dci] = mcs_binary[bit];
        index_bit_dci ++;
    }

    int rv_binary[2];
    free5GRAN::utils::common_utils::convert_decimal_to_binary(sizeof(rv_binary)/sizeof(*rv_binary), dci_object.rv, rv_binary);
    for (int bit = 0; bit < 2; bit++){
        dci_bits[index_bit_dci] = rv_binary[bit];
        index_bit_dci ++;
    }

    dci_bits[index_bit_dci] = dci_object.si;

}


void phy::adding_dci_crc(int *dci_bits, int *dci_bits_with_crc, int *crc_polynom, int length_input, int length_crc, int *rnti){
    int crc_dci_descrambled[length_crc];
    int crc_dci_scrambled[length_crc];
    free5GRAN::phy::transport_channel::compute_crc(dci_bits, crc_polynom, crc_dci_descrambled,  length_input, length_crc);
    free5GRAN::utils::common_utils::display_table(crc_dci_descrambled, length_crc-1, "crc_dci_descrambled");

    free5GRAN::utils::common_utils::scramble(crc_dci_descrambled, rnti, crc_dci_scrambled, length_crc, 0);
    free5GRAN::utils::common_utils::display_table(crc_dci_scrambled, length_crc-1, "crc_dci_scrambled");

    for (int bit = 0; bit < length_input+length_crc; bit++) {
        if (bit < length_input){
            dci_bits_with_crc[bit] = dci_bits[bit];
        }else{
            dci_bits_with_crc[bit] = crc_dci_scrambled[bit-length_input];
        }
    }
}

void phy::UE_decode_polar_dci(std::vector<int> polar_encoded_dci, int K, int N, int E, int freq_domain_ra_size, int *rnti, bool &validated, free5GRAN::dci_1_0_si_rnti dci_object){

    int rate_recovered[N], polar_decoded[K], remainder[25], descrambled[K + 24];

    int A = K-24;

    /*
     * Polar decoding
     */
    free5GRAN::phy::transport_channel::polar_decode(rate_recovered,polar_decoded,N,K,9,1,0,0, E);
    /*
     * RNTI de-masking and CRC validation
     */
    for (int i = 0; i < 24; i ++){
        descrambled[i] = 1;
    }
    for (int i = 0; i < K; i ++){
        if (i < A+8){
            descrambled[i + 24] = polar_decoded[i];
        }
        else {
            descrambled[i + 24] = (polar_decoded[i] + rnti[i - A - 8]) % 2;
        }
    }
    free5GRAN::phy::transport_channel::crc_validate(descrambled, free5GRAN::G_CRC_24_C, remainder, K+24, 25);
    validated = true;
    for (int i = 0; i < 25; i ++){
        if (remainder[i] ==1){
            validated = false;
            break;
        }
    }
    BOOST_LOG_TRIVIAL(trace) << "## CRC " << ((validated) ? "validated" :  "not validated");

    int decoded_dci_bits[K-24];
    for (int i = 0; i < A; i ++){
        decoded_dci_bits[i] = polar_decoded[i];
    }
    if (validated) {

        dci_object.RIV = 0;
        for (int i = 0 ; i < freq_domain_ra_size; i ++){
            dci_object.RIV += decoded_dci_bits[i] * pow(2, freq_domain_ra_size - i - 1);
        }
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
    }
}


