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


#include <iostream>
#include "phy/phy.h"
#include "rf/rf.h"
#include <complex>
#include <thread>
#include <vector>
#include <libconfig.h++>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <cstdlib>
#include "../lib/phy/libphy/libphy.h"
#include "../lib/utils/common_utils/common_utils.h"
#include "../lib/phy/physical_channel/physical_channel.h"
#include "../lib/variables/common_variables/common_variables.h"
#include <uhd.h>
#include <uhd/usrp/multi_usrp.hpp>
#include <chrono>
#include <mutex>
#include <unistd.h>

namespace logging = boost::log;
void init_logging(string warning);
//inline std::mutex mtx_main = {};           // mutex for critical section
uhd::usrp::multi_usrp::sptr usrp2;


/** This function will run continuously to send frames */
void send_buffer_multithread(rf rf_variable_2, vector<complex<float>> * buff_to_send){
    BOOST_LOG_TRIVIAL(warning) << "Function send_buffer_multithread begins ";
    rf_variable_2.buffer_transmition(*buff_to_send);
}

void send_buffer_test_mutex(vector<complex<float>> * buff_to_send){
    BOOST_LOG_TRIVIAL(warning) <<"Function send_buffer_test_mutex begins ";
    buffer_transm_test_mutex(*buff_to_send);
}


int main(int argc, char *argv[]) {


    bool run_with_usrp = true; /** put 'true' if running_platform is attached to an USRP */
    bool run_mutex_test = false; /** put 'true' for running WITHOUT usrp but with multithred */

    phy phy_variable;
    const char *config_file;
    if (run_with_usrp == true) {
        config_file = argv[1];
    }
    if (run_with_usrp == false)
    {
        config_file = ("../config/ssb_emission.cfg");
    }
    if (run_mutex_test == true){
        config_file = ("../config/ssb_emission.cfg");
    }

    /** Read Config File with function read_config_gNodeB */
    free5GRAN::utils::common_utils::read_config_gNodeB(config_file);

    /** Initialize mib_object and usrp_info_object */
    free5GRAN::mib mib_object = free5GRAN::gnodeB_config_globale.mib_object;
    free5GRAN::usrp_info usrp_info_object = free5GRAN::gnodeB_config_globale.usrp_info_object;

    /** Initialize log file with log_level */
    init_logging(free5GRAN::gnodeB_config_globale.log_level);
    free5GRAN::display_variables = free5GRAN::gnodeB_config_globale.display_variable;

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


    /** Calculate the number of sample that a frame (10 ms) will contain (= sampling_rate / 100) */
    int num_samples_in_frame;
    phy_variable.compute_num_sample_per_frame(mib_object, num_samples_in_frame);

    /** Calculate the number of symbols that a frame (10 ms) will contain */
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


    /** Initialize some vectors used in function ifft */
    free5GRAN::ONEframe_SSB_freq.resize(free5GRAN::num_symbols_frame, std::vector<std::complex<float>>(free5GRAN::SIZE_IFFT_SSB, {0.0, 0.0}));
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


    if (run_with_usrp == false) {
        /** Run generate_frame one time for testing */

        int sfn = 555;
        std::vector<std::complex<float>> buff_main_10ms(num_samples_in_frame);

        phy_variable.generate_frame(mib_object, 1, free5GRAN::num_symbols_frame, cp_lengths_one_frame, sfn, free5GRAN::gnodeB_config_globale.pci, N,
                                    free5GRAN::gnodeB_config_globale.i_b_ssb,
                                    free5GRAN::gnodeB_config_globale.scaling_factor, buff_main_10ms);
        free5GRAN::utils::common_utils::display_vector(buff_main_10ms, free5GRAN::num_symbols_frame, "\n\nbuff_main_10ms from main");
    }

    /** TRYING MUTEX */
    if(run_mutex_test) {

        std::vector<std::complex<float>> buffer_to_send(num_samples_in_frame, 0);

        /** launch thread sending_mutex */
        thread sending_mutex(send_buffer_test_mutex, &buffer_to_send);

        unsigned int time_to_generate = 9000;
        unsigned int time_to_copy = 95;
        while (true) {
            std::cout << "GENERATE (main)" << std::endl;
            usleep(time_to_generate);
            free5GRAN::mtx_common.lock();
            std::cout << "COPY (main)" << std::endl;
            usleep(time_to_copy);
            free5GRAN::mtx_common.unlock();
        }
    }



    /** Sending buffer MULTITHREADING */
    if (run_with_usrp == true){
        /** Initialize the 2 buffers. One will be generated while the other will be send */
        std::vector<std::complex<float>> buffer_generated(num_samples_in_frame);
        std::vector<std::complex<float>> buffer_to_send(num_samples_in_frame);
        std::vector<std::complex<float>> buffer_null(num_samples_in_frame, 0);

        /** Initialize the rf (USRP B210) parameters */
        rf rf_variable_2(usrp_info_object.sampling_rate, usrp_info_object.center_frequency,
                         usrp_info_object.gain,usrp_info_object.bandwidth, usrp_info_object.subdev,
                         usrp_info_object.ant, usrp_info_object.ref2, usrp_info_object.device_args);
        BOOST_LOG_TRIVIAL(info) << "Initialize the rf parameters done";

        /** launch thread sending */
        thread sending(send_buffer_multithread, rf_variable_2, &buffer_to_send);

        /** Determine number of SSB block in each frame */
        int ssb_period_symbol_int = 0;
        int num_SSB_in_this_frame = 0;
        float ssb_period_symbol = 0.0;
        std::cout<<"gnodeB_config_globale.ssb_period = "<<free5GRAN::gnodeB_config_globale.ssb_period<<std::endl;
        if (free5GRAN::gnodeB_config_globale.ssb_period == float(0.005)){
            num_SSB_in_this_frame = 2;
            ssb_period_symbol = 0.5;
        }else{
            ssb_period_symbol = free5GRAN::gnodeB_config_globale.ssb_period / 0.01;
            ssb_period_symbol_int = ssb_period_symbol;
        }
        std::cout<<"ssb_period_symbol_int = "<<ssb_period_symbol_int<<std::endl;

        /** Initialize variables before loop 'while true' */
        int sfn = 0, duration_sum = 0, duration_sum_num_SSB = 0, duration_sum_generate = 0, duration_sum_copy = 0, i = 0;
        auto start_num_SSB = chrono::high_resolution_clock::now(), stop_num_SSB = chrono::high_resolution_clock::now();
        auto start_generate = chrono::high_resolution_clock::now(), stop_generate = chrono::high_resolution_clock::now();
        auto start_copy = chrono::high_resolution_clock::now(), stop_copy = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(stop_generate - start_generate);
        auto duration_num_SSB = chrono::duration_cast<chrono::microseconds>(stop_generate - start_generate);
        auto duration_generate = chrono::duration_cast<chrono::microseconds>(stop_generate - start_generate);
        auto duration_copy = chrono::duration_cast<chrono::microseconds>(stop_generate - start_generate);
        int duration_int, duration_num_SSB_int, duration_generate_int, duration_copy_int;
        int number_calculate_mean = 400; /** indicates the number of iterations of 'while true' before display the mean durations */

        std::cout << "\nGenerating Frame indefinitely..."<<std::endl;
        std::cout<<" num_SSB_in_this_frame before calculation = "<<num_SSB_in_this_frame<<std::ends;
        while (true) {
            BOOST_LOG_TRIVIAL(warning) << "SFN = " + std::to_string(sfn);
            start_num_SSB = chrono::high_resolution_clock::now();


            /** Calculate the number of ssb block that the next frame will contain. To be optimize */
            if (num_SSB_in_this_frame != 2) {
                if (sfn % ssb_period_symbol_int == 0) {
                    num_SSB_in_this_frame = 1;
                } else {
                    num_SSB_in_this_frame = 0;
                }
            }
            //std::cout<<" num_SSB = "<<num_SSB_in_this_frame<<std::ends;
            stop_num_SSB = chrono::high_resolution_clock::now();

            if (num_SSB_in_this_frame == 1 || num_SSB_in_this_frame == 2) {
                start_generate = chrono::high_resolution_clock::now();
                //usleep(10000);
                phy_variable.generate_frame(mib_object, num_SSB_in_this_frame, free5GRAN::num_symbols_frame, cp_lengths_one_frame,
                                            sfn,
                                            free5GRAN::gnodeB_config_globale.pci, N,
                                            free5GRAN::gnodeB_config_globale.i_b_ssb,
                                            free5GRAN::gnodeB_config_globale.scaling_factor, buffer_generated);
                //std::cout<<"GENERATE ; "<<std::ends;
                stop_generate = chrono::high_resolution_clock::now();
                BOOST_LOG_TRIVIAL(warning) << "function generate_frame done";


                //usleep(10000);
                free5GRAN::mtx_common.lock();
                start_copy = chrono::high_resolution_clock::now();
                buffer_to_send = buffer_generated;
                stop_copy = chrono::high_resolution_clock::now();
                //std::cout<<"COPY ; "<<std::ends;
                free5GRAN::mtx_common.unlock();

                BOOST_LOG_TRIVIAL(warning) << "Copy buffer_generated to buffer_to_send done";
            }
            if (num_SSB_in_this_frame == 0){
                start_generate = chrono::high_resolution_clock::now();
                stop_generate = chrono::high_resolution_clock::now();

                free5GRAN::mtx_common.lock();
                start_copy = chrono::high_resolution_clock::now();
                buffer_to_send = buffer_null;
                stop_copy = chrono::high_resolution_clock::now();
                free5GRAN::mtx_common.unlock();
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
            sfn = (sfn + 1) % 1024;
        }
    }
}















/** Initialize a logging file*/
void init_logging(std::string level)
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
