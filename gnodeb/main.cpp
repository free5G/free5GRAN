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
#include <chrono>

namespace logging = boost::log;
void init_logging(string warning);


/** This function will run continuously to send frames */
void send_buffer_multithread(rf rf_variable_2, vector<complex<float>> * buff_to_send){
    BOOST_LOG_TRIVIAL(warning) << "Function send_buffer_multithread begins ";
    rf_variable_2.buffer_transmition(*buff_to_send);
}


int main(int argc, char *argv[]) {

    /** put 'true' if running_platform is attached to an USRP */
    bool run_with_usrp = true;

    phy phy_variable;
    const char *config_file;
    if (run_with_usrp) {
        config_file = argv[1];
    }
    if (run_with_usrp == false)
    {
        config_file = ("../config/ssb_emission.cfg");
    }

    /** Read Config File with function read_config_gNodeB */
    free5GRAN::utils::common_utils::read_config_gNodeB(config_file);

    /** Initialize mib_object and usrp_info_object */
    free5GRAN::mib mib_object = free5GRAN::gnodeB_config_globale.mib_object;
    free5GRAN::usrp_info usrp_info_object = free5GRAN::gnodeB_config_globale.usrp_info_object;

    /** Initialize log file */
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


    /** Calculate the number of sample that a frame (10 ms) will contain */
    int num_samples_in_frame;
    phy_variable.compute_num_sample_per_frame(mib_object, num_samples_in_frame);

    /** Calculate the number of symbols that a frame (10 ms) will contain */
    int Num_symbols_per_subframe;
    if (mib_object.scs == 15000) {
        Num_symbols_per_subframe = 14;
    } else if (mib_object.scs == 30000) {
        Num_symbols_per_subframe = 28;
    }
    int num_symbols_frame = Num_symbols_per_subframe * 10;
    free5GRAN::num_symbols_frame = num_symbols_frame; // To be optimize, we can reduce number of lines

    /** Display some useful information */
    std::cout << "\n###### RADIO" << std::endl;
    std::cout << "num_samples_in_frame = " << num_samples_in_frame << std::endl;
    std::cout << "num_symbols_in_frame = " << num_symbols_frame << std::endl;
    std::cout << "# Frequency: " << usrp_info_object.center_frequency /1e6<< " MHz" << std::endl;
    std::cout << "# Ifft Size: " << free5GRAN::SIZE_IFFT_SSB << std::endl;
    std::cout << "# ssb_period: " << free5GRAN::gnodeB_config_globale.ssb_period << " second" << std::endl;
    std::cout << "num_samples_in_frame = " << num_samples_in_frame << std::endl;
    std::cout << "\n###### CELL" << std::endl;
    std::cout << "# PCI: " << free5GRAN::gnodeB_config_globale.pci << std::endl;
    std::cout << "# I_B_SSB: " << free5GRAN::gnodeB_config_globale.i_b_ssb << std::endl;
    std::cout << "\n###### MIB" << std::endl;
    std::cout << "# Frame number: varies cyclically between 0 and 1023" << std::endl;
    std::cout << "# PDCCH configuration: " << mib_object.pdcch_config << std::endl;
    std::cout << "# SCS: " << mib_object.scs/1e3 <<" kHz"<<std::endl;
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
    free5GRAN::ONEframe_SSB_freq.resize(num_symbols_frame, std::vector<std::complex<float>>(free5GRAN::SIZE_IFFT_SSB, {0.0, 0.0}));
    free5GRAN::freq_domain_reversed_frame.resize(num_symbols_frame, std::vector<std::complex<float>>(free5GRAN::SIZE_IFFT_SSB, {0.0, 0.0}));
    free5GRAN::time_domain_frame.resize(num_symbols_frame, std::vector<std::complex<float>>(free5GRAN::SIZE_IFFT_SSB, {0.0, 0.0}));


    /** Calculate cp_length */
    int cp_lengths_one_subframe[Num_symbols_per_subframe], cum_sum_cp_lengths[Num_symbols_per_subframe];
    free5GRAN::phy::signal_processing::compute_cp_lengths(mib_object.scs / 1000, free5GRAN::SIZE_IFFT_SSB, 0,
                                                          Num_symbols_per_subframe, &cp_lengths_one_subframe[0],
                                                          &cum_sum_cp_lengths[0]);

    /** Initialize cp_length for each symbols of a frame */
    int cp_lengths_one_frame[num_symbols_frame];
    for (int sub_frame = 0; sub_frame < 10; sub_frame++) {
        for (int symbol = 0; symbol < Num_symbols_per_subframe; symbol++) {
            cp_lengths_one_frame[Num_symbols_per_subframe * sub_frame + symbol] = cp_lengths_one_subframe[symbol];
        }
    }

    /** Calculate the position of ssb block in a frame */
    int index_symbol_ssb = free5GRAN::BAND_N_78.ssb_symbols[free5GRAN::gnodeB_config_globale.i_b_ssb];

    std::cout<<"Size of symbol normal CP = "<<cp_lengths_one_subframe[1] + free5GRAN::SIZE_IFFT_SSB<<"  && Size of symbol long CP = "<<cp_lengths_one_subframe[0] + free5GRAN::SIZE_IFFT_SSB<<std::endl;


    if (run_with_usrp == false) {
        /** Run generate_frame one time for testing */

        int index_symbol_ssb = free5GRAN::BAND_N_78.ssb_symbols[free5GRAN::gnodeB_config_globale.i_b_ssb];
        int sfn = 555;
        std::vector<std::complex<float>> buff_main_10ms(num_samples_in_frame);

        phy_variable.generate_frame(mib_object, index_symbol_ssb, 2, num_symbols_frame, cp_lengths_one_frame, sfn, free5GRAN::gnodeB_config_globale.ssb_period, free5GRAN::gnodeB_config_globale.pci, N, free5GRAN::gnodeB_config_globale.gscn,
                                    free5GRAN::gnodeB_config_globale.i_b_ssb,
                                    free5GRAN::gnodeB_config_globale.scaling_factor, buff_main_10ms);
        free5GRAN::utils::common_utils::display_vector(buff_main_10ms, num_symbols_frame, "\n\nbuff_main_10ms from main");
    }

    /** Sending buffer MULTITHREADING */
    if(run_with_usrp) {

        /** Initialize the 2 buffers. One will be generated while the other will be send */
        std::vector<std::complex<float>> buffer_generated(num_samples_in_frame);
        std::vector<std::complex<float>> buffer_to_send(num_samples_in_frame);
        std::vector<std::complex<float>> buffer_null(num_samples_in_frame, 0);

       /** Initialize the rf (USRP B210) parameters */
       rf rf_variable_2(usrp_info_object.sampling_rate, usrp_info_object.center_frequency,
                        usrp_info_object.gain,usrp_info_object.bandwidth, usrp_info_object.subdev,
                        usrp_info_object.ant, usrp_info_object.ref2, usrp_info_object.device_args);
       BOOST_LOG_TRIVIAL(info) << "Initialize the rf parameters done";

        /** thread sending in continuous buffer_to_send */
        thread sending(send_buffer_multithread, rf_variable_2, &buffer_to_send);

        BOOST_LOG_TRIVIAL(warning) << "index_frame_to_send from main = " + std::to_string(free5GRAN::index_frame_to_send);
        BOOST_LOG_TRIVIAL(warning) << "index_frame_sent from main = " + std::to_string(free5GRAN::index_frame_sent);

        /** Determine number of SSB block in each frame */
        int ssb_period_symbol_int = 0;
        int num_SSB_in_this_frame = 0;
        std::cout<<"gnodeB_config_globale.ssb_period = "<<free5GRAN::gnodeB_config_globale.ssb_period<<std::endl;
        if (free5GRAN::gnodeB_config_globale.ssb_period == float(0.005)){
            num_SSB_in_this_frame = 2;
        }else{
            float ssb_period_symbol = free5GRAN::gnodeB_config_globale.ssb_period / 0.01;
            ssb_period_symbol_int = ssb_period_symbol;
        }

        /** Initialize variables before loop 'while true' */
        int sfn = 0, duration_sum = 0;
        auto start = chrono::high_resolution_clock::now(), stop = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
        int duration_int;

        std::cout << "\nGenerating Frame indefinitely..."<<std::endl;
        while (true) {

            /** If the frame_sent has an index equal to the next frame_to_send, we generate the next frame_to_send */
            if (free5GRAN::index_frame_to_send == free5GRAN::index_frame_sent) {

                /** Calculate the number of ssb block that the next frame will contain. To be optimize */
                if (num_SSB_in_this_frame != 2) {
                    if (sfn % ssb_period_symbol_int == 0) {
                        num_SSB_in_this_frame = 1;
                    } else {
                        num_SSB_in_this_frame = 0;
                    }
                }
                //std::cout<<"num_SSB_in_this_frame = "<<num_SSB_in_this_frame<<std::endl;

                auto start = chrono::high_resolution_clock::now();
                if (num_SSB_in_this_frame == 1 || num_SSB_in_this_frame == 2) {
                    phy_variable.generate_frame(mib_object, index_symbol_ssb, num_SSB_in_this_frame, num_symbols_frame, cp_lengths_one_frame,
                                                sfn, free5GRAN::gnodeB_config_globale.ssb_period,
                                                free5GRAN::gnodeB_config_globale.pci, N,
                                                free5GRAN::gnodeB_config_globale.gscn,
                                                free5GRAN::gnodeB_config_globale.i_b_ssb,
                                                free5GRAN::gnodeB_config_globale.scaling_factor, buffer_generated);
                    BOOST_LOG_TRIVIAL(warning) << "function generate_frame done";
                    buffer_to_send = buffer_generated;
                    BOOST_LOG_TRIVIAL(warning) << "Copy buffer_generated to buffer_to_send done";
                }else{
                    buffer_to_send = buffer_null;
                    BOOST_LOG_TRIVIAL(warning) << "Copy buffer_null to buffer_to_send done";
                }

                free5GRAN::index_frame_to_send = (free5GRAN::index_frame_to_send + 1) % 10000;
                sfn = (sfn + 1) % 1024;
                auto stop = chrono::high_resolution_clock::now();

                /** Calculate the mean duration of the 300 first call of function 'generate_frame' */
                if (free5GRAN::index_frame_to_send < 300) {
                        duration = chrono::duration_cast<chrono::microseconds>(stop - start);
                        duration_int = duration.count();
                        duration_sum = duration_sum + duration_int;
                }
                /** Display the mean duration */
                if (free5GRAN::index_frame_to_send == 301) {
                    float mean_duration = (duration_sum) / 300;
                    cout << "\n" << "duration of generate_frame (mean of 300 first) = " << mean_duration / 1000
                    << " ms" << endl;
                }
            }
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
