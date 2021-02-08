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


/** This function will run continuous to send frames */
void send_buffer_multithread(free5GRAN::usrp_info2 usrp_info_object, double ssb_period, rf rf_variable_2, vector<complex<float>> * buff_to_send){
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
    free5GRAN::mib mib_object = free5GRAN::gnodeB_config_globale.mib_object;
    free5GRAN::usrp_info2 usrp_info_object = free5GRAN::gnodeB_config_globale.usrp_info_object;

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

    std::cout << "################ SSB EMISSION #################" << std::endl;

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


    /** Display some useful information */
    std::cout << "###### SSB" << std::endl;
    std::cout << "# ssb_period: " << free5GRAN::gnodeB_config_globale.ssb_period << " second" << std::endl;
    std::cout << "# i_b_ssb: " << free5GRAN::gnodeB_config_globale.i_b_ssb << std::endl;
    std::cout << "# ifft_size: " << free5GRAN::SIZE_IFFT_SSB << std::endl;
    std::cout << "\n###### CELL & MIB" << std::endl;
    std::cout << "# pddchc_config: " << mib_object.pdcch_config << std::endl;
    std::cout << "# k_ssb: " << mib_object.k_ssb << std::endl;
    std::cout << "# SCS: " << mib_object.scs << std::endl;
    std::cout << "# cell_barred: " << mib_object.cell_barred << std::endl;
    std::cout << "# dmrs_type_a_position: " << mib_object.dmrs_type_a_position << std::endl;
    std::cout << "# intra_freq_reselection: " << mib_object.intra_freq_reselection << std::endl;
    std::cout << "# PCI: " << free5GRAN::gnodeB_config_globale.pci << std::endl;
    std::cout << "\n###### USRP" << std::endl;
    std::cout << "# Sampling rate: " << usrp_info_object.sampling_rate << " Hz" << std::endl;
    std::cout << "# Bandwidth: " << usrp_info_object.bandwidth << " Hz" << std::endl;
    std::cout << "# Center frequency: " << usrp_info_object.center_frequency << " Hz" << std::endl;
    std::cout << "# Emission Gain: " << usrp_info_object.gain << " dB\n" << std::endl;

    /** Calculate the number of sample that a frame (10 ms) will contain */
    int Num_samples_in_frame;
    phy_variable.compute_num_sample_per_frame(mib_object, Num_samples_in_frame);
    std::cout << "Num_samples_in_frame = "<<Num_samples_in_frame<<std::endl;

    if (run_with_usrp == false) {
        /** Run generate_frame one time for testing */
        int sfn = 555;
        std::vector<std::complex<float>> buff_main_10ms(Num_samples_in_frame);
        phy_variable.generate_frame(mib_object, sfn, free5GRAN::gnodeB_config_globale.ssb_period, free5GRAN::gnodeB_config_globale.pci, N, free5GRAN::gnodeB_config_globale.gscn,
                                    free5GRAN::gnodeB_config_globale.i_b_ssb,
                                    free5GRAN::gnodeB_config_globale.scaling_factor, buff_main_10ms);
        free5GRAN::utils::common_utils::display_vector(buff_main_10ms, Num_samples_in_frame, "buff_main_10ms");
    }

    /** Sending buffer MULTITHREADING */
    if(run_with_usrp) {

        /** Initialize the 2 buffers. One will be generated while the other will be send */
        std::vector<std::complex<float>> buffer_generated(Num_samples_in_frame);
        std::vector<std::complex<float>> buffer_to_send(Num_samples_in_frame);

       rf rf_variable_2(usrp_info_object.sampling_rate, usrp_info_object.center_frequency,
                        usrp_info_object.gain,usrp_info_object.bandwidth, usrp_info_object.subdev,
                        usrp_info_object.ant, usrp_info_object.ref2, usrp_info_object.device_args);
       BOOST_LOG_TRIVIAL(info) << "Initialize the rf parameters ";

        /** thread sending in continuous buffer_to_send */
        thread sending(send_buffer_multithread, usrp_info_object, free5GRAN::gnodeB_config_globale.ssb_period, rf_variable_2, &buffer_to_send);

        std::cout << "\nGenerating Frame indefinitely..."<<std::endl;

        BOOST_LOG_TRIVIAL(warning) << "index_frame_to_send from main = " + std::to_string(free5GRAN::index_frame_to_send);
        BOOST_LOG_TRIVIAL(warning) << "index_frame_sent from main = " + std::to_string(free5GRAN::index_frame_sent);

        int sfn = 1, i = 0, duration_sum = 0;

        while (true) {
            /** If the frame_sent has an index equal to the next frame_to_send, we generate the next frame_to_send */

                auto start = chrono::high_resolution_clock::now();

                phy_variable.generate_frame(mib_object, sfn, free5GRAN::gnodeB_config_globale.ssb_period, free5GRAN::gnodeB_config_globale.pci, N, free5GRAN::gnodeB_config_globale.gscn, free5GRAN::gnodeB_config_globale.i_b_ssb, free5GRAN::gnodeB_config_globale.scaling_factor, buffer_generated);

                auto stop = chrono::high_resolution_clock::now();
                BOOST_LOG_TRIVIAL(warning) << "function generate_frame done";

                if (free5GRAN::index_frame_to_send == free5GRAN::index_frame_sent) {
                    buffer_to_send = buffer_generated;
                    BOOST_LOG_TRIVIAL(warning) << "Copy buffer_generated to buffer_to_send done";

                    free5GRAN::index_frame_to_send++;
                    sfn = (sfn + 1) % 1024;
                }

                /** Calculate the mean duration of the 300 first call of function 'generate_frame */
                int duration_int;
                if (i < 300) {
                    auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
                    duration_int = duration.count();
                    duration_sum = duration_sum + duration_int;
                }
                if (i == 301) {
                    float mean_duration = (duration_sum)/300;
                    cout <<"\n"<<"duration of generate_frame (mean of 300 first) = "<< mean_duration/1000 <<" ms" <<endl;
                    //free5GRAN::utils::common_utils::display_vector(buffer_generated, Num_samples_in_frame, "buffer_generated");
                }
                i++;

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
