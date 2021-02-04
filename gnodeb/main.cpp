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


void send_buffer_multithread(usrp_info2 usrp_info_object, double ssb_period, rf rf_variable_2, vector<complex<float>> * buff_to_send){
    BOOST_LOG_TRIVIAL(warning) << "Function send_buffer_multithread begins ";
    rf_variable_2.buffer_transmition(*buff_to_send);
}



int main(int argc, char *argv[]) {

    /** put 'true' if runing_platform is attached to an USRP */
    bool run_multi_thread = true;

    free5GRAN::mib mib_object;
    usrp_info2 usrp_info_object;
    phy phy_variable;

    /** READING CONFIG FILE */
    libconfig::Config cfg_gNodeB;
    try {
        if (run_multi_thread) {
            cfg_gNodeB.readFile(
                    argv[1]);   /** Use this for CLI launch. command in /build : sudo ./NRPhy_2 ../config/ssb_emission.cfg */
        } else {
            cfg_gNodeB.readFile("../config/ssb_emission.cfg"); /** Use this for launch in CLion */
        }
    }

    /** Return an error if config file is not found */
    catch (libconfig::FileIOException &e) {
        std::cout << "FileIOException occurred. Could not find the config file ssb_emission.cfg!!\n";
        return (EXIT_FAILURE);
    }

    /** Return an error if config file contains parse error */
    catch (libconfig::ParseException &pe) {
        std::cout << "Parse error at " << pe.getFile() << " : " << pe.getLine() << " - " << pe.getError() << std::endl;
        return (EXIT_FAILURE);
    }

    /** Read 'level' in config_file and create the log file */
    std::string level = cfg_gNodeB.lookup("logging");
    std::cout << "log level = " << level << std::endl;
    init_logging(level);

    /** Read 'display_variables' in config_file */
    free5GRAN::display_variables = cfg_gNodeB.lookup("display_variables");

    /** Look at function's name in config file */
    std::string func_gNodeB = cfg_gNodeB.lookup("function");
    const libconfig::Setting &root = cfg_gNodeB.getRoot();

    /** Initialize variables defined in the config file */
    int gscn, pci, i_b_ssb;
    float scaling_factor;
    double ssb_period;

    if (func_gNodeB == "SSB_EMISSION") {
        BOOST_LOG_TRIVIAL(info) << "FUNCTION DETECTED IN CONFIG FILE: SSB EMISSION";
        std::cout << "################ SSB EMISSION #################" << std::endl;
        const libconfig::Setting &mib_info = root["mib_info"], &cell_info = root["cell_info"], &usrp_info = root["usrp_info"];

        /** Fill usrp_info_object with values contained in config file  */
        std::string device_args = usrp_info.lookup("device_args");
        usrp_info_object.device_args = device_args;

        std::string subdev = usrp_info.lookup("subdev");
        usrp_info_object.subdev = subdev;

        std::string ant = usrp_info.lookup("ant");
        usrp_info_object.ant = ant;

        std::string ref2 = usrp_info.lookup("ref2");
        usrp_info_object.ref2 = ref2;

        usrp_info_object.center_frequency = usrp_info.lookup("center_frequency");

        usrp_info_object.gain = usrp_info.lookup("gain");

        scaling_factor = usrp_info.lookup(
                "scaling_factor"); /** Multiplying factor (before ifft) to enhance the radio transmission */

        /** Calculate scs (sub-carrier spacing) in function of center_frequency. scs is stored on MIB on 1 bit */
        /** Calculation according to !! TS TO BE ADDED !! */
        if (usrp_info_object.center_frequency < 3000e6) {
            mib_object.scs = 15e3; /** in Hz */
        } else {
            mib_object.scs = 30e3; /** in Hz */
        }

        usrp_info_object.sampling_rate = free5GRAN::SIZE_IFFT_SSB * mib_object.scs;
        usrp_info_object.bandwidth = usrp_info_object.sampling_rate;


        /** Fill mib_object with values in config file */
        mib_object.pdcch_config = mib_info.lookup("pddchc_config"); /** stored on MIB on 8 bits */
        mib_object.k_ssb = mib_info.lookup(
                "k_ssb"); /** stored on MIB on 5 bits. Number of Ressource Blocks between point A and SSB */
        mib_object.cell_barred = mib_info.lookup("cell_barred"); /** stored on MIB on 1 bit */
        mib_object.dmrs_type_a_position = mib_info.lookup("dmrs_type_a_position"); /** stored on MIB on 1 bit */
        mib_object.intra_freq_reselection = mib_info.lookup("intra_freq_reselection"); /** stored on MIB on 1 bit */



        /** Fill cell_info with values contained in config file */
        pci = cell_info.lookup("pci"); /** (Physical Cell Id). int between 0 and 1007 */
        i_b_ssb = cell_info.lookup("i_b_ssb"); /** SSB index. int between 0 and 7. */
        ssb_period = cell_info.lookup("ssb_period"); /** in seconds */


    } else {
        std::cout << "Please enter a function name in config file" << std::endl;
        BOOST_LOG_TRIVIAL(error) << "couldn't recognize function's name in config file";
        return (EXIT_FAILURE);
    }

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
    BOOST_LOG_TRIVIAL(info) << "pci = " + std::to_string(pci);
    BOOST_LOG_TRIVIAL(info) << "i_b_ssb = " + std::to_string(i_b_ssb);
    BOOST_LOG_TRIVIAL(info) << "ssb_perdiod (seconds) = " + std::to_string(ssb_period);
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
    std::cout << "# ssb_period: " << ssb_period << " second" << std::endl;
    std::cout << "# i_b_ssb: " << i_b_ssb << std::endl;
    std::cout << "# ifft_size: " << free5GRAN::SIZE_IFFT_SSB << std::endl;
    std::cout << "" << std::endl;
    std::cout << "###### CELL & MIB" << std::endl;
    std::cout << "# pddchc_config: " << mib_object.pdcch_config << std::endl;
    std::cout << "# k_ssb: " << mib_object.k_ssb << std::endl;
    std::cout << "# SCS: " << mib_object.scs << std::endl;
    std::cout << "# cell_barred: " << mib_object.cell_barred << std::endl;
    std::cout << "# dmrs_type_a_position: " << mib_object.dmrs_type_a_position << std::endl;
    std::cout << "# intra_freq_reselection: " << mib_object.intra_freq_reselection << std::endl;
    std::cout << "# PCI: " << pci << std::endl;
    std::cout << "" << std::endl;
    std::cout << "###### USRP" << std::endl;
    std::cout << "# Sampling rate: " << usrp_info_object.sampling_rate << " Hz" << std::endl;
    std::cout << "# Bandwidth: " << usrp_info_object.bandwidth << " Hz" << std::endl;
    std::cout << "# Center frequency: " << usrp_info_object.center_frequency << " Hz" << std::endl;
    std::cout << "# Emission Gain: " << usrp_info_object.gain << " dB" << std::endl;
    std::cout << "" << std::endl;





    /** Calculate the number of sample that a frame (10 ms) will contain */
    int Num_samples_in_frame;
    phy_variable.generate_num_sample_per_frame(mib_object, Num_samples_in_frame);
    std::cout << "Num_samples_in_frame = "<<Num_samples_in_frame<<std::endl;


    /** Some test to generate one_frame_1_dimension */

    int sfn = 500;
    std::vector<std::complex<float>> buff_main_10ms(Num_samples_in_frame);
    phy_variable.generate_frame_10ms(mib_object, usrp_info_object, sfn, ssb_period, pci, N, gscn,
                                     i_b_ssb,
                                     scaling_factor, buff_main_10ms);

    //free5GRAN::utils::common_utils::display_vector(buff_main_10ms, Num_samples_in_frame, "buff_main_10ms");


    /** Sending buff_10ms MULTITHREAD */
    if(run_multi_thread) {

        /** Initialize the 2 buffers. One will be generated while the other will be send */
        std::vector<std::complex<float>> buff_10ms_generate(Num_samples_in_frame);
        std::vector<std::complex<float>> buff_10ms_sending(Num_samples_in_frame);

       rf rf_variable_2(usrp_info_object.sampling_rate, usrp_info_object.center_frequency,
                       usrp_info_object.gain, usrp_info_object.bandwidth, usrp_info_object.subdev,
                       usrp_info_object.ant, usrp_info_object.ref2, usrp_info_object.device_args);
       BOOST_LOG_TRIVIAL(info) << "Initialize the rf parameters ";


        /** thread sending in continuous buff_10ms_sending */
        thread t8(send_buffer_multithread, usrp_info_object, ssb_period, rf_variable_2, &buff_10ms_sending);



        std::cout << "\nGenerating Frame indefinitely..."<<std::endl;

        BOOST_LOG_TRIVIAL(warning) << "index_frame_to_send from main = " + std::to_string(free5GRAN::index_frame_to_send);
        BOOST_LOG_TRIVIAL(warning) << "index_frame_sent from main = " + std::to_string(free5GRAN::index_frame_sent);

        int sfn = 1;
        int i = 0;
        int duration_sum = 0;

        while (true) {
            /** If the frame_sent has an index equal to the next frame_to_send, we generate the next frame_to_send */
            if (free5GRAN::index_frame_to_send == free5GRAN::index_frame_sent) {
                auto start = chrono::high_resolution_clock::now();

                phy_variable.generate_frame_10ms(mib_object, usrp_info_object, sfn, ssb_period, pci, N, gscn,
                                                 i_b_ssb,
                                                 scaling_factor, buff_10ms_generate);

                BOOST_LOG_TRIVIAL(fatal) << "function generate_frame_10ms done";

                buff_10ms_sending = buff_10ms_generate;
                BOOST_LOG_TRIVIAL(fatal) << "Copy buff_10ms_generate to buff_10ms_sending done";

                free5GRAN::index_frame_to_send++;

                sfn = (sfn+1)%1024;

                auto stop = chrono::high_resolution_clock::now();

                /** Calculate the mean duration of the 300 first call of function 'generate_frame_10ms */
                int duration_int;
                if (i < 300) {
                    auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
                    duration_int = duration.count();
                    duration_sum = duration_sum + duration_int;
                }
                if (i == 301) {
                    float mean_duration = (duration_sum)/300;
                    cout <<"\n"<<"duration of generate_frame_10ms (mean of 300 first) = "<< mean_duration/1000 <<" ms" <<endl;
                    //free5GRAN::utils::common_utils::display_vector(buff_10ms_generate, Num_samples_in_frame, "buff_10ms_generate");
                    //free5GRAN::utils::common_utils::display_vector(buff_10ms_sending, Num_samples_in_frame, "buff_10ms_sending");
                }
                i++;
            }
        }
    }
}



















/** Initialize a logging file */
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

