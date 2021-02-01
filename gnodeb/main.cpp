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
#include <mutex>
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

            //auto start3 = chrono::high_resolution_clock::now();

            rf_variable_2.buffer_transmition(*buff_to_send);

            //auto stop3 = chrono::high_resolution_clock::now();
            //auto duration3 = chrono::duration_cast<chrono::microseconds>(stop3 - start3);

            //cout <<"duration of buffer_transmition = "<<duration3.count() << endl;
            //BOOST_LOG_TRIVIAL(warning) << "duration of buffer_transmittion = "+ std::to_string(duration3.count());
}



int main(int argc, char *argv[]) {

    /** put 'true' in one of the following if runing_platform is attached to an USRP */
    bool run_with_usrp_10ms = false;
    bool run_with_usrp_5ms = false;
    bool run_multi_thread = true;

    free5GRAN::mib mib_object;
    usrp_info2 usrp_info_object;

    /** READING CONFIG FILE */
    libconfig::Config cfg_gNodeB;

    try {
        if (run_with_usrp_10ms || run_with_usrp_5ms) {
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


        /** Fill usrp_info with values contained in config file  */
        //sampling_rate = usrp_info.lookup("sampling_rate");
        //sampling_rate = free5GRAN::SIZE_IFFT_SSB * mib_object.scs;

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
                "scaling_factor"); /** Dividing factor (before ifft) to enhance the radio transmission */
        //scaling_factor = 1;

        /** Calculate scs (sub-carrier spacing) in function of center_frequency. scs is stored on MIB on 1 bit */
        /** Calculation according to !! TS TO BE ADDED !! */
        if (usrp_info_object.center_frequency < 3000e6) {
            mib_object.scs = 15e3; /** in Hz */
        } else {
            mib_object.scs = 30e3; /** in Hz */
        }

        //usrp_info_object.sampling_rate = usrp_info.lookup("sampling_rate");
        usrp_info_object.sampling_rate = free5GRAN::SIZE_IFFT_SSB * mib_object.scs;

        //usrp_info_object.bandwidth = usrp_info.lookup("bandwidth");
        usrp_info_object.bandwidth = usrp_info_object.sampling_rate;


        /** Fill mib_object with values in config file */
        mib_object.sfn = mib_info.lookup("sfn"); /** stored on MIB on 10 bits */
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


    BOOST_LOG_TRIVIAL(info) << "sfn = " + std::to_string(mib_object.sfn);
    BOOST_LOG_TRIVIAL(info) << "pddchc_config = " + std::to_string(mib_object.pdcch_config);
    BOOST_LOG_TRIVIAL(info) << "k_ssb = " + std::to_string(mib_object.k_ssb);
    BOOST_LOG_TRIVIAL(info) << "scs = " + std::to_string(mib_object.scs);
    BOOST_LOG_TRIVIAL(info) << "dmrs_type_a_position = " + std::to_string(mib_object.dmrs_type_a_position);
    BOOST_LOG_TRIVIAL(info) << "intra_freq_reselection = " + std::to_string(mib_object.intra_freq_reselection);
    BOOST_LOG_TRIVIAL(info) << "cell_barred = " + std::to_string(mib_object.cell_barred);


    /** Generate N which is the length of BCH payload after polar encode */
    int n = free5GRAN::phy::transport_channel::compute_N_polar_code(free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2,
                                                                    free5GRAN::SIZE_PBCH_POLAR_DECODED, 9);
    int N = pow(2, n);

    BOOST_LOG_TRIVIAL(info) << "pci = " + std::to_string(pci);
    BOOST_LOG_TRIVIAL(info) << "i_b_ssb = " + std::to_string(i_b_ssb);
    BOOST_LOG_TRIVIAL(info) << "ssb_perdiod (seconds) = " + std::to_string(ssb_period);
    BOOST_LOG_TRIVIAL(info) << "n = " + std::to_string(n);
    BOOST_LOG_TRIVIAL(info) << "N (length of BCH payload after polar encode) = " + std::to_string(N);


    phy phy_variable;

    /** MIB GENERATION -> Generate mib_bits sequence (32 bits long in our case) from mib_object. TS38.331 V15.11.0 Section 6.2.2*/
    int mib_bits[free5GRAN::BCH_PAYLOAD_SIZE];
    free5GRAN::utils::common_utils::encode_mib(mib_object, mib_bits);
    BOOST_LOG_TRIVIAL(info) << "MIB GENERATION";

    if (free5GRAN::display_variables) {
        free5GRAN::utils::common_utils::display_table(mib_bits, free5GRAN::BCH_PAYLOAD_SIZE, "mib_bits from main");
    }


    /** ENCODE BCH -> Generate rate_matched_bch (864 bits in our case) from mib_bits. TS38.212 V15.2.0 Section 5 */
    int *rate_matched_bch = new int[free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2];
    free5GRAN::phy::transport_channel::bch_encoding(mib_bits, pci, N, rate_matched_bch);

    if (free5GRAN::display_variables) {
        free5GRAN::utils::common_utils::display_table(rate_matched_bch, free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2,
                                                      "rate_matched_bch from main");
    }
    BOOST_LOG_TRIVIAL(info) << "ENCODE BCH";

    /** ENCODE PBCH -> Generate pbch_symbols (432 symbols in our case) from rate_matched_bch. TS38.212 V15.2.0 Section 7.3.3.1 and 5.1.3 */
    std::complex<float> *pbch_symbols;
    pbch_symbols = new std::complex<float>[free5GRAN::SIZE_SSB_PBCH_SYMBOLS];
    free5GRAN::phy::physical_channel::pbch_encoding(rate_matched_bch, pci, gscn, i_b_ssb, pbch_symbols);
    BOOST_LOG_TRIVIAL(info) << "ENCODE PBCH";

    if (free5GRAN::display_variables) {
        free5GRAN::utils::common_utils::display_complex_float(pbch_symbols, free5GRAN::SIZE_SSB_PBCH_SYMBOLS,
                                                              "pbch_symbols from main");
    }

    /** GENERATE SSB -> Generate SSB_signal_time_domain (4 * 256 symbols in our case) from pbch_symbols. TS38.211 V15.2.0 Section 7.4 */
    std::complex<float> **SSB_signal_time_domain;
    SSB_signal_time_domain = new std::complex<float> *[free5GRAN::NUM_SYMBOLS_SSB];
    for (int symbol = 0; symbol < free5GRAN::NUM_SYMBOLS_SSB; symbol++) {
        SSB_signal_time_domain[symbol] = new std::complex<float>[free5GRAN::SIZE_IFFT_SSB];
    }

    free5GRAN::phy::signal_processing::generate_time_domain_ssb(pbch_symbols, pci, i_b_ssb, scaling_factor,
                                                                free5GRAN::SIZE_IFFT_SSB, SSB_signal_time_domain);
    BOOST_LOG_TRIVIAL(info) << "GENERATE SSB";


    /** COMPUTE CP. TS38.211 V15.2.0 Section 5.3 */
    int Num_symbols_per_subframe;
    if (mib_object.scs == 15000) {
        Num_symbols_per_subframe = 14;
    } else if (mib_object.scs == 30000) {
        Num_symbols_per_subframe = 28;
    }

    std::cout << "Num_symbols_per_subframe = " << Num_symbols_per_subframe << std::endl;

    int cp_lengths[Num_symbols_per_subframe], cum_sum_cp_lengths[Num_symbols_per_subframe];

    free5GRAN::phy::signal_processing::compute_cp_lengths(mib_object.scs / 1000, free5GRAN::SIZE_IFFT_SSB, 0,
                                                          Num_symbols_per_subframe, &cp_lengths[0],
                                                          &cum_sum_cp_lengths[0]);


    if (free5GRAN::display_variables) {
    }
    std::cout << "cp_length[1] = " << cp_lengths[1] << std::endl;
    std::cout << "" << std::endl;
    free5GRAN::utils::common_utils::display_table(cp_lengths, Num_symbols_per_subframe, "cp_lengths from main");

    BOOST_LOG_TRIVIAL(info) << "COMPUTE CP LENGTH";
    BOOST_LOG_TRIVIAL(info) << "cp_lengths[1] (which will be used) = " + std::to_string(cp_lengths[1]);

    /** ADDING CP TO SSB -> Generate SSB_signal_time_domain_CP from SSB_signal_time_domain TS TO BE ADDED */
    std::complex<float> **SSB_signal_time_domain_CP;
    SSB_signal_time_domain_CP = new std::complex<float> *[free5GRAN::NUM_SYMBOLS_SSB];
    for (int symbol = 0; symbol < free5GRAN::NUM_SYMBOLS_SSB; symbol++) {
        SSB_signal_time_domain_CP[symbol] = new std::complex<float>[free5GRAN::SIZE_IFFT_SSB + cp_lengths[1]];
    }

    free5GRAN::phy::signal_processing::adding_cp(SSB_signal_time_domain, free5GRAN::NUM_SYMBOLS_SSB,
                                                 free5GRAN::SIZE_IFFT_SSB, cp_lengths[1],
                                                 (std::complex<float> **) SSB_signal_time_domain_CP);

    BOOST_LOG_TRIVIAL(info) << "ADD CP (Cyclic Prefix) to the SSB (time domain)";

    if (free5GRAN::display_variables) {
        free5GRAN::utils::common_utils::display_signal_float(SSB_signal_time_domain_CP, free5GRAN::NUM_SYMBOLS_SSB,
                                                             free5GRAN::SIZE_IFFT_SSB + cp_lengths[1],
                                                             "SSB_signal_time_domain_CP from main ");
    }




    //-----------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------

    if (free5GRAN::display_variables) {
        std::cout
                << " //-----------------------------------------------------------------------------------------------------------"
                << std::endl;
        std::cout
                << " //-----------------------------------------------------------------------------------------------------------"
                << std::endl;
    }




    /** SENDING SIGNAL TO USRP  */



    int Num_samples_per_symbol_SSB = free5GRAN::SIZE_IFFT_SSB + cp_lengths[1];
    int num_symbols_SSB = 4;
    float sample_duration = 1 / usrp_info_object.sampling_rate;

    BOOST_LOG_TRIVIAL(info) << "sampling rate for USRP = " + std::to_string(usrp_info_object.sampling_rate);
    BOOST_LOG_TRIVIAL(info) << "Num_samples_per_symbol_SSB = " + std::to_string(Num_samples_per_symbol_SSB);
    BOOST_LOG_TRIVIAL(info) << "num_symbols_SSB of SSB: " + std::to_string(num_symbols_SSB);
    BOOST_LOG_TRIVIAL(info) << "sample duration: " + std::to_string(sample_duration);

    std::cout << "###### SSB" << std::endl;
    std::cout << "# ssb_period: " << ssb_period << " second" << std::endl;
    std::cout << "# i_b_ssb: " << i_b_ssb << std::endl;
    std::cout << "# Num_samples_per_symbol_SSB: " << Num_samples_per_symbol_SSB << std::endl;
    std::cout << "# num_symbols_SSB: " << num_symbols_SSB << std::endl;
    std::cout << "# sample_duration (1/sampling_rate): " << sample_duration << " second" << std::endl;
    std::cout << "# ifft_size: " << free5GRAN::SIZE_IFFT_SSB << std::endl;
    std::cout << "" << std::endl;



    BOOST_LOG_TRIVIAL(info) << "USRP serial = " + usrp_info_object.device_args;
    BOOST_LOG_TRIVIAL(info) << "USRP subdev = " + usrp_info_object.subdev;
    BOOST_LOG_TRIVIAL(info) << "USRP ant = " + usrp_info_object.ant;
    BOOST_LOG_TRIVIAL(info) << "USRP ref2 = " + usrp_info_object.ref2;
    BOOST_LOG_TRIVIAL(info) << "usrp_info_object.sampling_rate = " + std::to_string(usrp_info_object.sampling_rate);


    /** Fill buff_main with signal */
    std::vector<std::complex<double>> buff_main;
    for (int symbol = 0; symbol < 4; symbol++) {
        for (int sample = 0; sample < Num_samples_per_symbol_SSB; sample++) {
            buff_main.push_back(SSB_signal_time_domain_CP[symbol][sample]);
        }
    }

    BOOST_LOG_TRIVIAL(info) << "Fill buff_main (used to send SSB in continuous)";



    //Display some useful information
    std::cout << "###### CELL & MIB" << std::endl;
    std::cout << "# SFN: " << mib_object.sfn << std::endl;
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





    /**################ BELOW IS UNDER CONSTRUCTION !! ##################### */

    //Create a radio-frame of 10 ms. SSB will be placed in it in function of i_b_ssb, SCS and ssb_period
    std::cout << " ######### BUILD A RADIO-FRAME OF 10 MS ###########" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "num_symbols_SSB = " << num_symbols_SSB << std::endl;
    std::cout << "Num_samples_per_symbol_SSB = " << Num_samples_per_symbol_SSB << std::endl;
    std::cout << "sample_duration = " << sample_duration << std::endl;


    int Num_sample_per_frame =
            (1e-2) *
            usrp_info_object.sampling_rate; //This corresponds to divide the frame duration (10 ms) by the sample_duration.
    std::cout << "Num_sample_per_frame = " << (Num_sample_per_frame) << std::endl;

    int Num_symbols_per_frame = Num_symbols_per_subframe * 10;
    std::cout << "Num_symbols_per_frame = " << Num_symbols_per_frame << std::endl;


    int index_first_ssb_in_frame = free5GRAN::BAND_N_78.ssb_symbols[i_b_ssb];
    std::cout << "index_first_ssb_in_frame = " << index_first_ssb_in_frame << std::endl;

    if (free5GRAN::display_variables) {
        free5GRAN::utils::common_utils::display_table(cp_lengths, Num_symbols_per_subframe, "cp_lengths from main");
    }

    //Initialize the cp_length for each symbols of a frame
    int cp_lengths_one_frame[Num_symbols_per_frame];
    for (int sub_frame = 0; sub_frame < 10; sub_frame++) {
        for (int symbol = 0; symbol < Num_symbols_per_subframe; symbol++) {
            cp_lengths_one_frame[Num_symbols_per_subframe * sub_frame + symbol] = cp_lengths[symbol];
        }
    }

    if (free5GRAN::display_variables) {
        free5GRAN::utils::common_utils::display_table(cp_lengths_one_frame, Num_symbols_per_frame,
                                                      "cp_lengths_one_frame");
    }
    //Initialize the symbol_size for each symbols of a frame

    int symbols_size_one_frame[Num_symbols_per_frame];
    for (int symbol = 0; symbol < Num_symbols_per_frame; symbol++) {
        symbols_size_one_frame[symbol] = free5GRAN::SIZE_IFFT_SSB + cp_lengths_one_frame[symbol];
    }

    if (free5GRAN::display_variables) {
        free5GRAN::utils::common_utils::display_table(symbols_size_one_frame, Num_symbols_per_frame,
                                                      "symbols_size_one_frame");
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



    //Fill a buffer buff_main_10ms with one_frame
    std::vector<std::complex<float>> buff_main_10ms;
    for (int symbol = 0; symbol < Num_symbols_per_frame; symbol++) {
        for (int sample = 0; sample < symbols_size_one_frame[symbol]; sample++) {
            buff_main_10ms.push_back(one_frame[symbol][sample]);
        }
    }

    if (free5GRAN::display_variables) {
        //Display buff_main_10ms
        free5GRAN::utils::common_utils::display_vector(buff_main_10ms, Num_sample_per_frame, "buff_main_10ms");
    }


    /** Sending the buff_main_10ms via USRP */
    if (run_with_usrp_10ms) {
        //Emission for SCS = 30 KHz
        BOOST_LOG_TRIVIAL(info) << "Initialize the rf parameters ";
        rf rf_variable(usrp_info_object.sampling_rate, usrp_info_object.center_frequency,
                       usrp_info_object.gain, usrp_info_object.bandwidth, usrp_info_object.subdev,
                       usrp_info_object.ant, usrp_info_object.ref2, usrp_info_object.device_args);

        std::cout << " ################ SENDING a frame of 10ms continuously, with SSB every " << ssb_period
                  << " seconds" << std::endl;

        rf_variable.buffer_transmition(buff_main_10ms);
    }



    /** Sending buff_main_10ms MULTITHREAD */

    if(run_multi_thread) {

        std::vector<std::complex<float>> buff_main_10ms_3;
        //buff_main_10ms_3 = buff_main_10ms;

        BOOST_LOG_TRIVIAL(info) << "Initialize the rf parameters ";
        rf rf_variable_2(usrp_info_object.sampling_rate, usrp_info_object.center_frequency,
                       usrp_info_object.gain, usrp_info_object.bandwidth, usrp_info_object.subdev,
                       usrp_info_object.ant, usrp_info_object.ref2, usrp_info_object.device_args);


        /** thread sending in continuous buff_main_10ms_3 */
        thread t8(send_buffer_multithread, usrp_info_object, ssb_period, rf_variable_2, &buff_main_10ms_3);
        //thread t8(rf_variable_2.buffer_transmition, buff_main_10ms_3);

        int sfn = 1;
        while (true) {

            /** If the frame_sent has an index higher than the next frame_to_send, we generate the next frame_to_send */
            if (free5GRAN::index_frame_to_send < free5GRAN::index_frame_sent + 2) {
                auto start = chrono::high_resolution_clock::now();

                buff_main_10ms = phy_variable.generate_frame_10ms(mib_object, usrp_info_object, sfn, ssb_period, pci, N, gscn,
                                                     i_b_ssb,
                                                     scaling_factor);

                BOOST_LOG_TRIVIAL(warning) << "function generate_frame_10ms done";
                BOOST_LOG_TRIVIAL(warning) << "index_frame_to_send in if = " + std::to_string(free5GRAN::index_frame_to_send);
                BOOST_LOG_TRIVIAL(warning) << "index_frame_sent in if = " + std::to_string(free5GRAN::index_frame_sent);

                free5GRAN::index_frame_to_send++;
                sfn++;

                if (sfn == 1024) {
                    sfn = 0;
                }

                buff_main_10ms_3 = buff_main_10ms;

                auto stop = chrono::high_resolution_clock::now();
                auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
                cout << duration.count() << endl;

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

