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

#include "phy/phy.h"
#include "rf/rf.h"
#include <complex>
#include <vector>
#include <boost/format.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <thread>
#include "../lib/phy/libphy/libphy.h"
#include "../lib/utils/common_utils/common_utils.h"
#include "../lib/phy/physical_channel/physical_channel.h"
#include "../lib/variables/common_variables/common_variables.h"
#include "../lib/utils/sequence_generator/sequence_generator.h"


void send_buffer_multithread(rf rf_variable_2, vector<complex<float>> * buff_generated1, vector<complex<float>> * buff_generated2){
    /** This function will run continuously to send the 2 buffers alternatively and is called by thread 'sending' */
    BOOST_LOG_TRIVIAL(warning) << "MAIN Function send_buffer_multithread begins ";
    rf_variable_2.buffer_transmition(*buff_generated1, *buff_generated2);
}


void generate_buffer_multithread(phy phy_object){
    /** This function will run continuously to generate the 2 buffers alternatively and is called by thread 'generate' */
    BOOST_LOG_TRIVIAL(warning) << "MAIN Function generate_buffer_multithread begins ";
    phy_object.continuous_buffer_generation();
}


int main(int argc, char *argv[]) {

    bool run_with_usrp = false; /** put 'true' if running_platform is attached to an USRP */
    bool run_one_time_generate_frame = true; /** put 'true' for running one time function 'generate_frame' and display result */

    /** Depending on the running platform, select the right config file */
    const char *config_file;
    if (run_with_usrp == true) {
        config_file = argv[1]; // To launch on Linux CLI, run: sudo ./free5GRAN-gNodeB ../config/ssb_emission.cfg
    }
    if (run_with_usrp == false) {
        config_file = ("../config/ssb_emission.cfg");
    }

    /** Read Config File with function read_config_gNodeB */
    free5GRAN::utils::common_utils::read_config_gNodeB(config_file);

    /** Initialize log file with log_level */
    free5GRAN::utils::common_utils::init_logging(free5GRAN::gnodeB_config_globale.log_level);
    BOOST_LOG_TRIVIAL(warning) << "Log file should have been initialized";

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

    /** Calculate sampling_rate & bandwidth */
    usrp_info_object.sampling_rate = free5GRAN::SIZE_IFFT_SSB * mib_object.scs;
    usrp_info_object.bandwidth = usrp_info_object.sampling_rate;

    /** Calculate number of sample that a frame (10 ms) will contain (= sampling_rate / 100) */
    int num_samples_in_frame;
    free5GRAN::utils::common_utils::compute_num_sample_per_frame(mib_object, num_samples_in_frame);

    /** Calculate number of symbols that a frame (10 ms) will contain */
    int Num_symbols_per_subframe;
    if (mib_object.scs == 15000) {
        Num_symbols_per_subframe = 14;
    } else if (mib_object.scs == 30000) {
        Num_symbols_per_subframe = 28;
    }
    free5GRAN::num_symbols_frame = Num_symbols_per_subframe * 10;

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

    /** instantiate a phy object */
    phy phy_object(mib_object, cp_lengths_one_frame, cum_sum_cp_lengths, free5GRAN::SIZE_IFFT_SSB, num_samples_in_frame);


    BOOST_LOG_TRIVIAL(info) << "pddchc_config = " + std::to_string(mib_object.pdcch_config);
    BOOST_LOG_TRIVIAL(info) << "k_ssb = " + std::to_string(mib_object.k_ssb);
    BOOST_LOG_TRIVIAL(info) << "scs = " + std::to_string(mib_object.scs);
    BOOST_LOG_TRIVIAL(info) << "dmrs_type_a_position = " + std::to_string(mib_object.dmrs_type_a_position);
    BOOST_LOG_TRIVIAL(info) << "intra_freq_reselection = " + std::to_string(mib_object.intra_freq_reselection);
    BOOST_LOG_TRIVIAL(info) << "cell_barred = " + std::to_string(mib_object.cell_barred);
    BOOST_LOG_TRIVIAL(info) << "pci = " + std::to_string(free5GRAN::gnodeB_config_globale.pci);
    BOOST_LOG_TRIVIAL(info) << "i_b_ssb = " + std::to_string(free5GRAN::gnodeB_config_globale.i_b_ssb);
    BOOST_LOG_TRIVIAL(info) << "ssb_perdiod (seconds) = " + std::to_string(free5GRAN::gnodeB_config_globale.ssb_period);
    BOOST_LOG_TRIVIAL(info) << "sampling rate for USRP = " + std::to_string(usrp_info_object.sampling_rate);
    BOOST_LOG_TRIVIAL(info) << "USRP serial = " + usrp_info_object.device_args;
    BOOST_LOG_TRIVIAL(info) << "USRP subdev = " + usrp_info_object.subdev;
    BOOST_LOG_TRIVIAL(info) << "USRP ant = " + usrp_info_object.ant;
    BOOST_LOG_TRIVIAL(info) << "USRP ref2 = " + usrp_info_object.ref2;
    BOOST_LOG_TRIVIAL(info) << "USRP bandwidth = " + std::to_string(usrp_info_object.bandwidth);
    BOOST_LOG_TRIVIAL(info) << "num_samples_in_frame = " + std::to_string(num_samples_in_frame);
    BOOST_LOG_TRIVIAL(info) << "free5GRAN::num_symbols_frame = " + std::to_string(free5GRAN::num_symbols_frame);

    /** Display some useful informations in console */
    std::cout << "\n###### RADIO" << std::endl;
    std::cout << "# num_samples_in_frame = " << num_samples_in_frame << std::endl;
    std::cout << "# num_symbols_in_frame = " << free5GRAN::num_symbols_frame << std::endl;
    std::cout << "# Frequency: " << usrp_info_object.center_frequency / 1e6 << " MHz" << std::endl;
    std::cout << "# ifft Size: " << free5GRAN::SIZE_IFFT_SSB << std::endl;
    std::cout << "# ssb_period: " << free5GRAN::gnodeB_config_globale.ssb_period << " second" << std::endl;
    std::cout << "# num_samples_in_frame = " << num_samples_in_frame << std::endl;
    std::cout << "\n###### CELL" << std::endl;
    std::cout << "# PCI: " << free5GRAN::gnodeB_config_globale.pci << std::endl;
    std::cout << "# I_B_SSB: " << free5GRAN::gnodeB_config_globale.i_b_ssb << std::endl;
    std::cout << "\n###### MIB" << std::endl;
    std::cout << "# Frame number: varies cyclically between 0 and 1023" << std::endl;
    std::cout << "# PDCCH configuration: " << mib_object.pdcch_config << std::endl;
    std::cout << "# SCS: " << mib_object.scs / 1e3 << " kHz" << std::endl;
    std::cout << "# cell_barred: " << mib_object.cell_barred << std::endl;
    std::cout << "# DMRS type A position: " << mib_object.dmrs_type_a_position << std::endl;
    std::cout << "# k SSB: " << mib_object.k_ssb << std::endl;
    std::cout << "# Intra freq reselection: " << mib_object.intra_freq_reselection << std::endl;
    std::cout << "\n###### DCI" << std::endl;
    std::cout << "# RIV: " << free5GRAN::gnodeB_config_globale.dci_object.RIV<< std::endl;
    std::cout << "# Time Domain RA: " << free5GRAN::gnodeB_config_globale.dci_object.TD_ra<< std::endl;
    std::cout << "# vrb_prb_interleaving (0 = non-interleaved): " << free5GRAN::gnodeB_config_globale.dci_object.vrb_prb_interleaving<< std::endl;
    std::cout << "# Modulation coding scheme: " << free5GRAN::gnodeB_config_globale.dci_object.mcs<< std::endl;
    std::cout << "# Redudancy version: " << free5GRAN::gnodeB_config_globale.dci_object.rv<< std::endl;
    std::cout << "# System information (0 = SIB1): " << free5GRAN::gnodeB_config_globale.dci_object.si<< std::endl;
    std::cout << "\n###### USRP" << std::endl;
    std::cout << "# Sampling rate: " << usrp_info_object.sampling_rate / 1e6 << " MHz" << std::endl;
    std::cout << "# Bandwidth: " << usrp_info_object.bandwidth / 1e6 << " MHz" << std::endl;
    std::cout << "# Emission Gain: " << usrp_info_object.gain << " dB" << std::endl;
    std::cout << "# Scaling factor = " << free5GRAN::gnodeB_config_globale.scaling_factor <<"\n"<< std::endl;



    /** Run generate_frame one time just for testing */
    if (run_with_usrp == false && run_one_time_generate_frame == true) {


        int sfn = 563;
        int num_SSB_in_next_frame;

        /** Calculate the number of ssb block that the next frame will contain */
        phy_object.compute_num_SSB_in_frame(free5GRAN::gnodeB_config_globale.ssb_period, sfn, num_SSB_in_next_frame);


        std::vector<std::complex<float>> buff_main_10ms(num_samples_in_frame);

        /** */
        phy_object.generate_frame(num_SSB_in_next_frame, free5GRAN::num_symbols_frame, sfn,
                                  free5GRAN::gnodeB_config_globale.pci,
                                  free5GRAN::gnodeB_config_globale.i_b_ssb,
                                  free5GRAN::gnodeB_config_globale.scaling_factor, buff_main_10ms);

        free5GRAN::utils::common_utils::display_vector_per_symbols(buff_main_10ms, free5GRAN::num_symbols_frame,
                                                                   "\n\nbuff_main_10ms from main");
    }


    /** Sending and generate buffers MULTITHREADING */
    if (run_with_usrp == true) {

        /** Initialize the rf (USRP B210) parameters */
        rf rf_variable_2(usrp_info_object.sampling_rate, usrp_info_object.center_frequency,
                         usrp_info_object.gain, usrp_info_object.bandwidth, usrp_info_object.subdev,
                         usrp_info_object.ant, usrp_info_object.ref2, usrp_info_object.device_args);
        BOOST_LOG_TRIVIAL(info) << "Initialize the rf parameters done";


        /** Initialize Semaphore to manage multithread */
        sem_init(&free5GRAN::semaphore_common1, 1, 0);
        sem_init(&free5GRAN::semaphore_common2, 1, 0);


        /** launch thread 'sending' which will run continuously */
        std::cout << "\nSending Frame indefinitely..." << std::endl;
        thread sending(send_buffer_multithread, rf_variable_2, &free5GRAN::buffer_generated1,
                       &free5GRAN::buffer_generated2);


        /** launch thread 'generate' which will run continuously */
        std::cout << "\nGenerating Frame indefinitely..." << std::endl;
        thread generate(generate_buffer_multithread, phy_object);


        sending.join();
        generate.join();
    }
}
