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

namespace logging = boost::log;

/** Initialize a logging file */
void init_logging(std::string level)
{
    boost::log::register_simple_formatter_factory<boost::log::trivial::severity_level, char>("Severity");
    boost::log::add_file_log
            (
                    boost::log::keywords::file_name = "free5GRAN_ben.log",
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






int main(int argc, char *argv[]) {


    bool display_variables = true; /** indicates if you want to display variables in the console */
    bool run_with_usrp = false; /** indicates if you launch the program on a computer attached to USRP. If not, put 'false' */

    free5GRAN::mib mib_object;
    usrp_info2 usrp_info_object;

    /** READING CONFIG FILE */
    libconfig::Config cfg_ben;

    try {
        /**
         * We note that the log file is correctly created only when we don't send anything on USRP */

        if(run_with_usrp){
            cfg_ben.readFile (argv[1]);   /** Use this line for CLI launch, command in /build : sudo ./NRPhy_2 ../config_ben/ssb_emission.cfg */
        }else {
            cfg_ben.readFile ("../config/ssb_emission.cfg"); /** Use this line for launch in CLion */
        }
    }

        /** Return an error if the config file is not found */
    catch (libconfig::FileIOException &e){
        std::cout<<"FileIOException occurred. Could not find the config file ssb_emission.cfg!!\n";
        return (EXIT_FAILURE);
    }

        /** Return an error if the config file contains parse error */
    catch (libconfig::ParseException &pe){
        std::cout << "Parse error at " <<pe.getFile() << " : " <<pe.getLine() << " - "<<pe.getError()<<std::endl;
        return(EXIT_FAILURE);
    }



    /** Read the level in config_file and create the log file */
    std::string level = cfg_ben.lookup("logging");
    std::cout<<"log level = "<<level<<std::endl;
    init_logging(level);

    /** Looking at the function name in config file */
    std::string func_ben = cfg_ben.lookup("function");
    const libconfig::Setting &root = cfg_ben.getRoot();


    /** Initialize variables defined in the config file */
    int gscn, pci, i_b_ssb;
    float sampling_rate;
    double frequency, ssb_period;



    if (func_ben == "SSB_EMISSION") {
        BOOST_LOG_TRIVIAL(info) << "FUNCTION DETECTED IN CONFIG FILE: SSB EMISSION";
        std::cout << "################ SSB EMISSION #################" << std::endl;
        const libconfig::Setting &mib_info = root["mib_info"];
        const libconfig::Setting &cell_info = root["cell_info"];
        const libconfig::Setting &usrp_info = root["usrp_info"];



        /** If needed, calculate frequency from gscn */
        if (cell_info.lookupValue("frequency", frequency) || cell_info.lookupValue("gscn", gscn)) {
            /** To be verified, I'm not sure */
            if (!cell_info.lookupValue("frequency", frequency)) {
                frequency = free5GRAN::phy::signal_processing::compute_freq_from_gscn(gscn);
            }

        } else {
            std::cerr << "Missing frequency or gscn parameter in config file" << std::endl;
            return(EXIT_FAILURE);
        }

        /** Fill the mib_object with the values contained in the config file */

        mib_object.sfn = mib_info.lookup("sfn"); /** stored on MIB on 10 bits */
        mib_object.pdcch_config = mib_info.lookup("pddchc_config"); /** stored on MIB on 8 bits */
        mib_object.k_ssb = mib_info.lookup("k_ssb"); /** stored on MIB on 5 bits. It indicates the number of Ressource Block between point A and SSB */
        mib_object.scs = mib_info.lookup("scs"); /** must be 15 or 30. stored on MIB on 1 bit */
        mib_object.cell_barred = mib_info.lookup("cell_barred"); /** stored on MIB on 1 bit */
        mib_object.dmrs_type_a_position = mib_info.lookup("dmrs_type_a_position"); /** stored on MIB on 1 bit */
        mib_object.intra_freq_reselection = mib_info.lookup("intra_freq_reselection"); /** stored on MIB on 1 bit */


        /** Fill the cell_info with the values contained in the config file */
        pci = cell_info.lookup("pci"); /** (Physical Cell Id). Should be between 0 and 1007 */
        i_b_ssb = cell_info.lookup("i_b_ssb"); /** i_b_ssb is the SSB index. Should be between 0 and 7. */
        ssb_period = cell_info.lookup("ssb_period"); /** in seconds */


        /** Fill the usrp_info with the values contained in the config file  */
        sampling_rate = usrp_info.lookup("sampling_rate");
        std::string device_args = usrp_info.lookup("device_args");
        usrp_info_object.device_args = device_args;

        std::string subdev = usrp_info.lookup("subdev");
        usrp_info_object.subdev = subdev;

        std::string ant = usrp_info.lookup("ant");
        usrp_info_object.ant = ant;

        std::string ref2 = usrp_info.lookup("ref2");
        usrp_info_object.ref2 = ref2;


        usrp_info_object.sample_rate = usrp_info.lookup("sample_rate");

        usrp_info_object.sampling_rate = usrp_info.lookup("sampling_rate");

        usrp_info_object.center_frequency = usrp_info.lookup("center_frequency");

        usrp_info_object.gain = usrp_info.lookup("gain");

        usrp_info_object.bandwidth = usrp_info.lookup("bandwidth");


    }   else {
        std::cout << "Please enter a function name in config file"<< std::endl;
        BOOST_LOG_TRIVIAL(error) << "couldn't recognize function's name in config file";
        return(EXIT_FAILURE);
    }

    if (display_variables) {
        /** Display MIB infos and put them in log file*/
        std::cout << "sfn = " << mib_object.sfn << std::endl;
        BOOST_LOG_TRIVIAL(info) << "sfn = " + std::to_string(mib_object.sfn);
        std::cout << "pddchc_config = " << mib_object.pdcch_config << std::endl;
        BOOST_LOG_TRIVIAL(info) << "pddchc_config = " + std::to_string(mib_object.pdcch_config);
        std::cout << "k_ssb = " << mib_object.k_ssb << std::endl;
        BOOST_LOG_TRIVIAL(info) << "k_ssb = " + std::to_string(mib_object.k_ssb);
        std::cout << "scs = " << mib_object.scs << std::endl;
        BOOST_LOG_TRIVIAL(info) << "scs = " + std::to_string(mib_object.scs);
        std::cout << "dmrs_type_a_position = " << mib_object.dmrs_type_a_position << std::endl;
        BOOST_LOG_TRIVIAL(info) << "dmrs_type_a_position = " + std::to_string(mib_object.dmrs_type_a_position);
        std::cout << "intra_freq_reselection = " << mib_object.intra_freq_reselection << std::endl;
        BOOST_LOG_TRIVIAL(info) << "intra_freq_reselection = " + std::to_string(mib_object.intra_freq_reselection);
        std::cout << "cell_barred = " << mib_object.cell_barred << std::endl;
        BOOST_LOG_TRIVIAL(info) << "cell_barred = " + std::to_string(mib_object.cell_barred);
        std::cout << "" << std::endl;
    }

    /** Display the cell_info and put them in log file */

    if (display_variables) {
        std::cout << "frequency = " << frequency << std::endl;
        std::cout << "pci = " << pci << std::endl;
        std::cout << "i_b_ssb = " << i_b_ssb << std::endl;
        std::cout << "ssb_period = " << ssb_period << std::endl;
    }

    BOOST_LOG_TRIVIAL(info) << "pci = " + std::to_string(pci);
    BOOST_LOG_TRIVIAL(info) << "i_b_ssb = " + std::to_string(i_b_ssb);
    BOOST_LOG_TRIVIAL(info) << "frequency = " + std::to_string(frequency);
    BOOST_LOG_TRIVIAL(info) << "ssb_perdiod (seconds) = " + std::to_string(ssb_period);


    /** Generate N which is the length of BCH payload after polar encode */
    int n = free5GRAN::phy::transport_channel::compute_N_polar_code(free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2, free5GRAN::SIZE_PBCH_POLAR_DECODED, 9);
    BOOST_LOG_TRIVIAL(info) << "n = " + std::to_string(n);
    int N = pow(2, n);
    BOOST_LOG_TRIVIAL(info) << "N (length of BCH payload after polar encode) = " + std::to_string(N);


    phy phy_variable;

    /** MIB GENERATION -> Generating mib_bits sequence (32 bits long in our case) from mib_object. TS38.331 V15.11.0 Section 6.2.2*/
    int mib_bits[free5GRAN::BCH_PAYLOAD_SIZE];
    //phy_variable.encode_mib(mib_object, mib_bits);
    free5GRAN::utils::common_utils::encode_mib(mib_object, mib_bits);
    BOOST_LOG_TRIVIAL(info) << "MIB GENERATION";

    if(display_variables){
        phy_variable.display_table(mib_bits, free5GRAN::BCH_PAYLOAD_SIZE, "mib_bits from main");}


    /** ENCODE BCH -> Generate rate_matched_bch (864 bits in our case) from mib_bits. TS38.212 V15.2.0 Section 5 */
    int *rate_matched_bch = new int[free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2];
    //phy_variable.encode_bch(mib_bits, pci, N, rate_matched_bch); //TO BE DELETED
    free5GRAN::phy::transport_channel::bch_encoding(mib_bits, pci, N, rate_matched_bch);

            if (display_variables) {
                free5GRAN::utils::common_utils::display_table(rate_matched_bch, free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2,
                                                              "rate_matched_bch from main");
            }
    BOOST_LOG_TRIVIAL(info) << "ENCODE BCH";

    /** ENCODE PBCH -> Generate pbch_symbols2 (432 symbols in our case) from rate_matched_bch. TS38.212 V15.2.0 Section 7.3.3.1 and 5.1.3 */
    std::complex<float> *pbch_symbols2;
    pbch_symbols2 = new std::complex<float>[free5GRAN::SIZE_SSB_PBCH_SYMBOLS];
    //phy_variable.encode_pbch_and_modulation(rate_matched_bch, pci, gscn, i_b_ssb, pbch_symbols2); //TO BE DELETED
    free5GRAN::phy::physical_channel::pbch_encoding(rate_matched_bch, pci, gscn, i_b_ssb, pbch_symbols2);
    BOOST_LOG_TRIVIAL(info) << "ENCODE PBCH";

    if (display_variables){free5GRAN::utils::common_utils::display_complex_float(pbch_symbols2, free5GRAN::SIZE_SSB_PBCH_SYMBOLS, "pbch_symbols2 from main");}

    /** GENERATE SSB -> Generate SSB_signal_time_domain (4 * 256 symbols in our case) from pbch_symbols2. TS38.211 V15.2.0 Section 7.4 */
    std::complex<float> **SSB_signal_time_domain;
    SSB_signal_time_domain = new std::complex<float> *[free5GRAN::NUM_SYMBOLS_SSB];
    SSB_signal_time_domain[0] = new std::complex<float>[free5GRAN::SIZE_IFFT_SSB];
    SSB_signal_time_domain[1] = new std::complex<float>[free5GRAN::SIZE_IFFT_SSB];
    SSB_signal_time_domain[2] = new std::complex<float>[free5GRAN::SIZE_IFFT_SSB];
    SSB_signal_time_domain[3] = new std::complex<float>[free5GRAN::SIZE_IFFT_SSB];

    phy_variable.generate_SSB_time(pbch_symbols2, pci, i_b_ssb, mib_object, SSB_signal_time_domain);
    //free5GRAN::phy::signal_processing::generate_time_domain_ssb(pbch_symbols2, pci, i_b_ssb, mib_object, SSB_signal_time_domain);
    BOOST_LOG_TRIVIAL(info) << "GENERATE SSB";



    /** COMPUTE CP. TS38.211 V15.2.0 Section 5.3 */
    int cp_lengths[28];
    int cum_sum_cp_lengths[28];
    phy_variable.compute_cp_lengths(mib_object.scs, free5GRAN::SIZE_IFFT_SSB, 0, 28, &cp_lengths[0],
                                        &cum_sum_cp_lengths[0]);
    BOOST_LOG_TRIVIAL(info) << "COMPUTE CP LENGTH";
    std::cout << "" << std::endl;
    BOOST_LOG_TRIVIAL(info) << "cp_lengths[1] (which will be used) = "+std::to_string(cp_lengths[1]);

    /** ADDING CP TO SSB -> Generate SSB_signal_time_domain_CP from SSB_signal_time_domain TS TO BE ADDED */
    std::complex<float> **SSB_signal_time_domain_CP;
    SSB_signal_time_domain_CP = new std::complex<float> *[free5GRAN::NUM_SYMBOLS_SSB];
    SSB_signal_time_domain_CP[0] = new std::complex<float>[free5GRAN::SIZE_IFFT_SSB + cp_lengths[1]];
    SSB_signal_time_domain_CP[1] = new std::complex<float>[free5GRAN::SIZE_IFFT_SSB + cp_lengths[1]];
    SSB_signal_time_domain_CP[2] = new std::complex<float>[free5GRAN::SIZE_IFFT_SSB + cp_lengths[1]];
    SSB_signal_time_domain_CP[3] = new std::complex<float>[free5GRAN::SIZE_IFFT_SSB + cp_lengths[1]];

    //phy_variable.adding_cp(SSB_signal_time_domain, free5GRAN::NUM_SYMBOLS_SSB, free5GRAN::SIZE_IFFT_SSB, cp_lengths[1], (std::complex<float> **) SSB_signal_time_domain_CP);

    free5GRAN::phy::signal_processing::adding_cp(SSB_signal_time_domain, free5GRAN::NUM_SYMBOLS_SSB, free5GRAN::SIZE_IFFT_SSB, cp_lengths[1], (std::complex<float> **) SSB_signal_time_domain_CP);

    BOOST_LOG_TRIVIAL(info) << "ADD CP (Cyclic Prefix) to the SSB (time domain)";

    if(display_variables) {
        phy_variable.display_signal_float(SSB_signal_time_domain_CP, free5GRAN::NUM_SYMBOLS_SSB,
                                              free5GRAN::SIZE_IFFT_SSB + cp_lengths[1], "SSB_signal_time_domain_CP");
    }




    /** TEST WITH FULL POWER. Generate a test signal with max power everywhere to test, via Matlab spectrogram, the emission with USRP */


    std::complex<float> **Test_signal_full_power;
    Test_signal_full_power = new std::complex<float> *[free5GRAN::NUM_SYMBOLS_SSB];
    Test_signal_full_power[0] = new std::complex<float>[free5GRAN::SIZE_IFFT_SSB + cp_lengths[1]];
    Test_signal_full_power[1] = new std::complex<float>[free5GRAN::SIZE_IFFT_SSB + cp_lengths[1]];
    Test_signal_full_power[2] = new std::complex<float>[free5GRAN::SIZE_IFFT_SSB + cp_lengths[1]];
    Test_signal_full_power[3] = new std::complex<float>[free5GRAN::SIZE_IFFT_SSB + cp_lengths[1]];

    for (int symbol = 0; symbol<4; symbol++){
        for (int sample = 0; sample < free5GRAN::SIZE_IFFT_SSB; sample++){
            //Test_signal_full_power[symbol][sample] = (30);
            Test_signal_full_power[symbol][sample] = {100,30};
        }
    }

    if (display_variables){
        for (int symbol = 0; symbol<4; symbol++){
            phy_variable.display_complex_float(Test_signal_full_power[symbol], free5GRAN::SIZE_IFFT_SSB + cp_lengths[1],"Test_signal_full_power");
        }}


    //-----------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------

    if (display_variables) {
        std::cout
                << " //-----------------------------------------------------------------------------------------------------------"
                << std::endl;
        std::cout
                << " //-----------------------------------------------------------------------------------------------------------"
                << std::endl;
    }

    /** FROM HERE, THE CODE AIMS TO DECODE INFORMATION AND NOT TO ENCODE IT ANYMORE (AYMERIC's CODE) */
    /** IT IS ONLY HERE TO VERIFY THAT WHAT WE ENCODE IS POSSIBLE TO BE DECODED CORRECTLY */

/*
    //std::vector<std::complex<float>> AY_vector_pbch_symbol = {};

    std::vector<std::complex<float>> AY_vector_pbch_symbol = {};
    std::cout << "" << std::endl;
    for (int i = 0; i < free5GRAN::SIZE_SSB_PBCH_SYMBOLS; i++) {
        AY_vector_pbch_symbol.push_back({0, 0});
    }
    if (display_variables){
        phy_variable.display_vector(AY_vector_pbch_symbol, free5GRAN::SIZE_SSB_PBCH_SYMBOLS,
                                        "AY_vector_pbch_symbol before");}

    AY_vector_pbch_symbol = phy_variable.AY_extract_pbch(SSB_signal_time_domain_CP, pci);
    if (display_variables){
        phy_variable.display_vector(AY_vector_pbch_symbol, free5GRAN::SIZE_SSB_PBCH_SYMBOLS,
                                        "AY_vector_pbch_symbol after");}


    int *bch_bits3 = new int[free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2];
    //bch_bits3 = phy_variable.AY_decode_pbch(pci, vector_pbch_symbols2);
    bch_bits3 = phy_variable.AY_decode_pbch(pci, AY_vector_pbch_symbol);

    if (display_variables){
        phy_variable.display_table(bch_bits3, 864, "AY_bch_bits3 from main");}

    int *mib_bits2 = new int[32];
    phy_variable.AY_decode_bch(bch_bits3, pci, mib_bits2);
    //phy_variable.AY_decode_bch(rate_matched_bch, pci, mib_bits2);

    if (display_variables){
        phy_variable.display_table(mib_bits2, 32, "AY_mib_bits2 from main");}

    free5GRAN::mib mib_object2;
    phy_variable.AY_decode_mib(mib_bits2, mib_object2);

    if (display_variables) {
        std::cout << "sfn = " << mib_object2.sfn << std::endl;
        std::cout << "pddchc_config = " << mib_object2.pdcch_config << std::endl;
        std::cout << "k_ssb = " << mib_object2.k_ssb << std::endl;
        std::cout << "scs = " << mib_object2.scs << std::endl;
        std::cout << "cell_barred = " << mib_object2.cell_barred << std::endl;
        std::cout << "dmrs_type_a_position = " << mib_object2.dmrs_type_a_position << std::endl;
        std::cout << "intra_freq_reselection = " << mib_object2.intra_freq_reselection << std::endl;
    }

 */
    //----------------------------------------------------------------------------------------------
    //----------------------------------------------------------------------------------------------

    /** FROM HERE, THE CODE AIMS TO SEND THE SIGNAL GENERATED VIA USRP B210 */

    /** Filling a signal SSB_signal_time_domain_CP_5ms with some 0 values
     *      to obtain a transmission of exactly ssb_period for a sampling rate at 7.68 Mhz
     */

    std::cout<< "###### SSB"<<std::endl;
    std::cout << "# ssb_period: " << ssb_period<<" second" << std::endl;
    std::cout<<"# i_b_ssb: "<<i_b_ssb<<std::endl;

    BOOST_LOG_TRIVIAL(info) << "sampling rate for USRP = "+std::to_string(sampling_rate);

    int Num_samples_per_symbol_SSB = free5GRAN::SIZE_IFFT_SSB + cp_lengths[1];
    std::cout << "# Num_samples_per_symbol_SSB: " << Num_samples_per_symbol_SSB << std::endl;
    BOOST_LOG_TRIVIAL(info) << "Num_samples_per_symbol_SSB = "+std::to_string(Num_samples_per_symbol_SSB);
    int num_symbols_SSB = 4;
    std::cout << "# num_symbols_SSB: " << num_symbols_SSB << std::endl;
    BOOST_LOG_TRIVIAL(info) << "num_symbols_SSB of SSB: "+std::to_string(num_symbols_SSB);
    float sample_duration = 1 / sampling_rate;
    std::cout << "# sample_duration (1/sampling_rate): " << sample_duration <<" second"<< std::endl;
    BOOST_LOG_TRIVIAL(info) << "sample duration: "+std::to_string(sample_duration);

    int num_of_0_to_add = 1+ (ssb_period - (num_symbols_SSB * Num_samples_per_symbol_SSB * sample_duration)) / sample_duration;
    std::cout << "# num_of_0_to_add to obtain 5 ms: " << num_of_0_to_add << std::endl;
    std::cout<<""<<std::endl;
    BOOST_LOG_TRIVIAL(info) << "number of 0 added at the end of USRP (to obtain a precise time interval) = "+std::to_string(num_of_0_to_add);



    std::complex<float> **SSB_signal_time_domain_CP_5ms;
    SSB_signal_time_domain_CP_5ms = new std::complex<float> *[num_symbols_SSB + 1];
    SSB_signal_time_domain_CP_5ms[0] = new std::complex<float>[Num_samples_per_symbol_SSB];
    SSB_signal_time_domain_CP_5ms[1] = new std::complex<float>[Num_samples_per_symbol_SSB];
    SSB_signal_time_domain_CP_5ms[2] = new std::complex<float>[Num_samples_per_symbol_SSB];
    SSB_signal_time_domain_CP_5ms[3] = new std::complex<float>[Num_samples_per_symbol_SSB];
    SSB_signal_time_domain_CP_5ms[4] = new std::complex<float>[num_of_0_to_add];

    BOOST_LOG_TRIVIAL(info) << "Fill the variable SSB_signal_time_domain_CP_5ms";
    for (int symbol = 0; symbol < num_symbols_SSB + 1; symbol++) {
        if (symbol != 4) {
            for (int sample = 0; sample < Num_samples_per_symbol_SSB; sample++) {
                /* Use this line for the generated SSB signal: */
                SSB_signal_time_domain_CP_5ms[symbol][sample] = SSB_signal_time_domain_CP[symbol][sample];

                /** Use this line for the test signal (real = 30): */
                //SSB_signal_time_domain_CP_5ms[symbol][sample] = Test_signal_full_power[symbol][sample];
            }
        }
        if (symbol == 4) {
            for (int sample = 0; sample < num_of_0_to_add; sample++) {
                //SSB_signal_time_domain_CP_5ms[symbol][sample] = {1e-3, 1e-3};
                SSB_signal_time_domain_CP_5ms[symbol][sample] = {0, 0};
                //SSB_signal_time_domain_CP_5ms[symbol][sample] = {100, 30};
            }
        }
    }


    //phy_variable.display_signal_float(SSB_signal_time_domain_CP_5ms, 5, Num_samples_per_symbol_SSB, "SSB_signal_time_domain_CP_5ms from main ");

    if (display_variables) {
        phy_variable.display_complex_float(SSB_signal_time_domain_CP_5ms[0], Num_samples_per_symbol_SSB,
                                           "SSB_5ms symbol 0 = ");
        phy_variable.display_complex_float(SSB_signal_time_domain_CP_5ms[1], Num_samples_per_symbol_SSB,
                                           "SSB_5ms symbol 1 = ");
        phy_variable.display_complex_float(SSB_signal_time_domain_CP_5ms[2], Num_samples_per_symbol_SSB,
                                           "SSB_5ms symbol 2 = ");
        phy_variable.display_complex_float(SSB_signal_time_domain_CP_5ms[3], Num_samples_per_symbol_SSB,
                                           "SSB_5ms symbol 3 = ");
        phy_variable.display_complex_float(SSB_signal_time_domain_CP_5ms[4], num_of_0_to_add,
                                               "SSB_5ms symbol 4 = ");
    }
    BOOST_LOG_TRIVIAL(info) << "USRP serial = "+usrp_info_object.device_args;
    BOOST_LOG_TRIVIAL(info) << "USRP subdev = "+usrp_info_object.subdev;
    BOOST_LOG_TRIVIAL(info) << "USRP ant = "+usrp_info_object.ant;
    BOOST_LOG_TRIVIAL(info) << "USRP ref2 = "+usrp_info_object.ref2;


    BOOST_LOG_TRIVIAL(info) << "usrp_info_object.sample_rate = "+std::to_string(usrp_info_object.sample_rate);

    //rf rf_variable(usrp_info_object.sample_rate, 3699.84e6, 55, 7.68e6, "A:A", usrp_info_object.ant, usrp_info_object.ref2, usrp_info_object.device_args);


    /** Emission for SCS = 15 KHz */
    //rf rf_variable(3.846, 3699.84e6, 75, 3.84e6, subdev, ant, ref2, device_args);


    // Create buffer
    int num_samples = 240;
    double time_first_sample = 0;


    /** Filling buff_main with normal signal */
    std::vector<std::complex<double>> buff_main;
    for (int symbol = 0; symbol < 4; symbol++) {
        for (int sample = 0; sample < Num_samples_per_symbol_SSB; sample++) {
            buff_main.push_back(SSB_signal_time_domain_CP[symbol][sample]);
        }
    }
    BOOST_LOG_TRIVIAL(info) << "Fill buff_main (used to send SSB in continuous)";


    /** Filling buff_main_5ms with signal 5ms */
    std::vector<std::complex<float>> buff_main_5ms;
    /** We note that last symbol (number 4) is not really a 'symbol' */
    for (int symbol = 0; symbol < 5; symbol++) {
        if (symbol != 4) {
            for (int sample = 0; sample < Num_samples_per_symbol_SSB; sample++) {
                buff_main_5ms.push_back(SSB_signal_time_domain_CP_5ms[symbol][sample]);
            }
        }
        if (symbol == 4) {
            for (int sample = 0; sample < num_of_0_to_add; sample++) {
                buff_main_5ms.push_back(SSB_signal_time_domain_CP_5ms[symbol][sample]);
            }
        }
    }
    BOOST_LOG_TRIVIAL(info) << "Fill buff_main_5ms (used to send SSB every 5ms only)";

    /** Filling a txt file file_ben_SSB_5ms with buff_main_5ms to verify spectogram on Python */
    std::ofstream file_ben_main_SSB_5ms;
    file_ben_main_SSB_5ms.open("file_ben_SSB_5ms.txt");

    for (int i = 0; i < Num_samples_per_symbol_SSB * 4 + num_of_0_to_add; i++) {
        file_ben_main_SSB_5ms << buff_main_5ms[i];
        file_ben_main_SSB_5ms << "\n";
    }
    file_ben_main_SSB_5ms.close();


    /** Display some useful information */
    std::cout<<"###### CELL & MIB"<<std::endl;
    std::cout<<"# Size of buff_main_5ms: "<<buff_main_5ms.size()<<std::endl;
    std::cout<<"# SFN: "<<mib_object.sfn<<std::endl;
    std::cout<<"# pddchc_config: "<<mib_object.pdcch_config<<std::endl;
    std::cout<<"# k_ssb: "<<mib_object.k_ssb<<std::endl;
    std::cout<<"# SCS: "<<mib_object.scs<<std::endl;
    std::cout<<"# cell_barred: "<<mib_object.cell_barred<<std::endl;
    std::cout<<"# dmrs_type_a_position: "<<mib_object.dmrs_type_a_position<<std::endl;
    std::cout<<"# intra_freq_reselection: "<<mib_object.intra_freq_reselection<<std::endl;
    std::cout<<"# PCI: "<<pci<<std::endl;
    std::cout<<""<<std::endl;

    /** Sending buff_main_5ms via USRP, continuously*/
    std::cout<<"###### USRP"<<std::endl;
    std::cout<<"# Sampling rate: "<<usrp_info_object.sampling_rate<<" Hz"<<std::endl;
    std::cout<<"# Bandwidth: "<<usrp_info_object.bandwidth<<" Hz"<<std::endl;
    std::cout<<"# Center frequency: "<<usrp_info_object.center_frequency<<" Hz"<<std::endl;
    std::cout<<"# Emission Gain: "<<usrp_info_object.gain<<" dB"<<std::endl;
    std::cout<<""<<std::endl;



/** Sending buff_main_5ms */
        if (run_with_usrp) {
                //Emission for SCS = 30 KHz
            BOOST_LOG_TRIVIAL(info) << "Initialize the rf parameters ";
            rf rf_variable(usrp_info_object.sample_rate, usrp_info_object.center_frequency,
                                   usrp_info_object.gain, usrp_info_object.bandwidth, usrp_info_object.subdev,
                                   usrp_info_object.ant, usrp_info_object.ref2, usrp_info_object.device_args);

            std::cout<<" ################ free5GRAN SENDING SSB EVERY "<<ssb_period <<" SECONDS ################"<<std::endl;
            rf_variable.send_from_file(buff_main_5ms);
        }








    /** Create a radio-frame of 10 ms. SSB will be placed in it in function of i_b_ssb, SCS and ssb_period */

    float SSB_duration = num_symbols_SSB * Num_samples_per_symbol_SSB * sample_duration;
    std::cout<<"SSB_duration = "<<SSB_duration<<std::endl;

    int Num_sample_per_frame = (1e-2)*sampling_rate; /** This corresponds to divide the frame duration (10 ms) by the sample_duration. */
    std::cout<<"sample_duration= "<<(sample_duration)<<std::endl;
    std::cout<<"Num_sample_per_frame = "<<(Num_sample_per_frame)<<std::endl;

    int Num_symbols_per_frame = 280; /** To be replaced by a variable */
    std::cout<<"Num_symbols_per_frame = "<<Num_symbols_per_frame<<std::endl;

    std::complex<float> *one_frame;
    one_frame = new std::complex<float>[Num_sample_per_frame];

    /** Initialize all the frame at 0 */
    for (int sample = 0; sample < Num_sample_per_frame; sample++){
        one_frame[sample] = {0,0};
    }

    /** Create a buffer for 10 ms */

    std::vector<std::complex<float>> buff_main_10ms;
    for (int sample = 0; sample < Num_sample_per_frame; sample++){
        buff_main_10ms.push_back(one_frame[sample]);
    }


    int index_first_ssb_in_frame = free5GRAN::BAND_N_78.ssb_symbols[i_b_ssb];
    std::cout<<"index_first_ssb_in_frame = "<<index_first_ssb_in_frame<<std::endl;


    phy_variable.display_table(cp_lengths, 28, "cp_lengths from main_ben");

    /** Initialize the cp_length for each symbols of a frame */
    int cp_lengths_one_frame[Num_symbols_per_frame];
    for (int sub_frame = 0; sub_frame < 10; sub_frame++){
        for (int symbol = 0; symbol < 28; symbol++){
            cp_lengths_one_frame[28*sub_frame+symbol] = cp_lengths[symbol];
        }
    }

    phy_variable.display_table(cp_lengths_one_frame, Num_symbols_per_frame, "cp_lengths_one_frame");

    /** Initialize the symbol_size for each symbols of a frame */

    int symbols_size_one_frame[Num_symbols_per_frame];
    for (int symbol = 0; symbol < Num_symbols_per_frame; symbol ++){
        symbols_size_one_frame[symbol] = free5GRAN::SIZE_IFFT_SSB + cp_lengths_one_frame[symbol];
    }
    phy_variable.display_table(symbols_size_one_frame, Num_symbols_per_frame, "symbols_size_one_frame");

    int ssb_period_in_samples = ssb_period * sampling_rate;
    std::cout<<"ssb_period_in_samples = "<<ssb_period_in_samples;

    /** Initialize the cumulative_symbol_size for each symbols of a frame
     * cumulative_symbol_size[i] corresponds to the total number of samples in the frame at the ith symbol.
     */



    /** Initialyse one_frame_2 with the right number of sample for each sumbol */

    std::complex<float> **one_frame_2;
    one_frame_2 = new std::complex<float> *[Num_symbols_per_frame];

    for (int symbol = 0; symbol < Num_symbols_per_frame; symbol ++){
        one_frame_2[symbol] = new std::complex<float>[symbols_size_one_frame[symbol]];
    }


    /** Fill one_frame_2 with 0 values everywhere */

    for (int symbol = 0; symbol < Num_symbols_per_frame; symbol++){
        for (int sample = 0; sample < symbols_size_one_frame[symbol]; sample ++){
            one_frame_2[symbol][sample] = {0,0};
        }
    }

    /** Fill one_frame_2 with SSB at the right place */

    int index_ssb_in_frame = index_first_ssb_in_frame;
    for (int symbol_SSB_index = 0; symbol_SSB_index < free5GRAN::NUM_SYMBOLS_SSB; symbol_SSB_index++) {
        one_frame_2[index_ssb_in_frame] = SSB_signal_time_domain_CP[symbol_SSB_index];
        index_ssb_in_frame ++;
    }


    /** Display one_frame_2

    for (int symbol = 0; symbol < Num_symbols_per_frame; symbol++){
        phy_variable.display_complex_float(one_frame_2[symbol], symbols_size_one_frame[symbol], "one_frame_2, one symbol = ");
    }
     */





    /** Sending the buffer_10ms via USRP

    if (run_with_usrp) {
        //Emission for SCS = 30 KHz
        BOOST_LOG_TRIVIAL(info) << "Initialize the rf parameters ";
        rf rf_variable(usrp_info_object.sample_rate, usrp_info_object.center_frequency,
                               usrp_info_object.gain, usrp_info_object.bandwidth, usrp_info_object.subdev,
                               usrp_info_object.ant, usrp_info_object.ref2, usrp_info_object.device_args);

        std::cout<<" ################ SENDING a frame of 10ms continuously"<<std::endl;
        rf_variable.send_from_file(buff_main_10ms);
    }
*/




}
