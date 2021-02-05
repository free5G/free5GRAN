/*
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
 */

#include <cmath>
#include "common_utils.h"

#include <iostream>

/** THE INCLUDE BELOW ARE ADDITION FROM BENOIT. BE CAREFUL WHEN MERGING */

#include "../../utils/sequence_generator/sequence_generator.h"
#include "../../variables/common_matrices/common_matrices.h"
#include "../../phy/libphy/libphy.h"

#include <fftw3.h>
#include <complex>
#include <vector>
#include "../../utils/sequence_generator/sequence_generator.h"
#include "../../variables/common_variables/common_variables.h"
#include "../../utils/common_utils/common_utils.h"
#include "../../phy/libphy/libphy.h"

#include_next <math.h>
#include <fstream>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <libconfig.h++>


void free5GRAN::utils::common_utils::parse_mib(int *mib_bits, free5GRAN::mib &mib_object) {
    /**
     * \fn parse_mib
     * \brief MIB parser
     * \standard TS 38.331 V15.3.0 Section 6.2
     *
     * \param[in] mib_bits: Input bits sequence
     * \param[out] mib_object: Output MIB object
     */
    /*
     * Getting MIB informations
    */
    // Recovering SFN from BCH payload
    int sfn_array[10] = {mib_bits[1], mib_bits[2], mib_bits[3], mib_bits[4], mib_bits[5], mib_bits[6],mib_bits[24], mib_bits[25], mib_bits[26], mib_bits[27]};
    mib_object.sfn = 0;
    for (int i = 0 ; i < 10; i ++){
        mib_object.sfn += sfn_array[i] * pow(2, 10 - 1 - i);
    }

    int available_scs[2] = {15, 30};
    // Recovering PDCCH config identifier
    int pdcch_config_array[8];
    for (int i = 0 ; i < 8; i ++){
        pdcch_config_array[i] = mib_bits[13 + i];
    }
    mib_object.pdcch_config = 0;
    for (int i = 0 ; i < 8; i ++){
        mib_object.pdcch_config += pdcch_config_array[i] * pow(2, 8 - 1 - i);
    }
    // Recovering cell barred information
    mib_object.cell_barred = mib_bits[21];
    mib_object.scs = available_scs[mib_bits[7]];

    int k_ssb_array[5] = {mib_bits[29], mib_bits[8], mib_bits[9], mib_bits[10], mib_bits[11]};
    mib_object.k_ssb = 0;
    for (int i = 0 ; i < 5; i ++){
        mib_object.k_ssb += k_ssb_array[i] * pow(2, 5 - 1 - i);
    }
    mib_object.dmrs_type_a_position = 2 + mib_bits[12];
    mib_object.intra_freq_reselection = mib_bits[22];
}

/*
 * Scrambling implementation: output = (input + c_seq)[2]
 */
void free5GRAN::utils::common_utils::scramble(int *input_bits, int *c_seq, int *output_bits, int length, int offset) {
    /**
     * \fn scramble
     * \brief Hard bits scrambling
     * \param[in] input_bits: Input bits sequence
     * \param[in] c_seq: Scrambling sequence
     * \param[out] output_bits: Output sequence
     * \param[in] length: Input bits sequence length
     * \param[in] offset: Scrambling offset
     */
    for (int i = 0; i < length; i ++){
        output_bits[i] = (input_bits[i] + c_seq[i + offset]) % 2;
    }
}

void free5GRAN::utils::common_utils::scramble(double *input_bits, int *c_seq, double *output_bits, int length, int offset) {
    /**
     * \fn scramble
     * \brief Soft bits scrambling
     * \param[in] input_bits: Input bits sequence
     * \param[in] c_seq: Scrambling sequence
     * \param[out] output_bits: Output sequence
     * \param[in] length: Input bits sequence length
     * \param[in] offset: Scrambling offset
     */
    for (int i = 0; i < length; i ++){
        output_bits[i] = input_bits[i] * c_seq[i + offset];
    }
}











/** FROM HERE, IT'S ADDITION FROM BENOIT. BE CAREFUL WHEN MERGING ! */


void free5GRAN::utils::common_utils::encode_mib(free5GRAN::mib mib_object, int *mib_bits) {
    /**
    * \fn encode_mib (free5GRAN::mib mib_object, int* mib_bits)
    * \brief Transforms the MIB informations (decimal) into the mib bits sequence.
    * In our case, mib bits sequence is 32 bits long.
    * \standard TS38.331 V15.11.0 Section 6.2.2
    *
    * \param[in] mib_object object MIB created in common_structures.h, including sfn, scs, cell_barred...
    * \param[out] mib_bits bit sequence returned by the function.
    */

    /** Following mib_bits are unused and set to 0 according to TS38.331 V15.11.0 Section 6.2.2 */
    int unused_bits_size = 5;
    for (int i = 0; i < unused_bits_size; i++) {
        mib_bits[free5GRAN::INDEX_UNUSED_BITS_IN_MIB[i]] = 0;
    }

    int sfn_binary_size = 10;
    int sfn_binary[sfn_binary_size];
    convert_decimal_to_binary(sfn_binary_size, mib_object.sfn, sfn_binary);

    /** Put sfn bits into mib_bits sequence according to TS38.331 V15.11.0 Section 6.2.2 */
    for (int i = 0; i < sfn_binary_size; i++) {
        mib_bits[free5GRAN::INDEX_SFN_BITS_IN_MIB[i]] = sfn_binary[i];
    }

    int pddchc_config_binary_size = 8;
    int pddchc_config_binary[pddchc_config_binary_size];
    convert_decimal_to_binary(pddchc_config_binary_size, mib_object.pdcch_config, pddchc_config_binary);

    /** Put the pddchc_config bits into mib_bits sequence according to TS38.331 V15.11.0 Section 6.2.2 */
    for (int i = 0; i < pddchc_config_binary_size; i++) {
        mib_bits[free5GRAN::INDEX_PDDCHC_CONFIG_BITS_IN_MIB[i]] = pddchc_config_binary[i];
    }

    int k_ssb_binary_size = 5;
    int k_ssb_binary[k_ssb_binary_size];
    convert_decimal_to_binary(k_ssb_binary_size, mib_object.k_ssb, k_ssb_binary);

    /** Put the k_ssb bits into mib_bits sequence according to TS38.331 V15.11.0 Section 6.2.2 */
    for (int i = 0; i < k_ssb_binary_size; i++) {
        mib_bits[free5GRAN::INDEX_K_SSB_BITS_IN_MIB[i]] = k_ssb_binary[i];
    }


    /** Following mib information are not needed to be converted into bits as they are stored on only 1 bit */

    int available_scs[2] = {15000, 30000}; /** In FR1, SCS = 15 kHz or 30 kHz. In FR2, SCS = 30 kHz or 60 kHz */
    for (int i = 0; i < 2; i++) {
        /** Put SCS (Sub Carrier Spacing) bit into mib_bits sequence, according to TS38.331 V15.11.0 Section 6.2.2 */
        if (mib_object.scs == available_scs[i]) {
            mib_bits[free5GRAN::INDEX_AVAILABLE_SCS_IN_MIB[0]] = i;
        }
    }


    /** Put the cell_barred bit into mib_bits sequence, according to TS38.331 V15.11.0 Section 6.2.2 */
    mib_bits[free5GRAN::INDEX_CELL_BARRED_BITS_IN_MIB[0]] = mib_object.cell_barred;

    /** Put the DMRS type A position (position of first DM-RS for downlink) bit into mib_bits sequence, according to TS38.331 V15.11.0 Section 6.2.2 */
    mib_bits[free5GRAN::INDEX_DMRS_TYPE_A_POSITION_BITS_IN_MIB[0]] = mib_object.dmrs_type_a_position - 2;

    /** Put the intra frequency reselection bit into mib_bits sequence, according to TS38.331 V15.11.0 Section 6.2.2 */
    mib_bits[free5GRAN::INDEX_INTRA_FREQ_RESELECTION_BITS_IN_MIB[0]] = mib_object.intra_freq_reselection;
    BOOST_LOG_TRIVIAL(info) << "function encode_mib done";

}



void free5GRAN::utils::common_utils::convert_decimal_to_binary(int size, int decimal, int *table_output) {
    /**
    * \fn convert_decimal_to_binary (int size, int decimal, int* table_output)
    * \brief Converts an integer number into a binary bit sequence.
    *
    * \param[in] size Indicates number of bits in the output sequence. Please verify that decimal <= 2^size.
    * \param[in] decimal Number to convert into a bit sequence.
    * \param[out] table_output Output bits sequence. Will contain only 1 and 0.
    */

    for (int i = size; i >= 0; i--) {
        if (decimal >= std::pow(2, i-1)) {
            table_output[size-i] = 1;
            decimal = decimal - std::pow(2, i-1);
        } else {
            table_output[size-i] = 0;
        }
    }
}


void free5GRAN::utils::common_utils::display_signal_float(std::complex<float> **signal_to_display, int num_symbols,
                                                          int num_sc, char *signal_name){
    /**
    * \fn display_signal_float (std::complex<float> **signal_to_display, int num_symbols, int num_sc, char *signal_name)
    * \brief Displayw a table of complex<float> in the console
    * \param[in] signal_to_display
    * \param[in] num_symbols number of symbol in the table
    * \param[in] num_sc number of subcarrier in each symbol
    * \param[in] signal_name name to display
    */
    for (int symbols = 0; symbols < num_symbols; symbols++){
        std::cout<<""<<std::endl;
        std::cout<<""<<std::endl;
        std::cout<<signal_name<< "of symbol "<<symbols<<" = "<<std::ends;
        display_complex_float(signal_to_display[symbols], num_sc, "");
    }
}


void free5GRAN::utils::common_utils::display_vector(std::vector<std::complex<float>> vector_to_display, int vector_size,
                                                    char *vector_name){
    /**
    * \fn display_vector (std::vector<std::complex<float>> *vector_to_display, int vector_size, char* vector_name)
    * \brief Displays a vector in the console.
    *
    * \param[in] vector_to_display
    * \param[in] vector_size number of element in the vector
    * \param[in] vector_name name to display
    */
    for (int i=0; i<vector_size; i++){
        if (i==0){
            std::cout <<""<< std::endl;
            std::cout<< vector_name << " (of size "<< vector_size<<") = "<<std::ends;
        }
        if (i% 274 == 0){         /** 10 here means that every 280 elements displayed, a line break is done */
            std::cout <<""<< std::endl;
        }
        std::cout<<vector_to_display[i]<<"  "<< std::ends;
    }
}


void free5GRAN::utils::common_utils::display_complex_double(std::complex<double> *vector_to_display, int vector_size,
                                                            char *vector_name){
    /**
    * \fn display_complex_double (std::complex<double> *vector_to_display, int vector_size, char* vector_name)
    * \brief Displays a vector in the console.
    *
    * \param[in] vector_to_display
    * \param[in] vector_size number of element in the vector
    * \param[in] vector_name name to display
    */
    for (int i=0; i<vector_size; i++){
        if (i % 10 == 0){        /** 10 here means that every 10 elements displayed, a line break is done*/
            std::cout <<""<< std::endl;
        }
        if (i == 0){
            std::cout << +vector_name << ": "<<std::ends;
        }
        std::cout<<vector_to_display[i] <<"   "<< std::ends;
    }
}


void free5GRAN::utils::common_utils::display_complex_float(std::complex<float> *vector_to_display, int vector_size,
                                                           char *vector_name){
    /**
    * \fn display_complex_float (std::complex<float> *vector_to_display, int vector_size, char* vector_name)
    * \brief Displays a vector in the console.
    *
    * \param[in] vector_to_display
    * \param[in] vector_size number of element in the vector
    * \param[in] vector_name name to display
    */
    for (int i=0; i<vector_size; i++){
        if (i % 274 == 0 && i !=0){        /** 10 here means that every 10 elements displayed, a line break is done */
            std::cout <<""<< std::endl;
        }
        if (i == 0){
            std::cout << +vector_name << ": "<<std::ends;
        }
        std::cout<<vector_to_display[i] <<"   "<< std::ends;
    }
}


void free5GRAN::utils::common_utils::display_table(int *table_to_display, int size, char *table_name) {
   /**
   * \fn display_table (int* table_to_display, int size, char* table_name)
   * \brief Displays a table in the console.
   *
   * \param[in] table_to_display
   * \param[in] size number of element in the table
   * \param[in] table_name name to display
   */

    for (int i = 0; i<size; i++) {
        if (i % 70== 0){        /** 70 here means that every 70 elements displayed, a line break is done */
            std::cout <<""<< std::endl;
        }
        if (i == 0){
            std::cout << +table_name << ": "<<std::ends;
        }
        std::cout<<table_to_display[i] <<" "<< std::ends;
    }
    std::cout <<""<< std::endl;
}









/** Initialize a logging file
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
    //boost::log::add_common_attributes();
}
*/









void free5GRAN::utils::common_utils::read_config_gNodeB(free5GRAN::gNodeB_config gNodeBConfig, char *argv[]) {

    bool run_multi_thread = true;

    namespace logging = boost::log;
    void init_logging(string warning); /** 'warning' has to be deleted */
    libconfig::Config cfg_gNodeB_lib;

    /** READING CONFIG FILE */
    libconfig::Config cfg_gNodeB_Lib;
    try {
        if (run_multi_thread) {
            cfg_gNodeB_Lib.readFile(
                    argv[1]);   /** Use this for CLI launch. command in /build : sudo ./NRPhy_2 ../config/ssb_emission.cfg */
        } else {
            cfg_gNodeB_Lib.readFile("../config/ssb_emission.cfg"); /** Use this for launch in CLion */
        }
    }

    /** Return an error if config file is not found */
    catch (libconfig::FileIOException &e) {
        std::cout << "FileIOException occurred. Could not find the config file ssb_emission.cfg!!\n";
        // return (EXIT_FAILURE);
    }

    catch (libconfig::ParseException &pe) {
        std::cout << "Parse error at " << pe.getFile() << " : " << pe.getLine() << " - " << pe.getError() << std::endl;
        //return (EXIT_FAILURE);
    }

    /** Read 'level' in config_file and create the log file */
    std::string log_level = cfg_gNodeB_Lib.lookup("logging");
    gNodeBConfig.log_level = log_level;
    std::cout << "log level = " << log_level << std::endl;
    //init_logging(log_level);

    /** Read 'display_variables' in config_file */
    free5GRAN::display_variables = cfg_gNodeB_Lib.lookup("display_variables");

    /** Look at function's name in config file */
    std::string func_gNodeB = cfg_gNodeB_Lib.lookup("function");
    const libconfig::Setting &root = cfg_gNodeB_Lib.getRoot();

    /** Initialize variables defined in the config file */
    int gscn, pci, i_b_ssb;
    float scaling_factor;
    double ssb_period;

    //----------------------------------------------------------------------------------------

    if (func_gNodeB == "SSB_EMISSION") {
        BOOST_LOG_TRIVIAL(info) << "FUNCTION DETECTED IN CONFIG FILE: SSB EMISSION";

        const libconfig::Setting &mib_info = root["mib_info"], &cell_info = root["cell_info"], &usrp_info = root["usrp_info"];

        /** Fill usrp_info_object with values contained in config file  */
        std::string device_args = usrp_info.lookup("device_args");
        gNodeBConfig.device_args = device_args;
        std::string subdev = usrp_info.lookup("subdev");
        gNodeBConfig.subdev = subdev;
        std::string ant = usrp_info.lookup("ant");
        gNodeBConfig.ant = ant;
        std::string ref2 = usrp_info.lookup("ref2");
        gNodeBConfig.ref2 = ref2;
        gNodeBConfig.center_frequency = usrp_info.lookup("center_frequency");
        gNodeBConfig.gain = usrp_info.lookup("gain");
        scaling_factor = usrp_info.lookup(
                "scaling_factor"); /** Multiplying factor (before ifft) to enhance the radio transmission */

        /** Calculate scs (sub-carrier spacing) in function of center_frequency. scs is stored on MIB on 1 bit */
        /** Calculation according to !! TS TO BE ADDED !! */
        if (gNodeBConfig.center_frequency < 3000e6) {
            gNodeBConfig.scs = 15e3; /** in Hz */
        } else {
            gNodeBConfig.scs = 30e3; /** in Hz */
        }

        gNodeBConfig.sampling_rate = free5GRAN::SIZE_IFFT_SSB * gNodeBConfig.scs;
        gNodeBConfig.bandwidth = gNodeBConfig.sampling_rate;

        /** Fill mib_object with values in config file */
        gNodeBConfig.pddchc_config = mib_info.lookup("pddchc_config"); /** stored on MIB on 8 bits */
        gNodeBConfig.k_ssb = mib_info.lookup(
                "k_ssb"); /** stored on MIB on 5 bits. Number of Ressource Blocks between point A and SSB */
        gNodeBConfig.cell_barred = mib_info.lookup("cell_barred"); /** stored on MIB on 1 bit */
        gNodeBConfig.dmrs_type_a_position = mib_info.lookup("dmrs_type_a_position"); /** stored on MIB on 1 bit */
        gNodeBConfig.intra_freq_reselection = mib_info.lookup("intra_freq_reselection"); /** stored on MIB on 1 bit */

        /** Fill cell_info with values contained in config file */
        pci = cell_info.lookup("pci"); /** (Physical Cell Id). int between 0 and 1007 */
        i_b_ssb = cell_info.lookup("i_b_ssb"); /** SSB index. int between 0 and 7. */
        ssb_period = cell_info.lookup("ssb_period"); /** in seconds */
    } else {
        std::cout << "Please enter a function name in config file" << std::endl;
        BOOST_LOG_TRIVIAL(error) << "couldn't recognize function's name in config file";
        //return (EXIT_FAILURE);
    }
}





