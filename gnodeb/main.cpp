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
#include "../lib/phy/libphy/libphy.h"
#include "../lib/utils/common_utils/common_utils.h"
#include "../lib/phy/physical_channel/physical_channel.h"
#include "../lib/variables/common_variables/common_variables.h"

/** This function will run continuously to send frames and is called by thread 'sending' */
void send_buffer_multithread(rf rf_variable_2, vector<complex<float>> * buff_to_send){
    BOOST_LOG_TRIVIAL(warning) << "Function send_buffer_multithread begins ";
    rf_variable_2.buffer_transmition(*buff_to_send);
}

int main(int argc, char *argv[]) {

    bool run_with_usrp = false; /** put 'true' if running_platform is attached to an USRP */
    bool run_one_time_ssb = false; /** put 'true' for running one time function 'generate_frame' and display result */
    bool run_test_dci = true;

    phy phy_variable_main;
    phy_variable_main.reduce_main(run_with_usrp, run_one_time_ssb, argv);






    /** Below is under construction (DCI - PDCCH) */

    if (run_test_dci == true) {

        free5GRAN::pdcch_t0ss_monitoring_occasions pdcch_ss_mon_occ;
        pdcch_ss_mon_occ.n_rb_coreset = 48;
        int freq_domain_ra_size, K;
        freq_domain_ra_size = ceil(log2(pdcch_ss_mon_occ.n_rb_coreset * (pdcch_ss_mon_occ.n_rb_coreset + 1) / 2));
        std::cout<< "freq_domain_ra_size = "<<freq_domain_ra_size<<std::endl;

        free5GRAN::dci_1_0_si_rnti dci_object;

        dci_object.RIV = 329;
        dci_object.TD_ra = 0;
        dci_object.vrb_prb_interleaving = 0;
        dci_object.mcs = 6;
        dci_object.rv = 3;
        dci_object.si = 0;

        K = freq_domain_ra_size +4+1+5+2+1+15+24;
        //K = freq_domain_ra_size +4+1+5+2+1;
        int dci_bits[freq_domain_ra_size +4+1+5+2+1+15];
        std::cout<<"freq_domain_ra_size = "<<freq_domain_ra_size<<std::endl;
        std::cout<<"K = "<<K<<std::endl;
        phy_variable_main.encode_dci(dci_object, dci_bits, freq_domain_ra_size);
        free5GRAN::utils::common_utils::display_table(dci_bits, freq_domain_ra_size +4+1+5+2+1+15, "dci_bits from main");


        int length_crc = 24;
        int dci_bits_with_crc[K];
        phy_variable_main.adding_dci_crc(dci_bits, dci_bits_with_crc, free5GRAN::G_CRC_24_C, K, length_crc + 1, free5GRAN::SI_RNTI);

        free5GRAN::utils::common_utils::display_table(dci_bits_with_crc, K, "dci_bits_with_crc");

        /** Polar encode */
        int n = 9;
        int N = pow(2, n);
        std::vector<int> polar_encoded_dci(N, 0);
        int input_size = K;
        free5GRAN::phy::transport_channel::polar_encoding(N, dci_bits_with_crc, input_size, polar_encoded_dci);
        free5GRAN::utils::common_utils::display_vector(polar_encoded_dci, N, "polar_encoded_dci");


        /** Rate Matching */
        int agg_level = pow(2, 3);
        int E = agg_level * free5GRAN::NUMBER_REG_PER_CCE * 9 * 2;

        std::vector<int> rate_matched_dci(E,0);
            // BE CAREFUL: for now, E is not an entry for function rate_matched_polar_coding. It's fixed to 864.
        free5GRAN::phy::transport_channel::rate_matching_polar_coding(polar_encoded_dci, rate_matched_dci);

        free5GRAN::utils::common_utils::display_vector(rate_matched_dci, E, "rate_matched_dci from main");



        /** UE try to decode */
        //int E = 9; // to be modified

        std::cout<<"\nE = "<<E<<std::endl;
        bool validated;
        free5GRAN::dci_1_0_si_rnti dci_object_UE;
        phy_variable_main.UE_decode_polar_dci(rate_matched_dci, input_size, N, E, input_size, freq_domain_ra_size, free5GRAN::SI_RNTI, validated, dci_object_UE);

        /** print dci_object_UE to verify that it's well decoded */
        std::cout<<"\ndci_object_UE.RIV = "<<dci_object_UE.RIV<<std::endl;
        std::cout<<"dci_object_UE.TD_ra = "<<dci_object_UE.TD_ra<<std::endl;
        std::cout<<"dci_object_UE.vrb_prb_interleaving = "<<dci_object_UE.vrb_prb_interleaving<<std::endl;
        std::cout<<"dci_object_UE.mcs = "<<dci_object_UE.mcs<<std::endl;
        std::cout<<"dci_object_UE.rv = "<<dci_object_UE.rv<<std::endl;
        std::cout<<"dci_object_UE.si = "<<dci_object_UE.si<<std::endl;

    }

}
