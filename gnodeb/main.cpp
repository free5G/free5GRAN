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

        //K = freq_domain_ra_size +4+1+5+2+1+15+24;
        K = freq_domain_ra_size +4+1+5+2+1;
        int dci_bits[K];
        std::cout<<"freq_domain_ra_size = "<<freq_domain_ra_size<<std::endl;
        phy_variable_main.encode_dci(dci_object, dci_bits, freq_domain_ra_size);
        free5GRAN::utils::common_utils::display_table(dci_bits, K, "dci_bits");
    }
}
