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

#ifndef FREE5GRAN_PHY_H
#define FREE5GRAN_PHY_H


#include <vector>
#include <chrono>
#include "../rf/rf.h"
#include "../../lib/variables/common_structures/common_structures.h"
#include "../../lib/asn1c/nr_rrc/BCCH-DL-SCH-Message.h"

class phy {


private:
    rf *rf_device;
    int common_cp_length, pci, i_b_ssb, n_id_1, n_id_2, *mib_bits, scs, is_extended_cp, i_ssb, l_max, index_first_pss, n_size_bwp, num_slots_per_frame, mu, frame_size;
    double ssb_period;
    int fft_size;
    bool cell_confirmed, crc_validated;
    float freq_offset, max_snr;
    std::vector<std::complex<float>> frame_data;
    free5GRAN::mib mib_object;
    free5GRAN::band band_object;
    free5GRAN::dci_1_0_si_rnti  dci_1_0_si_rnti;
    free5GRAN::pdcch_t0ss_monitoring_occasions pdcch_ss_mon_occ;
    std::chrono::time_point<std::chrono::high_resolution_clock> time_first_pss;
    std::vector<std::complex<float>> buff;
    BCCH_DL_SCH_Message_t* sib1=0;

public:
    phy(rf *rf_dev, double ssb_period, int fft_size, int scs, free5GRAN::band band_obj);
    void reconfigure(int fft_size);
    phy();

    /*
     * Try to find a cell. Execute time domain synchronization. PSS and SSS correlation. Compute received power
     */
    int cell_synchronization(float &received_power);

    /*
     * Re-synchronize with cell. Fine frequency synchronization. Resource element de-mapper (TS 38.211 7.3.3.3). Channel estimation & equalization
     */
    int extract_pbch();

    void search_pdcch(bool &validated);

    void print_cell_info();

    void parse_dci_1_0_si_rnti(int *dci_bits, int freq_domain_ra_size, free5GRAN::dci_1_0_si_rnti &dci);

    void print_dci_info();

    void extract_pdsch();

    BCCH_DL_SCH_Message_t* getSib();

    void print_sib1();

    int getSIB1RV();

};


#endif
