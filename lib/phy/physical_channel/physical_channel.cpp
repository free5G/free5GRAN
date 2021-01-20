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

#include <iostream>
#include <fstream>
#include "physical_channel.h"
#include "../libphy/libphy.h"
#include "../../utils/sequence_generator/sequence_generator.h"
#include "../transport_channel/transport_channel.h"
#include "../../variables/common_variables/common_variables.h"
#include "../../utils/common_utils/common_utils.h"

using namespace std;

void free5GRAN::phy::physical_channel::decode_pdcch(vector<complex<float>> pdcch_symbols,int *dci_bits, int agg_level, int* reg_index, int* reg_index_sorted, int pci) {
    /**
     * \fn decode_pdcch
     * \brief PDCCH decoding
     * \standard TS 38.211 V15.2.0 Section 7.3.2
     *
     * \details
     * Details:
     * - CCE to REG de-interleaving
     * - Demodulation
     * - Scrambling
     *
     * \param[in] pdcch_symbols: Input IQ data to be decoded
     * \param[out] dci_bits: PDCCH decoded bits
     * \param[in] agg_level: Aggregation level
     * \param[in] reg_index: Interleaved REG indexes
     * \param[in] reg_index_sorted: Sorted interleaved REG indexes
     * \param[in] pci: Cell PCI
    */
    int pdcch_bits[agg_level * 6 * 9 * 2];
    /*
     * Demodulate PBCH Signal
     */
    free5GRAN::phy::signal_processing::hard_demodulation(pdcch_symbols,pdcch_bits,agg_level * 6 * 9,1);

    int c_seq[agg_level * 6 * 9 * 2];
    free5GRAN::utils::sequence_generator::generate_c_sequence((long) pci % (long)pow(2,31), agg_level * 6 * 9 * 2, c_seq,0);

    /*
     * De-scramble pbch_bits to scrambled_bits
     */
    free5GRAN::utils::common_utils::scramble(pdcch_bits, c_seq, dci_bits, agg_level * 6 * 9 * 2, 0);

}

void free5GRAN::phy::physical_channel::decode_pdsch(vector<complex<float>> pdsch_samples, double *unscrambled_soft_bits, int pci) {
    /**
     * \fn decode_pdsch
     * \brief PDSCH decoding
     * \standard TS 38.211 V15.2.0 Section 7.3.1
     * \partial_imp No layer mapping, antenna port mapping and VRB to PRB de-interleaving
     * \details
     * Details:
     * - Demodulation
     * - Scrambling
     *
     * \param[in] pdsch_samples: Input IQ data to be decoded
     * \param[out] unscrambled_soft_bits: PDSCH decoded soft bits
     * \param[in] pci: Cell PCI
    */
    int ds_sch_bits_length;
    long c_init;
    ds_sch_bits_length = 2 * pdsch_samples.size();
    int c_seq[ds_sch_bits_length];
    double soft_bits[ds_sch_bits_length];
    /*
     * Soft demodulation
     */
    free5GRAN::phy::signal_processing::soft_demodulation(pdsch_samples, soft_bits, pdsch_samples.size(), 1);

    /*
     * C init computation. 65535 is the RNTI for SIB1
     */
    c_init = 65535 * pow(2,15) + pci;

    /*
     * Scrambling
     */
    free5GRAN::utils::sequence_generator::generate_c_sequence(c_init, ds_sch_bits_length, c_seq, 1);
    free5GRAN::utils::common_utils::scramble(soft_bits, c_seq, unscrambled_soft_bits, ds_sch_bits_length, 0);

}

void free5GRAN::phy::physical_channel::decode_pbch(vector<complex<float>> pbch_symbols, int i_ssb, int pci, int *bch_bits) {
    /**
     * \fn decode_pbch
     * \brief PBCH decoding
     * \standard TS 38.211 V15.2.0 Section 7.3.3
     *
     * \details
     * Details:
     * - Demodulation
     * - Scrambling
     *
     * \param[in] pbch_symbols: Input IQ data to be decoded
     * \param[in] i_ssb: SS Block index
     * \param[in] pci: Cell PCI
     * \param[out] bch_bits: PBCH decoded bits
    */
    int pbch_bits[free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2];
    /*
     * Demodulate PBCH Signal
     */
    free5GRAN::phy::signal_processing::hard_demodulation(pbch_symbols,pbch_bits,free5GRAN::SIZE_SSB_PBCH_SYMBOLS,1);

    // Generate de-scrambling sequence
    int c_seq[free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2 * (1 + i_ssb)];
    free5GRAN::utils::sequence_generator::generate_c_sequence(pci, free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2 * (1 + i_ssb), c_seq,0);

    /*
     * De-scramble pbch_bits to scrambled_bits
     */
    free5GRAN::utils::common_utils::scramble(pbch_bits, c_seq, bch_bits, free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2, i_ssb * free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2);
}

void free5GRAN::phy::physical_channel::compute_pbch_indexes(vector<vector<vector<int>>> &ref, int pci){
    /**
     * \fn compute_pbch_indexes
     * \brief Compute PBCH and DMRS symbols indexes
     * \param[out] ref: Reference grid for RE demapper
     * \param[in] pci: Physical Cell ID
    */
    //Looping over 3 OFDM symbols containing DMRS and PBCH
    for (int symbol = 1; symbol < free5GRAN::NUM_SYMBOLS_SSB; symbol ++){
        // Creating Resource element de-mapper reference grid
        for (int i = 0; i < free5GRAN::NUM_SC_SSB; i++){
            ref[2][symbol - 1][i] = 0;
            if (symbol == 1 || symbol == 3){
                if (pci % 4 != i % 4){
                    ref[0][symbol - 1][i] = 1;
                    ref[1][symbol - 1][i] = 0;
                }else{
                    ref[0][symbol - 1][i] = 0;
                    ref[1][symbol - 1][i] = 1;
                }
            }else if (symbol == 2){
                if (pci % 4 != i % 4 && (i < free5GRAN::INTERVAL_SSB_NO_PBCH_DMRS[0] || i > free5GRAN::INTERVAL_SSB_NO_PBCH_DMRS[1])){
                    ref[0][symbol - 1][i] = 1;
                    ref[1][symbol - 1][i] = 0;
                } else if (i < free5GRAN::INTERVAL_SSB_NO_PBCH_DMRS[0] || i > free5GRAN::INTERVAL_SSB_NO_PBCH_DMRS[1]){
                    ref[0][symbol - 1][i] = 0;
                    ref[1][symbol - 1][i] = 1;
                }else{
                    ref[0][symbol - 1][i] = 0;
                    ref[1][symbol - 1][i] = 0;
                }
                if (i >= 56 && i <=182){
                    ref[2][symbol - 1][i] = 1;
                }
            }
        }
    }
}

void free5GRAN::phy::physical_channel::compute_pdcch_indexes(vector<vector<vector<int>>> &ref, free5GRAN::pdcch_t0ss_monitoring_occasions pdcch_ss_mon_occ, int agg_level, int *reg_bundles, int height_reg_rb){
    /**
     * \fn compute_pdcch_indexes
     * \brief Compute PDCCH and DMRS samples Position
     * \param[out] ref: Reference grid for RE demapper.
     * \param[in] pdcch_ss_mon_occ: Search Space configuration
     * \param[in] agg_level: PDCCH candidate aggregation level
     * \param[in] reg_bundles: REG positions after CCE-to-REG interleaving
     * \param[in] height_reg_rb: Frequency-domain REG height, in RB
    */
    for (int symb = 0; symb < pdcch_ss_mon_occ.n_symb_coreset; symb ++){
        for (int k = 0 ; k < agg_level ; k ++) {
            for (int reg = 0; reg < free5GRAN::NUMBER_REG_PER_CCE; reg++) {
                if (reg % pdcch_ss_mon_occ.n_symb_coreset == symb){
                    for (int s = 0; s < 12; s++) {
                        if (s % 4 == 1){
                            ref[1][symb][reg_bundles[k] * 12 * height_reg_rb + (reg/pdcch_ss_mon_occ.n_symb_coreset) * 12 + s] = 1;
                            ref[0][symb][reg_bundles[k] * 12 * height_reg_rb + (reg/pdcch_ss_mon_occ.n_symb_coreset) * 12 + s] = 0;
                        }else {
                            ref[1][symb][reg_bundles[k] * 12 * height_reg_rb + (reg/pdcch_ss_mon_occ.n_symb_coreset) * 12 + s] = 0;
                            ref[0][symb][reg_bundles[k] * 12 * height_reg_rb + (reg/pdcch_ss_mon_occ.n_symb_coreset) * 12 + s] = 1;
                        }
                    }
                }
            }
        }
    }
}

void free5GRAN::phy::physical_channel::compute_pdsch_indexes(vector<vector<vector<int>>> &ref, bool dmrs_symbol_array[], int L, int lrb){
    /**
     * \fn compute_pdsch_indexes
     * \brief Compute PDSCH and DMRS samples Position
     * \param[out] ref: Reference grid for RE demapper.
     * \param[in] dmrs_symbol_array: True if symbol contains DMRS, false else
     * \param[in] L: Number of symbols for PDSCH
     * \param[in] lrb: Number of RB for PDSCH
    */
    // Creating Resource element de-mapper reference grid
    for (int symb = 0; symb < L; symb ++){
        for (int i = 0; i < 12 * lrb; i++){
            if (dmrs_symbol_array[symb]){
                if (i % 2 == 0){
                    ref[0][symb][i] = 0;
                    ref[1][symb][i] = 1;
                }else {
                    ref[0][symb][i] = 0;
                    ref[1][symb][i] = 0;
                }
            }
            else{
                ref[0][symb][i] = 1;
                ref[1][symb][i] = 0;
            }
        }
    }
}
