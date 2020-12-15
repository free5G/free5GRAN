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
    int * pdcch_bits;
    pdcch_bits = new int[agg_level * 6 * 9 * 2];
    /*
     * Demodulate PBCH Signal
     */
    free5GRAN::phy::signal_processing::hard_demodulation(pdcch_symbols,pdcch_bits,agg_level * 6 * 9,1);

    int * c_seq = new int[agg_level * 6 * 9 * 2];
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
    int *c_seq, ds_sch_bits_length;
    long c_init;
    ds_sch_bits_length = 2 * pdsch_samples.size();
    c_seq = new int[ds_sch_bits_length];
    double *soft_bits = new double[ds_sch_bits_length];
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
    int * pbch_bits;
    pbch_bits = new int[free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2];
    /*
     * Demodulate PBCH Signal
     */
    free5GRAN::phy::signal_processing::hard_demodulation(pbch_symbols,pbch_bits,free5GRAN::SIZE_SSB_PBCH_SYMBOLS,1);

    // Generate de-scrambling sequence
    int * c_seq = new int[free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2 * (1 + i_ssb)];
    free5GRAN::utils::sequence_generator::generate_c_sequence(pci, free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2 * (1 + i_ssb), c_seq,0);

    /*
     * De-scramble pbch_bits to scrambled_bits
     */
    free5GRAN::utils::common_utils::scramble(pbch_bits, c_seq, bch_bits, free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2, i_ssb * free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2);
}