/*
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
 */

#include "sequence_generator.h"
#include "../../variables/common_variables/common_variables.h"
#include <cmath>
#include <iostream>
using namespace std;

void free5GRAN::utils::sequence_generator::generate_pss_sequence(int n_id_2, int *output_sequence) {
    /**
     * \fn generate_pss_sequence
     * \brief Generate PSS sequence
     * \standard TS 38.211 V15.2.0 Section 7.4.2.2.1
     *
     * \param[in] n_id_2: N_ID_2 for which PSS is generated
     * \param[out] output_sequence: output sequence
     */
    int x_seq[free5GRAN::SIZE_PSS_SSS_SIGNAL], m;
    for (int i = 0; i < free5GRAN::SIZE_PSS_SSS_SIGNAL; i++){
        if (i < 7){
            x_seq[i] = free5GRAN::PSS_BASE_SEQUENCE[i];
        } else {
            x_seq[i] = (x_seq[i - 3] + x_seq[i - 7]) % 2;
        }
    }
    for (int n = 0; n < free5GRAN::SIZE_PSS_SSS_SIGNAL; n++){
        m = (n + 43 * n_id_2) % free5GRAN::SIZE_PSS_SSS_SIGNAL;
        output_sequence[n] = 1 - 2 * x_seq[m];
    }

}

void free5GRAN::utils::sequence_generator::generate_sss_sequence(int n_id_1, int n_id_2, int *output_sequence) {
    /**
     * \fn generate_sss_sequence
     * \brief Generate SSS sequence
     * \standard TS 38.211 V15.2.0 Section 7.4.2.3.1
     *
     * \param[in] n_id_1: N_ID_1 for which SSS is generated
     * \param[in] n_id_2: N_ID_2 for which SSS is generated
     * \param[out] output_sequence: output sequence
     */
    int m0, m1;
    int x0_seq[free5GRAN::SIZE_PSS_SSS_SIGNAL];
    int x1_seq[free5GRAN::SIZE_PSS_SSS_SIGNAL];
    for (int i = 0; i < free5GRAN::SIZE_PSS_SSS_SIGNAL; i++){
        if (i < 7){
            x0_seq[i] = free5GRAN::SSS_BASE_X0_SEQUENCE[i];
            x1_seq[i] = free5GRAN::SSS_BASE_X1_SEQUENCE[i];;
        } else {
            x0_seq[i] = (x0_seq[i - 3] + x0_seq[i - 7]) % 2;
            x1_seq[i] = (x1_seq[i - 6] + x1_seq[i - 7]) % 2;
        }
    }
    m0 = 15 * (n_id_1 / 112) + 5 * n_id_2;
    m1 = n_id_1 % 112;
    for (int n = 0; n < free5GRAN::SIZE_PSS_SSS_SIGNAL; n++){
        output_sequence[n] = (1 - 2 * x0_seq[(n + m0) % free5GRAN::SIZE_PSS_SSS_SIGNAL]) * (1 - 2 * x1_seq[(n + m1) % free5GRAN::SIZE_PSS_SSS_SIGNAL]);
    }
}

void free5GRAN::utils::sequence_generator::generate_pbch_dmrs_sequence(int pci, int i_bar_ssb, complex<float> *output_sequence) {
    /**
     * \fn generate_pbch_dmrs_sequence
     * \brief Generate PBCH DMRS sequence
     * \standard TS 38.211 V15.2.0 Section 7.4.1.4.1
     *
     * \param[in] pci: Cell PCI
     * \param[in] i_bar_ssb: SSB index value
     * \param[out] output_sequence: output sequence
     */
    int c_init = pow(2,11) * (i_bar_ssb + 1) * (pci / 4 + 1) + pow(2,6) * (i_bar_ssb + 1) + (pci % 4);
    int seq[2 * free5GRAN::SIZE_SSB_DMRS_SYMBOLS];
    generate_c_sequence(c_init, 2 * free5GRAN::SIZE_SSB_DMRS_SYMBOLS, seq, 0);
    for (int m = 0 ; m < free5GRAN::SIZE_SSB_DMRS_SYMBOLS; m++){
        output_sequence[m] = (float) (1 / sqrt(2)) * complex<float>(1 - 2 * seq[2 * m], 1 - 2 * seq[2 * m + 1]);
    }
}


void free5GRAN::utils::sequence_generator::generate_c_sequence(long c_init, int length, int *output_sequence, int demod_type) {
    /**
     * \fn generate_c_sequence
     * \brief Generic pseudo-random sequence generator
     * \standard TS 38.211 V15.2.0 Section 5.2
     *
     * \param[in] c_init: Sequence initializer
     * \param[in] length: Sequence length
     * \param[out] output_sequence: output sequence
     * \param[in] demod_type: Demodulation type (0 -> Hard demodulation (to be used by default) / 1 -> Soft demodulation)
     */
    int x1[1600 + length], x2[1600 + length], base_x2[32];

    for (int j = 0; j < 31; j ++){
        base_x2[j] = ((int) c_init / (int) pow(2,j)) % 2;
    }
    for (int n = 0; n < 1600 + length; n ++){
        if (n < 31){
            x1[n] = free5GRAN::DMRS_BASE_X1_SEQUENCE[n];
            x2[n] = base_x2[n];
        }else {
            x1[n] = (x1[n - 28] + x1[n - 31]) % 2;
            x2[n] = (x2[n - 28] + x2[n - 29] + x2[n - 30] + x2[n - 31]) % 2;
        }
    }
    for (int j = 0; j < length; j ++){
        output_sequence[j] = (x1[j + 1600] + x2[j + 1600]) % 2;
        if (demod_type == 1){
            output_sequence[j] = (output_sequence[j]==0) ? 1 : -1;
        }
    }

}

void free5GRAN::utils::sequence_generator::generate_pdcch_dmrs_sequence(int nid, int slot_number, int symbol_number, complex<float> *output_sequence, int size){
    /**
     * \fn generate_pdcch_dmrs_sequence
     * \brief Generic PDCCH DMRS sequence
     * \standard TS 38.211 V15.2.0 Section 7.4.1.3.1
     *
     * \param[in] nid: Scrambling ID (cell PCI by default)
     * \param[in] slot_number: Slot number within a frame
     * \param[in] symbol_number: Symbol number within a slot
     * \param[out] output_sequence: output sequence
     * \param[in] size: Sequence size
     */
    long c_init = (long)(pow(2, 17) * (14 * slot_number + symbol_number + 1) * (2 * nid + 1) + 2 * nid) % (long)pow(2,31);
    int seq[2 * size];
    generate_c_sequence(c_init, 2 * size, seq, 0);
    for (int m = 0; m < size; m++){
        output_sequence[m] =  (float) (1 / sqrt(2)) * complex<float>(1 - 2 * seq[2 * m], 1 - 2 * seq[2 * m + 1]);
    }
}


void free5GRAN::utils::sequence_generator::generate_pdcch_dmrs_sequence(int nid, int slot_number, int symbol_number, vector<complex<float>> &output_sequence, int size){
    /**
     * \fn generate_pdcch_dmrs_sequence
     * \brief Generic PDCCH DMRS sequence
     * \standard TS 38.211 V15.2.0 Section 7.4.1.3.1
     *
     * \param[in] nid: Scrambling ID (cell PCI by default)
     * \param[in] slot_number: Slot number within a frame
     * \param[in] symbol_number: Symbol number within a slot
     * \param[out] output_sequence: output sequence
     * \param[in] size: Sequence size
     */
    long c_init = (long)(pow(2, 17) * (14 * slot_number + symbol_number + 1) * (2 * nid + 1) + 2 * nid) % (long)pow(2,31);
    int seq[2 * size];
    generate_c_sequence(c_init, 2 * size, seq, 0);
    for (int m = 0; m < size; m++){
        output_sequence[m] =  (float) (1 / sqrt(2)) * complex<float>(1 - 2 * seq[2 * m], 1 - 2 * seq[2 * m + 1]);
    }
}

void free5GRAN::utils::sequence_generator::generate_pdsch_dmrs_sequence(int n_symb_slot, int slot_number, int symbol_number, int n_scid, int n_id_scid, complex<float> *output_sequence, int size){
    /**
     * \fn generate_pdsch_dmrs_sequence
     * \brief Generic PDSCH DMRS sequence
     * \standard TS 38.211 V15.2.0 Section 7.4.1.1.1
     *
     * \param[in] n_symb_slot: Number of symbols per slot
     * \param[in] slot_number: Slot number within a frame
     * \param[in] symbol_number: Symbol number within a slot
     * \param[in] n_scid: DMRS sequence initialization field (0 by default)
     * \param[in] n_id_scid: Sambling ID (cell PCI by default)
     * \param[out] output_sequence: output sequence
     * \param[in] size: Sequence size
     */
    long c_init = (long)(pow(2, 17) * (n_symb_slot * slot_number + symbol_number + 1) * (2 * n_id_scid + 1) + 2 * n_id_scid + n_scid) % (long)pow(2,31);
    int seq[2 * size];
    generate_c_sequence(c_init, 2 * size, seq, 0);
    for (int m = 0; m < size; m++){
        output_sequence[m] =  (float) (1 / sqrt(2)) * complex<float>(1 - 2 * seq[2 * m], 1 - 2 * seq[2 * m + 1]);
    }
}

