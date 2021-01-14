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

#include "phy.h"

#include <iostream>


#include "../../lib/utils/sequence_generator/sequence_generator.h"
#include "../../lib/variables/common_matrices/common_matrices.h"
#include "../../lib/phy/libphy/libphy.h"

#include <fftw3.h>
#include <complex>
#include <vector>
#include "../../lib/utils/sequence_generator/sequence_generator.h"
#include "../../lib/variables/common_variables/common_variables.h"
#include "../../lib/utils/common_utils/common_utils.h"
#include "../../lib/phy/libphy/libphy.h"

#include_next <math.h>
#include <fstream>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/file.hpp>




bool display_variable = false; /** indicates if you want to display all variables in the Console */

void phy::encode_mib(free5GRAN::mib mib_object, int *mib_bits) {

    /**
* \fn ph_ben * encode_mib (free5GRAN::mib mib_object, int* mib_bits)
* \brief This function aims to transforms the MIB informations (decimal) into the mib bits sequence.
* In our case, the mib bits sequence is 32 bits long.
* \standard TS38.331 V15.11.0 Section 6.2.2
*
* \param[in] mib_object object MIB created in common_structures.h, including sfn, scs, cell_barred...
* \param[out] mib_bits bit sequence returned by the function.
*/

    /** These following mib_bits are unused and set to 0 according to TS38.331 V15.11.0 Section 6.2.2 */
    int unused_bits_size = 5;
    for (int i = 0; i < unused_bits_size; i++) {
        mib_bits[free5GRAN::INDEX_OF_UNUSED_BITS_IN_MIB[i]] = 0;
    }

    /** Convert sfn (Sequence Frame Number) from decimal to binary. */
    int sfn_binary_size = 10;
    int sfn_binary[sfn_binary_size];
    convert_decimal_to_binary(sfn_binary_size, mib_object.sfn, sfn_binary);

    /** Put sfn bits into mib_bits sequence according to TS38.331 V15.11.0 Section 6.2.2 */
    for (int i = 0; i < sfn_binary_size; i++) {
        mib_bits[free5GRAN::INDEX_OF_SFN_BITS_IN_MIB[i]] = sfn_binary[i];
    }

    /** Convert pddchc_config from decimal to binary */
    int pddchc_config_binary_size = 8;
    int pddchc_config_binary[pddchc_config_binary_size];
    convert_decimal_to_binary(pddchc_config_binary_size, mib_object.pdcch_config, pddchc_config_binary);

    /** Put the pddchc_config bits into mib_bits sequence according to TS38.331 V15.11.0 Section 6.2.2 */
    for (int i = 0; i < pddchc_config_binary_size; i++) {
        mib_bits[free5GRAN::INDEX_OF_PDDCHC_CONFIG_BITS_IN_MIB[i]] = pddchc_config_binary[i];
    }

    /** Convert k_ssb from decimal to binary */
    int k_ssb_binary_size = 5;
    int k_ssb_binary[k_ssb_binary_size];
    convert_decimal_to_binary(k_ssb_binary_size, mib_object.k_ssb, k_ssb_binary);

    /** Put the k_ssb bits into mib_bits sequence according to TS38.331 V15.11.0 Section 6.2.2 */
    for (int i = 0; i < k_ssb_binary_size; i++) {
        mib_bits[free5GRAN::INDEX_OF_K_SSB_BITS_IN_MIB[i]] = k_ssb_binary[i];
    }


    int available_scs[2] = {15, 30}; /** In FR1, SCS = 15 kHz or 30 kHz. In FR2, SCS = 30 kHz or 60 kHz */
    for (int i = 0; i < 2; i++) {
        /** Put SCS (Sub Carrier Spacing) bit into mib_bits sequence, according to TS38.331 V15.11.0 Section 6.2.2 */
        if (mib_object.scs == available_scs[i]) {
            mib_bits[free5GRAN::INDEX_OF_AVAILABLE_SCS_IN_MIB[0]] = i;
        }
    }


    /** The following mib information are not needed to be converted into bits as they are stored on only 1 bit */

    /** Put the cell_barred bit into mib_bits sequence, according to TS38.331 V15.11.0 Section 6.2.2 */
    mib_bits[free5GRAN::INDEX_OF_CELL_BARRED_BITS_IN_MIB[0]] = mib_object.cell_barred;

    /** Put the DMRS type A position (position of first DM-RS for downlink) bit into mib_bits sequence, according to TS38.331 V15.11.0 Section 6.2.2 */
    mib_bits[free5GRAN::INDEX_OF_DMRS_TYPE_A_POSITION_BITS_IN_MIB[0]] = mib_object.dmrs_type_a_position - 2;

    /** Put the intra frequency reselection bit into mib_bits sequence, according to TS38.331 V15.11.0 Section 6.2.2 */
    mib_bits[free5GRAN::INDEX_OF_INTRA_FREQ_RESELECTION_BITS_IN_MIB[0]] = mib_object.intra_freq_reselection;
    BOOST_LOG_TRIVIAL(info) << "function encode_mib done";
}


void phy::bch_interleaving(int *mib_bits, int *mib_bits_interleaved) {

    /**
* \fn ph_ben * bch_interleaving (int* mib_bits, int* mib_bits_interleaved)
* \brief This function aims to interleave the MIB (Master Information Block) 32 bits sequence.
* \standard TS38.212 V15.2.0 Section 7.1.1
*
* \param[in] mib_bits MIB (Master Information Block) bits sequence, 32 bits long.
* \param[out] mib_bits_interleaved MIB bits sequence interleaved, 32 bits long.
*/

    /** Initialize the begin values used in the for loop */
    int A_bar = free5GRAN::BCH_PAYLOAD_SIZE - 8; // = 24
    int j_sfn = 0;
    int j_hrf = 10;
    int j_ssb = 11;
    int j_other = 14;


    /** For each i going from 0 to 31, the ith bits of mib_bits_interleaved is settled in function of the mib_bits and according to the PBCH payload interleaver pattern, in TS38.212 V15.2.0 Section 7.1.1 */
    for (int i = 0; i < free5GRAN::MIB_BITS_SIZE; i++) {
        if (i == 24 || i == 25 || i == 26 || i == 27 || i == 1 || i == 2 || i == 3 || i == 4 || i == 5 || i == 6) {
            mib_bits_interleaved[free5GRAN::PBCH_PAYLOAD_INTERLEAVER[j_sfn]] = mib_bits[i];
            j_sfn++;
        } else if (i == 28) {
            mib_bits_interleaved[free5GRAN::PBCH_PAYLOAD_INTERLEAVER[j_hrf]] = mib_bits[i];
        } else if (i >= A_bar + 5 && i <= A_bar + 7) {
            mib_bits_interleaved[free5GRAN::PBCH_PAYLOAD_INTERLEAVER[j_ssb]] = mib_bits[i];
            j_ssb++;
        } else {
            mib_bits_interleaved[free5GRAN::PBCH_PAYLOAD_INTERLEAVER[j_other]] = mib_bits[i];
            j_other++;
        }
    }
    BOOST_LOG_TRIVIAL(info) << "function bch_interleaving done. At this point, we have "+std::to_string(free5GRAN::MIB_BITS_SIZE)+ " bits";
}


void phy::scrambling_bch(int v, int pci, int *mib_bits_interleaved, int *bch_payload) {

    /**
* \fn ph_ben * scrambling_bch (int v, int pci, int* mib_bits_interleaved, int* bch_payload)
* \brief This function aims to scramble the 32 bits of mib bits (interleaved) to get the bch_payload (still 32 bits long)
* \details
* - First, a c_seq is generated from the pci, v and BCH payload size.
* - Then, a s_sequence is generated using c_seq.
* - Finally, the mib_bits_interleaved is scrambled into bch_payload sequence, using s_sequence.
* \standard TS38.212 V15.2.0 Section 7.1.2
*
* \param[in] v v depends on the SFN value (3rd LSB of SFN and 2nd LSB of SFN). It is an integer between 0 and 3.
* \param[in] pci pci is the Physical Cell Id (also called N cell ID)
* \param[in] mib_bits_interleaved MIB (Master Information Block) bits sequence, interleaved, 32 bits long.
* \param[out] bch_payload bch_payload, 32 bits long.
*/


    /** Initialize some variables */
    int *s_sequence, *c_seq;
    int A = free5GRAN::BCH_PAYLOAD_SIZE; // =32
    int M = A - 3;
    c_seq = new int[free5GRAN::BCH_PAYLOAD_SIZE + v * M];
    s_sequence = new int[A];

    /** Generate the c_sequence, according to TS38.211 5.2.1  V15.2.0 with c_init = pci */
    free5GRAN::utils::sequence_generator::generate_c_sequence(pci, free5GRAN::BCH_PAYLOAD_SIZE + v * M, c_seq, 0);

    /** Generate s_sequence from c_sequence, according to TS38.212 V15.2.0 Section 7.1.2 */
    int j = 0;
    for (int i = 0; i < A; i++) {
        if (i == 0 || i == 6 || i == 24) {
            s_sequence[i] = 0;
        } else {
            s_sequence[i] = c_seq[j + v * M];
            j++;
        }
    }

    /** Scramble mib_bits_interleaved using s_sequence to get bch_payload */
    free5GRAN::utils::common_utils::scramble(mib_bits_interleaved, s_sequence, bch_payload, free5GRAN::BCH_PAYLOAD_SIZE, 0);
    BOOST_LOG_TRIVIAL(info) << "function scrambling_bch done. At this point, we have "+std::to_string(free5GRAN::MIB_BITS_SIZE)+ " bits";
}


void phy::adding_crc(int *bch_payload, int *bch_payload_with_crc) {

    /**
* \fn ph_ben * adding_crc (int* bch_payload, int*bch_payload_with_crc)
* \brief This function aims to generate and add the CRC (Cyclic Redundancy Check) bits sequence of bch_payload bits sequence.
* \details
* First, the 24 bits bch_crc are computed from bch_payload sequence.
* Then, these 24 bits bch_crc are added at the end of bch_payload to get a 56 bits bch_payload_with_crc sequence.
* \standard TS38.212 V15.2.0 Section 5.1
* \param[in] bch_payload bch_payload bch payload, 32 bits long.
* \param[out] bch_payload_with_crc Contains 56 bits: 32 bits of bch payload and 24 bits of CRC.
*/

    /** Generate the 24 bits sequence CRC (bch_crc) of bch_payload, using the table G_CRC_24_C */
    free5GRAN::phy::transport_channel::compute_crc(bch_payload, free5GRAN::G_CRC_24_C, bch_crc, 32, 25);

    /** Complete the 32 first bits of bch_payload_with_crc with the bch_payload bits sequence */
    for (int i = 0; i < free5GRAN::BCH_PAYLOAD_SIZE; i++) {
        bch_payload_with_crc[i] = bch_payload[i];
    }

    /** Completing the 24 last bits of bch_payload_wit_crc with the bch_crc bits sequence */
    for (int i = 0; i < 24; i++) {
        bch_payload_with_crc[free5GRAN::BCH_PAYLOAD_SIZE + i] = bch_crc[i];
    }
    BOOST_LOG_TRIVIAL(info) << "function adding_crc done. At this point, we have "+std::to_string(free5GRAN::MIB_BITS_SIZE + 24)+ " bits";
}


void phy::polar_encode_bch(int N, int *bch_payload_with_crc, int *polar_encoded_bch) {

/**
* \fn ph_ben * polar_encode_bch (int N, int* bch_payload_with_crc, int* polar_encoded_bch)
* \brief This function aims to transform the 56 bits sequence bch_payload_with_crc into a 512 bits sequence polar_encoded_bch.
* \details
 * First, a pi_sequence is generated (56 bits long).
 * Then, a c_p sequence is generated from this pi_sequence and from the bch_payload_with_crc bits sequence.
 * Then, a q_0_n_1 sequence is generated.
 * Then, a q_i_n sequence is generated from this q_0_n_1 sequence.
 * Then, a u sequence is generated from the c_p sequence using the q_i_n sequence.
 * Finally, the polar_encoded_bch bits sequence is generated from the u sequence using the G9 matrix.
 *
* \standard TS38.212 V15.2.0 Section 5.3.1
* \param[in] N indicates the length of polar_encoded_bch bits sequence. N depends on the size of SSB PBCH Symbols and on the size of PBCH POLAR DECODED bit sequence. In our case, N=512.
* \param[in] bch_payload_with_crc Bits sequence. 56 bits long.
* \param[out] polar_encoded_bch Bit sequence. 512 bits long.
*/


    /** Initializing variables */
    int n_pc = 0;
    int i_il = 1;
    int nmax = 9;
    int n_wm_pc = 0;
    bool found;
    int K = free5GRAN::SIZE_PBCH_POLAR_DECODED; // = 56
    int K_max = 164;
    int *c_p = new int[56];
    int *q_0_n_1;
    q_0_n_1 = new int[N];
    int *q_i_n = new int[K + n_pc];
    int *u = new int[N];
    pi_seq = new int[56];
    int count_seq = 0;


    /** Generate pi sequence according to TS38.212 V15.2.0 Section 5.3.1.1 */
    for (int m = 0; m < K_max; m++) {
        if (free5GRAN::INTERLEAVING_PATTERN[m] >= K_max - K) {
            pi_seq[count_seq] = free5GRAN::INTERLEAVING_PATTERN[m] - (K_max - K);
            count_seq++;
        }
    }


    /** Generating c_p sequence from bch_payload_with_crc using pi_seq, according to TS38.212 V15.2.0 Section 5.3.1.1 */
    for (int k = 0; k < 56; k++) {
        c_p[k] = bch_payload_with_crc[pi_seq[k]];  // modified
    }

    /** Generating q_0_n_1 according to TS38.212 V15.2.0 Section 5.3.1.2 */
    count_seq = 0;
    for (int n = 0; n < 1024; n++) {
        if (free5GRAN::POLAR_SEQUENCE_QNMAX_AND_RELIABILITY[n] < N) {
            q_0_n_1[count_seq] = free5GRAN::POLAR_SEQUENCE_QNMAX_AND_RELIABILITY[n];
            count_seq++;
        }
    }


    /** Generate q_i_n sequance from q_0_n_1 according to TS38.212 V15.2.0 Section 5.3.1.2 */
    for (int n = 0; n < K + n_pc; n++) {
        q_i_n[n] = q_0_n_1[N - (K + n_pc) + n];
    }


    /** Polar coding apply to c_p to get u (using q_i_n) according to TS38.212 V15.2.0 Section 5.3.1.2 */
    count_seq = 0;
    for (int n = 0; n < N; n++) {
        found = false;
        for (int p = 0; p < K + n_pc; p++) {
            if (q_i_n[p] == n) {
                found = true;
                break;
            }
        }
        if (found) {
            u[n] = c_p[count_seq];
            count_seq++;
        } else {
            u[n] = 0;
        }
    }


    /** polar coding apply to u to get polar_encoded_bch using G9 matrix according to TS38.212 V15.2.0 Section 5.3.1.2 */
    for (int n = 0; n < N; n++) {
        polar_encoded_bch[n] = 0;
        for (int p = 0; p < N; p++) {
            polar_encoded_bch[n] ^= (u[p] * free5GRAN::G9[p][n]);  //G9 to be verified
        }
    }
    BOOST_LOG_TRIVIAL(info) << "function polar_encode_bch done. At this point, we have "+std::to_string(N)+ " bits";
}


void phy::rate_matching(int *polar_encode_bch, int *rate_matched_bch) {

    /**
* \fn ph_ben * rate_matching(int* polar_encode_bch, int* rate_matched_bch)
* \brief This function aims to apply the rate matching to the 512 bits sequence polar_encoded_bch to get a 864 bits long rate_matched_bch.
* \details
     * First the bits contained in polar_encoded_bch are interleaved (again).
     * Then, the 352 first bits of polar_encoded_bch are added at the end of this sequence, to get a 864 bits long sequence
* \standard TS38.212 V15.2.0 Section 5.4.1
* \param[in] polar_encode_bch polar_encode_bch, 512 bits long.
* \param[out] rate_matched_bch contains the final BCH 864 bits sequence.
*/

    /** Initialize variables */
    // Be carreful : replace compute_N by compute_N_olar_code when merging
    int n = free5GRAN::phy::transport_channel::compute_N_polar_code(free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2, free5GRAN::SIZE_PBCH_POLAR_DECODED, 9);
    int N = pow(2, n);
    int E = free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2;
    int i;
    int j_n;
    int *b1 = new int[N];

    /** Interleaving apply to polar_encode_bch to get the b1 sequence */
    for (int n = 0; n < N; n++) {
        i = floor(32 * (double) n / (double) N);
        j_n = free5GRAN::SUB_BLOCK_INTERLEAVER_PATTERN[i] * N / 32 + n % (N / 32);
        b1[n] = polar_encode_bch[j_n];
    }

    /** Add 352 bits at the end of b1 to get rate_matched_bch */
    for (int n = 0; n < E; n++) {
        if (n < N) {
            rate_matched_bch[n] = b1[n];
        } else {
            rate_matched_bch[n] = b1[n - N];
        }
    }
    BOOST_LOG_TRIVIAL(info) << "function rate_matching done. At this point, we have "+std::to_string(free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2)+ " bits";
}


void phy::encode_pbch(int gscn, int pci, int i_b_ssb, int* rate_matched_bch, int* encoded_pbch) {

    /**
* \fn ph_ben * encode_pbch(int gscn, int pci, int i_b_ssb, int* rate_matched_bch, int* encoded_pbch)
* \brief This function aims to scramble the 864 rate_matched_bch bits to get 864 encoded_pbch bits.
* \details
    * First l_max is calculated in function of the carrier frequency (designated by gscn)
    * Then, i_ssb (sometimes called v) is calculated in function of lmax and i_b_ssb
    * Then, a c_seq2 sequence is generated in function of pci and i_ssb
    * Finally, the 864 rate_matched_bch bits are scrambled to get the 864 encoded_pbch bits, using c_seq2
* \standard TS38.211 V15.2.0 Section 7.3.3.1
* \param[in] gscn Global Synchronization Channel Number
* \param[in] pci Physical Cell Id
* \param[in] i_b_ssb. It is the SSB index
* \param[in] rate_matched_bch, Rate matched BCH, 864 bits sequence.
* \param[out] encoded_pbch encoded pbch 864 bits sequence.
*/

    /** Initialize variables */
    double frequency;
    double ssb_period;
    frequency = free5GRAN::phy::signal_processing::compute_freq_from_gscn(gscn);

    /** Calculate l_max in function of the carrier frequency (designated by gscn) */
    if (frequency > 3000000000){
        l_max = 8;
    }else{
        l_max = 4;
    }

    /** Calculate i_ssb (sometimes called v) in function of l_max and i_b_ssb */
    if (l_max == 4){
        i_ssb = i_b_ssb % 4; //also called v in Aymeric's documentation.
    }else {
        i_ssb = i_b_ssb; //also called v in Aymeric's documentation.
    }

    /** Generate a c_seq2 sequence in function of pci and i_ssb */
    int * c_seq2 = new int[free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2 * (1 + i_ssb)];
    free5GRAN::utils::sequence_generator::generate_c_sequence(pci, free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2 * (1 + i_ssb), c_seq2,0);

    /** Scramble the 864 rate_matched_bch bits to get the 864 encoded_pbch bits, using c_seq2 */
    free5GRAN::utils::common_utils::scramble(rate_matched_bch, c_seq2, encoded_pbch, free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2, i_ssb * free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2);

    BOOST_LOG_TRIVIAL(info) << "function encode_pbch done. At this point, we have "+std::to_string(free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2)+ " bits";
}


void phy::modulation(int *bits, int bit_sequence_length, int modulation_scheme, std::complex<float> *pbch_symbols){

    /**
* \fn ph_ben * modulation(int *bits, int bit_sequence_length, int modulation_scheme, std::complex<float> *pbch_symbols)
* \brief This function aims to convert a bits sequence into IQ symbols, using BPSK (Binary Phase Shit Keying) or QPSK (Quadrature Phase Shift Keying)
* \details
    * For the BPSK (if modulation_scheme == 0): for each bit, the corresponding symbol will be 1 or -1
    * For the QPSK (if modulation_scheme == 1): for each pair of bits, the corresponding symbols will be +/- 1/sqrt(2) +/- i 1/sqrt(2)
* \standard TS38.211 V15.2.0 Section 5.1
* \param[in] *bits The input sequence of bits.
* \param[in] bit_sequence_length number of bits.
* \param[in] modulation_scheme. 0 if BPSK, 1 if QPSK.
* \param[out] pbch_symbols the output sequence of IQ symbols.
*/


    /**  BPSK modulation (modulation_scheme == 0)
* For the BPSK modulation pattern, see the TS38.211 V15.2.0 Section 5.1.2 */
    if (modulation_scheme == 0) {
        BOOST_LOG_TRIVIAL(info) << "Modulation scheme = BPSK";
        for (int i = 0; i < bit_sequence_length ; i++) {

            pbch_symbols[i] = {(1/(float) sqrt(2)) * (1 - 2*bits[2*i]), (1/(float) sqrt(2) * (1 - 2*bits[2*i]))};

        }
    }

    /**  QPSK modulation (modulation_scheme == 1)
    * For the QPSK modulation pattern, see the TS38.211 V15.2.0 Section 5.1.3 */

    if(modulation_scheme == 1){
        BOOST_LOG_TRIVIAL(info) << "Modulation scheme = QPSK";
        for (int i=0; i < (bit_sequence_length/2); i++) {
            pbch_symbols[i] = {(1/(float) sqrt(2)) * (1 - 2*bits[2*i]), (1/(float) sqrt(2)) * (1 - 2*bits[2*i + 1]) };
        }
    }

    BOOST_LOG_TRIVIAL(info) << "function modulation done. At this point, we have "+std::to_string(free5GRAN::SIZE_SSB_PBCH_SYMBOLS)+ " complex symbols";
}


void phy::generate_dmrs_of_pbch(int pci, int i_b_ssb, std::complex<float> *dmrs_symbols){

    /**
* \fn ph_ben * generate_dmrs_of_pbch (int pci, int i_b_ssb, std::complex<float> *pbch_symbols, std::complex<float> *dmrs_symbols)
* \brief This function aims to generate the DMRS (Demodulation Reference Signal).
* \standard TS38.211 V15.2.0 Section 7.4.1.4.1
* \param[in] pci Physical Cell Id
* \param[in] i_b_ssb. It is the SSB index
* \param[out] dmrs_symbols. In our case, it is a 144 symbols sequence length.
*/
    free5GRAN::utils::sequence_generator::generate_pbch_dmrs_sequence(pci, i_b_ssb, dmrs_symbols);
    BOOST_LOG_TRIVIAL(info) << "function generate_dmrs_of_pbch done. It will give "+std::to_string(free5GRAN::SIZE_SSB_DMRS_SYMBOLS)+ " complex symbols";
}


void phy::construct_reference_grid(int num_sc_ssb, int num_symbols_ssb, int pci, int ***ref) {
/**
* \fn ph_ben * construct_reference_grid (int num_sc_ssb, int num_symbols_ssb, int pci, int ***ref)
* \brief This function aims to construct a reference grid (called ref) to build the SSB signal.
* \details
 * In our case, there are 4 channels to build on the SSB signal:  PSS (127 symbols), SSS (127 symbols), PBCH (432 symbols) and DMRS (144 symbols).
 * In our case, the SSB contains 4 symbols of each 240 subcarriers (sc).
 * The goal here is to construct a reference grid that will allow, at the next step, to fill the SSB with teh 4 channels.
 * If the value of ref[channel][symbol][sc] is 1, it means that a symbol of this channel has to be placed at SSB[symbol][sc].
 *
* \standard TS38.211 V15.2.0 Section 7.4.3
* \param[in] num_sc_ssb. The number of sub-carrier (sc) in the SSB. In our case, this value is 240.
* \param[in] num_symbols_ssb. The number of symbols in the SSB. In our case, this value is 4.
 *\param[in] pci. Physical Cell Id. It is used to calculate DMRS index.
* \param[out] ref. A 3 dimensions table of int (0 or 1). ref[channel][symbol][subcarrier].
 *             In our case,     ref[0] -> index of PSS channel
 *                              ref[1] -> index of SSS channel
 *                              ref[2] -> index of PBCH channel
 *                              ref[3] -> index of DMRS channel
 *
 */


/** Loop over the symbols */
    for (int symbol = 0; symbol < num_symbols_ssb; symbol ++){
        /**Initialize the 3 dimensions table ref */
        ref[0][symbol] = new int [num_sc_ssb];
        ref[1][symbol] = new int [num_sc_ssb];
        ref[2][symbol] = new int [num_sc_ssb];
        ref[3][symbol] = new int [num_sc_ssb];

        /** Loop over the subcarriers */
        for (int sc = 0; sc < num_sc_ssb; sc++){

            /** Filling the ref table for each symbol and for each channel, according to TS38.211 V15.2.0 Section 7.4.3 */
            if (symbol == 0){
                if (sc >= free5GRAN::INTERVAL_SSB_PSS[0] && sc <= free5GRAN::INTERVAL_SSB_PSS[1]){
                    ref[0][symbol][sc] = 1;
                }else{
                    ref[0][symbol][sc] = 0;
                }
                ref[1][symbol][sc] = 0;
                ref[2][symbol][sc] = 0;
                ref[3][symbol][sc] = 0;
            }

            if (symbol == 1){
                ref[0][symbol][sc] = 0;
                ref[1][symbol][sc] = 0;
                if (sc % 4 != pci % 4){
                    ref[2][symbol][sc] = 1;
                    ref[3][symbol][sc] = 0;
                }else{
                    /** A DMRS symbol is placed every 4 symbols, beginning at pci%4 */
                    ref[2][symbol][sc] = 0;
                    ref[3][symbol][sc] = 1;
                }
            }

            if (symbol == 2){
                ref[0][symbol][sc] = 0;
                if (sc >= free5GRAN::INTERVAL_SSB_SSS[0] && sc <= free5GRAN::INTERVAL_SSB_SSS[1]){
                    ref[1][symbol][sc] = 1;
                    ref[2][symbol][sc] = 0;
                    ref[3][symbol][sc] = 0;
                }else {
                    ref[1][symbol][sc] = 0;
                }
                if (sc < free5GRAN::INTERVAL_SSB_NO_PBCH_DMRS[0] || sc > free5GRAN::INTERVAL_SSB_NO_PBCH_DMRS[1]){
                    if (sc % 4 != pci % 4){
                        ref[2][symbol][sc] = 1;
                        ref[3][symbol][sc] = 0;
                    }else{
                        /** A DMRS symbol is placed every 4 symbols, beginning at pci%4 */
                        ref[2][symbol][sc] = 0;
                        ref[3][symbol][sc] = 1;
                    }
                }else{
                    ref[2][symbol][sc] = 0;
                    ref[3][symbol][sc] = 0;
                }
            }

            if (symbol == 3){
                ref[0][symbol][sc] = 0;
                ref[1][symbol][sc] = 0;
                if (sc % 4 != pci % 4){
                    ref[2][symbol][sc] = 1;
                    ref[3][symbol][sc] = 0;
                }else{
                    /** A DMRS symbol is placed every 4 symbols, beginning at pci%4 */
                    ref[2][symbol][sc] = 0;
                    ref[3][symbol][sc] = 1;
                }
            }
        }
    }
    BOOST_LOG_TRIVIAL(info) << "function construct_reference_grid done";
}


void phy::channel_mapper(std::complex<float> **input_channels, int ***ref, std::complex<float> ** output_channels, int num_channels, int num_symbols, int num_sc){

    /**
* \fn ph_ben * channel_mapper (std::complex<float> **input_channels, int ***ref, std::complex<float> ** output_channels, int num_channels, int num_symbols, int num_sc)
* \brief This function aims to fill the SSB signal with the 4 channels (PSS, SSS, PBCH and DMRS) using the ref table constructed before.
* \standard TS38.211 V15.2.0 Section 7.4.3
*
* \param[in] std::complex<float> **input_channels. 2 dimensions table of complexes containing our input channels (in our case: PSS, SSS, PBCH and DMRS).
* \param[in] int ***ref. 3 dimensions table of int (1 or 0) that indicates the indexes of our channels.
* \param[in] int num_channels. Number of input channels. In our case, it is 4.
* \param[in] int num_symbols. Number of symbols on the output_channel (SSB). In our case, it is 4.
* \param[in] int num_sc. Number of subcarrier (sc) on the output_channel (SSB). In our case, it is 240.
* \param[out] std::complex<float> ** output_channels. 2 dimensions table of complexes. In our case, it is the SSB signal, composed of 4 symbols of each 240 subcarriers.
*/


    /** Initialize channel_counter for each channel */
    int *channel_counter = new int[num_channels];
    for (int channel =0; channel < num_channels; channel++){
        channel_counter[channel] = 0;
    }

    /** Loop over all channels (4 in our case) */
    for (int channel =0; channel < num_channels; channel ++){
        /** Loop over all symbols (4 in our case) */
        for (int symbol = 0; symbol < num_symbols; symbol ++){
            /** Loop over all subcarrier (240 in our case) */
            for (int sc = 0; sc < num_sc; sc++){

                if (ref[channel][symbol][sc] == 1){
                    output_channels[symbol][sc] = input_channels[channel][channel_counter[channel]];
                    channel_counter[channel] ++;
                }
            }
        }
    }
    BOOST_LOG_TRIVIAL(info) << "function channel_mapper done. It will give "+std::to_string(num_symbols)+ " * "+std::to_string(num_sc)+ " complex symbols";
}




void phy::increase_size_ssb(std::complex<float> ** input_channel, std::complex<float> ** output_channel, int num_symbols, int num_sc_input, int num_sc_output){
/**
* \fn ph_ben * increase_size_ssb (std::complex<float> ** input_channel, std::complex<float> ** output_channel, int num_symbols, int num_sc_input, int num_sc_output)
* \brief This function aims to increase the size of a signal, to prepare it for an IFFT (Inverse Fast Fourier Transform).
* \details
 * - In our case, 8 zero values are added at the beginning and at the end of the complex signal, making it 256 symbols long.
 * - As 256 is a power of 2, it will allow to make an ifft, taking this value as the fft size.
* \standard !! TS TO BE ADDED !!
* \param[in] std::complex<float> **input_channels. 2 dimensions table of complexes. In our case, it is the SSB signal (4*240 symbols)
* \param[in] int num_symbols. Number of symbols on the input and output channel. In our case, it is 4.
* \param[in] int num_sc_input. Number of subcarrier (sc) on the input_channel. In our case, it is 240.
* \param[in] int num_sc_output. Number of subcarrier (sc) on the output_channel. In our case, it is 256.
* \param[out] std::complex<float> ** output_channel. 2 dimensions table of complexes. In our case, it is the SSB signal extended (4*256 symbols)
*/

    /** Loop over all symbols */
    for (int symbol = 0; symbol < num_symbols; symbol++){
        int sc_in_counter = 0;
        /** Loop over all subcarriers of output signal */
        for (int sc_out = 0; sc_out < num_sc_output; sc_out++){
            if (sc_out < ((num_sc_output - num_sc_input)/2) || sc_out > num_sc_output - ((num_sc_output - num_sc_input)/2)){
                output_channel[symbol][sc_out] = {0,0};
            }else{
                output_channel[symbol][sc_out] = input_channel[symbol][sc_in_counter];
                sc_in_counter++;
            }
        }
    }
    BOOST_LOG_TRIVIAL(info) << "function increase_size_ssb done. It will give "+std::to_string(num_symbols)+ " * "+std::to_string(num_sc_output)+ " complex symbols";
}

void phy::reverse_ssb(std::complex<float> ** input_ssb, std::complex<float> ** output_reversed_ssb, int num_symbols, int num_sc){
/**
* \fn ph_ben * reverse_ssb (std::complex<float> ** input_ssb, std::complex<float> ** output_reversed_ssb, int num_symbols, int num_sc)
* \brief This function aims to inverse the 2 half of the SSB signal. This is done to put the frequency values at the right place, before making ifft.
* \param[in] std::complex<float> **input_ssb. 2 dimensions table of complexes. In our case, it is the SSB signal extended (4*256 symbols)
* \param[in] int num_symbols. Number of symbols on the input and output channel. In our case, it is 4.
* \param[in] int num_sc. Number of subcarriers (sc) on the input (and output) channel. In our case, it is 256.
* \param[out] std::complex<float> ** output_reversed_ssb. 2 dimensions table of complexes. In our case, it is the SSB signal extended and reversed(4*256 symbols)
*/

    /** Loop over all symbols */
    for (int symbol = 0; symbol < num_symbols; symbol++){
        int sc_counter1 = num_sc/2;
        int sc_counter2 = 0;
        /** Loop over all subcarriers of output signal */
        for (int sc = 0; sc < num_sc ; sc++){
            if (sc < num_sc/2){

                output_reversed_ssb[symbol][sc] = input_ssb[symbol][sc_counter1];
                //output_reversed_ssb[symbol][sc] = {1,1}; /** This line is here to make a test signal with 1 everywhere */
                sc_counter1++;
            }else{

                output_reversed_ssb[symbol][sc] = input_ssb[symbol][sc_counter2];
                //output_reversed_ssb[symbol][sc] = {1,1}; /** This line is here to make a test signal with 1 everywhere */
                sc_counter2++;
            }
        }
    }


    /** Divide each element by a facotr (here, it's 1000) to let the enhance the radio transmission */


    float dividing_factor = 200;
    for (int symbol = 0; symbol < num_symbols; symbol ++){
        for (int sc = 0; sc < num_sc; sc++){
            output_reversed_ssb[symbol][sc] = {(output_reversed_ssb[symbol][sc].real())/dividing_factor, (output_reversed_ssb[symbol][sc].imag())/dividing_factor};
        }
    }


    BOOST_LOG_TRIVIAL(info) << "function reverse_ssb done. It will give "+std::to_string(num_symbols)+ " * "+std::to_string(num_sc)+ " complex symbols";
}




void phy::ifft(std::complex<float> ** in_freq_domain_channel, std::complex<float> ** out_time_domain_channel, int fft_size, int sc_number) {
/**
* \fn ph_ben * ifft (std::complex<double> ** in_freq_domain_channel, std::complex<double> ** out_time_domain_channel, int fft_size, int sc_number)
* \brief This function aims to perform ifft (Inverse Fast Fourier Transform) to transform a frequency_domain signal into a time_domain signal. .
* \standard !! TS TO BE ADDED !!
* \param[in] std::complex<double> ** in_freq_domain_channel. 2 dimensions table of complexes. In our case, it is the SSB frequency signal extended (4*256 symbols)
* \param[in] int fft_size. Size of the fft. In our case, it is 256.
* \param[in] int sc_number. Number of sub_carrier per symbol in the input signal. In our case, it is 256.
* \param[out] std::complex<double> ** out_time_domain_channel. 2 dimensions table of complexes. In our case, it is the SSB time signal (4*256 symbols)
*/

    std::ofstream in_file_ben;
    in_file_ben.open("in_freq_ssb_ben.txt");

    /** Loop over all symbols */
    for (int symbol = 0; symbol < free5GRAN::NUM_SYMBOLS_SSB; symbol++) {


        /** Generate complex arrays to store IFFT signals */

        fftw_complex *signal_in = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * fft_size);
        fftw_complex *signal_out = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * fft_size);

        /** Generate plans */
        fftw_plan ifft_plan = fftw_plan_dft_1d(fft_size, signal_in, signal_out, FFTW_BACKWARD, FFTW_MEASURE);



        /** Initialize arrays */
        for (int i = 0; i < fft_size; i++) {
            signal_in[i][0] = 0;
            signal_out[i][1] = 0;
        }


        /** Filling signal_in with the input signal */

        for (int sc = 0; sc < sc_number; sc++) {
            signal_in[sc][0] = in_freq_domain_channel[symbol][sc].real();
            signal_in[sc][1] = in_freq_domain_channel[symbol][sc].imag();
        }



        /** Filling a txt file in_freq_ssb_ben to verify with signal_in for only 1 symbol */

        for (int sc = 0; sc < free5GRAN::SIZE_IFFT_SSB; sc++) {
            in_file_ben << in_freq_domain_channel[symbol][sc];
            in_file_ben << "\n";
        }

        in_file_ben.close();


        /** Execute the IFFT */
        fftw_execute(ifft_plan);


        /** Filling output signal with signal_out*/
        for (int sc = 0; sc < sc_number; sc++) {
            out_time_domain_channel[symbol][sc] = {signal_out[sc][0], signal_out[sc][1]};
        }
    }


    /** Filling a txt file to verify the ifft, for all symbols */
    std::ofstream out_file_ben;
    out_file_ben.open("out_time_ssb_ben.txt");
    for (int symbol = 0; symbol < free5GRAN::NUM_SYMBOLS_SSB; symbol ++){
        for (int sc = 0; sc < free5GRAN::SIZE_IFFT_SSB; sc++) {
            out_file_ben << out_time_domain_channel[symbol][sc];
            out_file_ben << "\n";
        }
    }
    out_file_ben.close();
    BOOST_LOG_TRIVIAL(info) << "function ifft done. It will give "+std::to_string(free5GRAN::NUM_SYMBOLS_SSB)+ " * "+std::to_string(free5GRAN::SIZE_IFFT_SSB)+ " complex symbols";
}


void phy::compute_cp_lengths(int scs, int nfft, int is_extended_cp, int num_symb_per_subframes, int *cp_lengths, int *cum_sum_cp_lengths){
    /**
     *
     * \fn compute_cp_lengths (int scs, int nfft, int is_extended_cp, int num_symb_per_subframes, int *cp_lengths, int *cum_sum_cp_lengths)
     * \brief Compute cyclic prefix size
     * \standard TS38.211 V15.2.0 Section 5.3
     * \param[in] scs: Subcarrier spacing
     * \param[in] nfft: FFT/iFFT size
     * \param[in] is_extended_cp: True if current CP is extended CP
     * \param[in] num_symb_per_subframes: Number of symbols per subframe
     * \param[out] cp_lengths: Returned CP
     * \param[out] cum_sum_cp_lengths: Cumulative CP sum
    */
    int nom_cp = ((is_extended_cp) ? 512 :  144);
    int base_cp = nom_cp * nfft / 2048;
    cum_sum_cp_lengths[0] = 0;
    for (int i = 0; i < num_symb_per_subframes; i ++){
        if (i % (num_symb_per_subframes / 2) == 0){
            cp_lengths[i] = (scs * nfft - num_symb_per_subframes * nfft - (num_symb_per_subframes - 2) * base_cp) / 2;
        }else {
            cp_lengths[i] = base_cp;
        }
        if (i < num_symb_per_subframes - 1){
            cum_sum_cp_lengths[i + 1] = cum_sum_cp_lengths[i] + cp_lengths[i] + nfft;
        }
    }
    BOOST_LOG_TRIVIAL(info) << "function compute_cp_length done.";
    display_table(cp_lengths, num_symb_per_subframes, "cp_lengths");
}





void phy::adding_cp(std::complex<float> ** input_channel, int num_symbols, int num_sc_in, int cp_lengths, std::complex<float> ** output_channel_with_cp){
    /**
        * \fn phy * adding_cp (std::complex<float> ** input_channel, int num_symbols, int num_sc_in, int cp_lengths, std::complex<float> ** output_channel_with_cp)
        * \brief This function aims to add the CP (Cyclic Prefix) to the SSB signal (time_domain).
        * \standard !! TS TO BE ADDED !!
        * \param[in] std::complex<float> ** input_channel. In our Case, it is the SSB (time domain) (4*256 symbols)
        * \param[in] int num_symbols. Number of symbol in the input_channel. In our case it's 4.
        * \param[in] int num_sc_in. Number of elements in each symbols. In our case, it is 256/
        * \param[in] int cp_lengths. Number of symbols the Cyclic Prefix should contain
        * \param[out] std::complex<float> ** output_channel_with_cp. In our case, it is the SSB with CP.
       */

    /** Loop over all symbols */
    for (int symbol = 0; symbol < num_symbols; symbol++ ) {
        /** Loop over all subcarriers */
        for (int sc_out = 0; sc_out < num_sc_in + cp_lengths; sc_out++) {
            if (sc_out < cp_lengths) {
                output_channel_with_cp[symbol][sc_out] = input_channel[symbol][num_sc_in - cp_lengths + sc_out];
            }else{
                output_channel_with_cp[symbol][sc_out] = input_channel[symbol][sc_out - cp_lengths];
            }
        }
    }


    /** Filling a txt file to verify our ssb_time_cp */
    std::ofstream out_file_ben;
    out_file_ben.open("time_ssb_ben_cp.txt");
    for (int symbol = 0; symbol < free5GRAN::NUM_SYMBOLS_SSB; symbol ++){
        for (int sc = 0; sc < free5GRAN::SIZE_IFFT_SSB + cp_lengths; sc++) {
            out_file_ben << output_channel_with_cp[symbol][sc];
            out_file_ben << "\n";
        }
    }
    out_file_ben.close();
    BOOST_LOG_TRIVIAL(info) << "function adding_cp done. It will give "+std::to_string(free5GRAN::NUM_SYMBOLS_SSB)+ " * "+std::to_string(free5GRAN::SIZE_IFFT_SSB + cp_lengths)+ " complex symbols";
}







//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------

/** Bellow are some little functions that are used in the functions above */



int * phy::convert_pci_into_nid2_and_nid1 (int pci) {

    /**
* \fn ph_ben * convert_pci_into_nid2_and_nid1 (int pci)
* \brief This function takes the Physical Cell Id (pci) as enter and return the array n_id which contains n_id_1 and n_id_2.
* \standard TS38.211 V15.2.0 Section 7.4.2.1
*
* \param[in] pci Physical Cell Id
*/
    int n_id_1;
    int n_id_2;
    n_id_2 = pci % 3;
    n_id_1 = (pci - n_id_2)/3;

    int * n_id = new int[2];
    n_id[0] = n_id_1;
    n_id[1] = n_id_2;
    return n_id;
}


void phy::display_signal_float(std::complex<float> ** signal_to_display, int num_symbols, int num_sc, char* signal_name){
    for (int symbols = 0; symbols < num_symbols; symbols++){
        std::cout<<""<<std::endl;
        std::cout<<""<<std::endl;
        std::cout<<signal_name<< "of symbol "<<symbols<<" = "<<std::ends;
        display_complex_float(signal_to_display[symbols], num_sc, "");
    }
}


void phy::display_vector(std::vector<std::complex<float>> vector_to_display, int vector_size, char* vector_name){
    /**
* \fn ph_ben * display_vector (std::vector<std::complex<float>> *vector_to_display, int vector_size, char* vector_name)
* \brief This function aims to display a vector in the console, using the command std::cout.
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
        if (i% 10 == 0){         /** 10 here means that every 10 elements displayed, a line break is done */
            std::cout <<""<< std::endl;
        }
        std::cout<<vector_to_display[i]<<"  "<< std::ends;
    }

}


void phy::display_complex_double(std::complex<double> *vector_to_display, int vector_size, char* vector_name){

    /**
* \fn ph_ben * display_complex_double (std::complex<double> *vector_to_display, int vector_size, char* vector_name)
* \brief This function aims to display a vector in the console, using the command std::cout.
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

void phy::display_complex_float(std::complex<float> *vector_to_display, int vector_size, char* vector_name){

    /**
* \fn ph_ben * display_complex_float (std::complex<float> *vector_to_display, int vector_size, char* vector_name)
* \brief This function aims to display a vector in the console, using the command std::cout.
*
* \param[in] vector_to_display
* \param[in] vector_size number of element in the vector
* \param[in] vector_name name to display
*/
    for (int i=0; i<vector_size; i++){
        if (i % 1000 == 0){        /** 10 here means that every 10 elements displayed, a line break is done */
            std::cout <<""<< std::endl;
        }
        if (i == 0){
            std::cout << +vector_name << ": "<<std::ends;
        }
        std::cout<<vector_to_display[i] <<"   "<< std::ends;
    }
}


void phy::display_table(int* table_to_display, int size, char* table_name) {

    /**
   * \fn ph_ben * display_table (int* table_to_display, int size, char* table_name)
   * \brief This function aims to display a table in the console, using the command std::cout.
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


void phy::convert_decimal_to_binary(int size, int decimal, int* table_output) {
    /**
      * \fn ph_ben * convert_decimal_to_binary (int size, int decimal, int* table_output)
      * \brief This function aims to convert an integer number into a binary bit sequence.
      *
      * \param[in] size Indicates the number of bits in the output sequence. Please verify that decimal <= 2^size.
      * \param[in] decimal the number to convert into a bit sequence.
      * \param[out] table_output the output bits sequence. Will contain only 1 and 0.
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


//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------

/** In this section, big functions are created using the little functions above. */
/** This aims to reduce the length of main_ben.cpp */

void phy::encode_bch(int * mib_bits, int pci, int N, int * rate_matched_bch){
    /**
      * \fn ph_ben * encode_bch(int * mib_bits, int pci, int N, int * rate_matched_bch)
      * \brief This function aims to transform the mib_bits into a rate_matched_bch bits sequence.
      * \details Here are the steps of this function: INTERLEAVING, SCRAMBLING, ADDING CRC, POLAR ENCODING and RATE MATCHING.
      * \standard TS38.212 V15.2.0 Section 7.1
      * \standard TS38.212 V15.2.0 Section 5
      *
      * \param[in] mib_bits. In our case, it is a 32 long bits sequence.
      * \param[in] pci. Physical Cell ID.
      * \param[in] N. It is the length of the BCH payload after polar encode. It's a power of 2. In our case, it's 512.
      * \param[out] rate_matched_bch. The output bits sequence. 864 bits long in our case.
      */

    /** INTERLEAVING -> Generating mib_bits_interleaved (32 bits long in our case) from mib_bits. TS38.212 V15.2.0 Section 7.1.1 */
    int mib_bits_interleaved[free5GRAN::BCH_PAYLOAD_SIZE];
    bch_interleaving(mib_bits, mib_bits_interleaved);

    if(display_variable){
        display_table(mib_bits_interleaved, free5GRAN::BCH_PAYLOAD_SIZE, "mib_bits_interleaved from phy");}

    /** SCRAMBLING -> Generating bch_payload (32 bits long in our case) from mib_bits_interleaved. TS38.212 V15.2.0 Section 7.1.2 */
    int bch_payload[free5GRAN::BCH_PAYLOAD_SIZE];
    int v = mib_bits[25] * 2 + mib_bits[26]; /** v depends on the SFN value (3rd LSB of SFN and 2nd LSB of SFN). */
    if (display_variable){std::cout <<"v (depends on SFN, 3rd and 2nd LSB "<<v<< std::endl;}
    scrambling_bch(v, pci, mib_bits_interleaved, bch_payload);

    if (display_variable){
        display_table(bch_payload, free5GRAN::BCH_PAYLOAD_SIZE, "bch_payload from phy");}

    /** CRC -> Generating bch_payload_with_crc (56 bits long in our case) from bch_payload. TS38.212 V15.2.0 Section 5.1 */
    int bch_payload_with_crc[free5GRAN::SIZE_PBCH_POLAR_DECODED];
    adding_crc(bch_payload, bch_payload_with_crc);
    if (display_variable){
        display_table(bch_payload_with_crc, free5GRAN::SIZE_PBCH_POLAR_DECODED, "bch_payload_with_crc from phy");}

    /** POLAR ENCODING -> Generating polar encoded_bch (512 bits long in our case) from bch_payload_with_crc. TS38.212 V15.2.0 Section 5.3.1 */
    int *polar_encoded_bch = new int[N];
    polar_encode_bch(N, bch_payload_with_crc, polar_encoded_bch);
    if (display_variable){
        display_table(polar_encoded_bch, N, "polar_encoded_bch from phy");}

    /** RATE MATCHING -> Generating rate_matching_bch (864 bits long in our case) from encoded_bch. TS38.212 V15.2.0 Section 5.4.1 */
    rate_matching(polar_encoded_bch, rate_matched_bch);
    if (display_variable){
        display_table(rate_matched_bch, free5GRAN::SIZE_SSB_PBCH_SYMBOLS*2, "rate_matched_bch from phy");}

}

void phy::encode_pbch_and_modulation(int * rate_matched_bch, int pci, int gscn, int i_b_ssb, std::complex<float> * pbch_symbols2){
    /**
    * \fn ph_ben * encode_pbch_and_modulation (int * rate_matched_bch, int pci, int gscn, int i_b_ssb, std::complex<float> * pbch_symbols2)
    * \brief This function aims to transform a rate_matched_bch bits sequence into a pbch symbols sequence.
    * \details The 2 main steps are ENCODING and MODULATION.
    * \standard TS38.211 V15.2.0 Section 7.3.3.1
    * \standard TS38.211 V15.2.0 Section 5.1.3
    *
    * \param[in] rate_matched_bch. In our case, it is a 864 long bits sequence.
    * \param[in] pci. Physical Cell ID.
    * \param[in] gscn. Global Synchronization Channel Number.
    * \param[in] i_b_ssb. It is the SSB index. Should be between 0 and 7.
    * \param[out] pbch_symbols2. The output symbols sequence. 432 symbols long in our case.
    */

    /** ENCODING -> Generating encoded_pbch (864 bits long in our case) from rate_matching_bch. TS38.211 V15.2.0 Section 7.3.3.1 */
    int *encoded_pbch = new int[free5GRAN::SIZE_SSB_PBCH_SYMBOLS*2];
    encode_pbch(gscn, pci, i_b_ssb, rate_matched_bch, encoded_pbch);
    if (display_variable){
        display_table(encoded_pbch, free5GRAN::SIZE_SSB_PBCH_SYMBOLS*2, "encoded_pbch from phy");}

    /** MODULATION -> Generating pbch_symbols2 (432 symbols long in our case) from encoded_pbch, using BPSK or QPSK. TS38.211 V15.2.0 Section 5.1.3 */

    modulation(encoded_pbch, free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2, 1, pbch_symbols2);
    if (display_variable){
        display_complex_float(pbch_symbols2, free5GRAN::SIZE_SSB_PBCH_SYMBOLS, "pbch_symbols2 from phy");}

}

void phy::generate_SSB_time(std::complex<float> * pbch_symbols2, int pci, int i_b_ssb, free5GRAN::mib mib_object, std::complex<float> ** SSB_signal_time_domain){
    /**
     * \fn ph_ben * generate_SSB_time (std::complex<float> * pbch_symbols2, int pci, int i_b_ssb, free5GRAN::mib mib_object, std::complex<float> ** SSB_signal_time_domain)
     * \brief This function aims to generate from a pbch sequence a SSB (Synchronization Signal Block), without Cyclic Prefix, in time domain.
     * \standard TS38.211 V15.2.0 Section 7.4
     * \param[in] pbch_symbols2. In our case, it is a 432 symbols sequence.
     * \param[in] pci. Physical Cell ID.
     * \param[in] i_b_ssb. It is the SSB index. Should be between 0 and 7.
     * \param[in] mib_object. The Master Information Blovk.
     * \param[out] SSB_signal_time_domain. In our case, it is a signal composed of 4*256 elements.
     */


    /** DMRS -> Generating dmrs_symbols (144 symbols long in our case) from pci and i_b_ssb. TS38.211 V15.2.0 Section 7.4.1.4.1 */
    std::complex<float> *dmrs_symbols;
    dmrs_symbols = new std::complex<float>[free5GRAN::SIZE_SSB_DMRS_SYMBOLS];
    generate_dmrs_of_pbch(pci, i_b_ssb, dmrs_symbols);
    if (display_variable){
        display_complex_float(dmrs_symbols, free5GRAN::SIZE_SSB_DMRS_SYMBOLS,
                              "dmrs_symbols from phy");}

    /** N_ID_1 & N_ID_2 -> Computing n_id_1 and n_id_2 from the pci. TS38.211 V15.2.0 Section 7.4.2.1 */
    int * n_id = new int[2];
    n_id = convert_pci_into_nid2_and_nid1(pci);
    int n_id_1 = n_id[0];
    int n_id_2 = n_id[1];
    if (display_variable) {std::cout<<"n_id_1 = "<< n_id_1 <<"; n_id_2 = "<< n_id_2 <<std::endl;}

    /** PSS -> Computing pss_sequence_symbols (127 symbols long in our case) from n_id_2. TS38.211 V15.2.0 Section 7.4.2.2.1 */
    int * pss_sequence_symbols= new int[free5GRAN::SIZE_PSS_SSS_SIGNAL];
    free5GRAN::utils::sequence_generator::generate_pss_sequence(n_id_2, pss_sequence_symbols);
    if (display_variable){
        display_table(pss_sequence_symbols, free5GRAN::SIZE_PSS_SSS_SIGNAL, "pss_sequence_symbols");}

    /** CONVERTING PSS -> Converting PSS sequence element from int to complex<float> (Imaginary part = 0) */
    std::complex<float> *pss_complex_symbols;
    pss_complex_symbols = new std::complex<float>[free5GRAN::SIZE_PSS_SSS_SIGNAL];
    for (int i=0; i<free5GRAN::SIZE_PSS_SSS_SIGNAL; i++){
        pss_complex_symbols[i] = {static_cast<float>(pss_sequence_symbols[i]), 0};
    }
    if (display_variable){
        display_complex_float(pss_complex_symbols, free5GRAN::SIZE_PSS_SSS_SIGNAL, "pss_complex_symbols");}


    /** SSS -> Computing sss_sequence_symbols (127 symbols long in our case) from n_id_1. TS38.211 V15.2.0 Section 7.4.2.3.1 */
    int * sss_sequence_symbols= new int[free5GRAN::SIZE_PSS_SSS_SIGNAL];
    free5GRAN::utils::sequence_generator::generate_sss_sequence(n_id_1, n_id_2, sss_sequence_symbols);
    if (display_variable){
        display_table(sss_sequence_symbols, free5GRAN::SIZE_PSS_SSS_SIGNAL, "sss_sequence_symbols");}

    /** CONVERTING SSS -> Converting SSS sequence element from int to complex<float> (Imaginary part = 0) */
    std::complex<float> *sss_complex_symbols;
    sss_complex_symbols = new std::complex<float>[free5GRAN::SIZE_PSS_SSS_SIGNAL];
    for (int i=0; i<free5GRAN::SIZE_PSS_SSS_SIGNAL; i++){
        sss_complex_symbols[i] = {static_cast<float>(sss_sequence_symbols[i]), 0};
    }
    if (display_variable){
        display_complex_float(sss_complex_symbols, free5GRAN::SIZE_PSS_SSS_SIGNAL, "sss_complex_symbols");}



    /** REFERENCE GRID -> Building reference grid ref to then fill the SSB correctly, according to TS38.211 V15.2.0 Section 7.4.3 */
    int *** ref;
    ref = new int **[4]; /** There are 4 channels */
    ref[0] = new int *[free5GRAN::NUM_SYMBOLS_SSB];
    ref[1] = new int *[free5GRAN::NUM_SYMBOLS_SSB];
    ref[2] = new int *[free5GRAN::NUM_SYMBOLS_SSB];
    ref[3] = new int *[free5GRAN::NUM_SYMBOLS_SSB];

    construct_reference_grid(free5GRAN::NUM_SC_SSB, free5GRAN::NUM_SYMBOLS_SSB, pci, ref);

    /** DISPLAY ref */

    if (display_variable) {
        for (int channel = 0; channel < 4; channel++) {
            std::cout << "" << std::endl;
            for (int symbol = 0; symbol < free5GRAN::NUM_SYMBOLS_SSB; symbol++) {
                std::cout << "" << std::endl;
                std::cout << "ref channel " << channel << " symbol " << symbol << " = " << std::ends;
                for (int sc = 0; sc < free5GRAN::NUM_SC_SSB; sc++) {
                    if (sc % 50 == 0) {
                        std::cout << " ____ " << std::ends;
                    }
                    std::cout << ref[channel][symbol][sc] << " " << std::ends;
                }
            }
        }
    }

    /** CHANNEL MAPPING --> Fill the SSB with PSS, SSS, PBCH and DMRS, using ref and according to TS38.211 V15.2.0 Section 7.4.3 */
    std::complex<float> ** SSB_signal_freq_domain;
    SSB_signal_freq_domain = new std::complex<float> *[free5GRAN::NUM_SYMBOLS_SSB];
    SSB_signal_freq_domain[0] = new std::complex<float> [free5GRAN::NUM_SC_SSB];
    SSB_signal_freq_domain[1] = new std::complex<float> [free5GRAN::NUM_SC_SSB];
    SSB_signal_freq_domain[2] = new std::complex<float> [free5GRAN::NUM_SC_SSB];
    SSB_signal_freq_domain[3] = new std::complex<float> [free5GRAN::NUM_SC_SSB];

    channel_mapper(new std::complex<float>*[4]{pss_complex_symbols, sss_complex_symbols, pbch_symbols2, dmrs_symbols}, ref, SSB_signal_freq_domain, 4, free5GRAN::NUM_SYMBOLS_SSB, free5GRAN::NUM_SC_SSB);
    if (display_variable){
        display_signal_float(SSB_signal_freq_domain, free5GRAN::NUM_SYMBOLS_SSB, free5GRAN::NUM_SC_SSB, "SSB_signal_freq_domain from phy");}


    /** SSB FROM 240 TO 256 SYMBOLS */
    std::complex<float> ** SSB_signal_extended;
    SSB_signal_extended = new std::complex<float> *[free5GRAN::NUM_SYMBOLS_SSB];
    SSB_signal_extended[0] = new std::complex<float> [free5GRAN::SIZE_IFFT_SSB];
    SSB_signal_extended[1] = new std::complex<float> [free5GRAN::SIZE_IFFT_SSB];
    SSB_signal_extended[2] = new std::complex<float> [free5GRAN::SIZE_IFFT_SSB];
    SSB_signal_extended[3] = new std::complex<float> [free5GRAN::SIZE_IFFT_SSB];

    increase_size_ssb(SSB_signal_freq_domain, SSB_signal_extended, free5GRAN::NUM_SYMBOLS_SSB, free5GRAN::NUM_SC_SSB, free5GRAN::SIZE_IFFT_SSB);
    if (display_variable){
        display_signal_float(SSB_signal_extended, free5GRAN::NUM_SYMBOLS_SSB, free5GRAN::SIZE_IFFT_SSB, "SSB_signal_extended from phy");}

    /** REVERSE SSB */
    std::complex<float> ** SSB_signal_extended_reversed;
    SSB_signal_extended_reversed = new std::complex<float> *[free5GRAN::NUM_SYMBOLS_SSB];
    SSB_signal_extended_reversed[0] = new std::complex<float> [free5GRAN::SIZE_IFFT_SSB];
    SSB_signal_extended_reversed[1] = new std::complex<float> [free5GRAN::SIZE_IFFT_SSB];
    SSB_signal_extended_reversed[2] = new std::complex<float> [free5GRAN::SIZE_IFFT_SSB];
    SSB_signal_extended_reversed[3] = new std::complex<float> [free5GRAN::SIZE_IFFT_SSB];

    reverse_ssb(SSB_signal_extended, SSB_signal_extended_reversed, free5GRAN::NUM_SYMBOLS_SSB, free5GRAN::SIZE_IFFT_SSB);
    if (display_variable){
        display_signal_float(SSB_signal_extended_reversed, free5GRAN::NUM_SYMBOLS_SSB, free5GRAN::SIZE_IFFT_SSB, "SSB_signal_extended_reversed from phy");}


    /**IFFT --> SSB from frequency domain to time domain */
    ifft(SSB_signal_extended_reversed, SSB_signal_time_domain, free5GRAN::SIZE_IFFT_SSB, free5GRAN::SIZE_IFFT_SSB);
    if (display_variable){
        display_signal_float(SSB_signal_time_domain, free5GRAN::NUM_SYMBOLS_SSB, free5GRAN::SIZE_IFFT_SSB, "SSB_signal_time_domain from phy");}

}


//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------







/** FROM HERE, THE CODE AIMS TO DECODE INFORMATIONS AND NOT TO ENCODE IT ANYMORE (AYMERIC's CODE) */
/** IT IS ONLY HERE TO VERIFY THAT WHAT WE ENCODE IS POSSIBLE TO BE DECODED CORRECTLY */


std::vector<std::complex<float>> phy::AY_extract_pbch(std::complex<float> ** input_SSB, int pci){


    int fft_size = 256;
    int cp_length = 18;
    int symbol_duration = fft_size + cp_length; // = 274
    float max_snr;
    int l_max = 4;  //TO BE VERIFIED !!


    std::vector<std::complex<float>> pbch_modulation_symbols(free5GRAN::SIZE_SSB_PBCH_SYMBOLS);
    std::vector<std::complex<float>> final_pbch_modulation_symbols(free5GRAN::SIZE_SSB_PBCH_SYMBOLS);

    // Initializing fft parameters
    fftw_complex *fft_in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * fft_size);
    fftw_complex *fft_out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * fft_size);

    fftw_plan fft_plan = fftw_plan_dft_1d(fft_size, fft_in, fft_out, FFTW_FORWARD, FFTW_MEASURE);


    //Extracting DMRS AND PBCH modulation symbols


    std::vector<std::complex<float>> temp_mod_symbols, temp_mod_symbols2,temp_mod_symbols_dmrs;
    std::complex<float> *pbch_symbols_ay, *dmrs_symbols;
    pbch_symbols_ay = new std::complex<float>[free5GRAN::SIZE_SSB_PBCH_SYMBOLS];
    dmrs_symbols = new std::complex<float>[free5GRAN::SIZE_SSB_DMRS_SYMBOLS];

    //ref[0] -> indexes of PBCH resource elements
    //ref[1] -> indexes of DMRS resource elements


    int ** dmrs_indexes, ** pbch_indexes, ***ref;
    ref = new int **[2];
    ref[0] = new int *[free5GRAN::NUM_SYMBOL_PBCH_SSB]; // = [3]
    ref[1] = new int *[free5GRAN::NUM_SYMBOL_PBCH_SSB]; // = [3]
    dmrs_indexes = new int *[2];
    dmrs_indexes[0] = new int[free5GRAN::SIZE_SSB_DMRS_SYMBOLS]; // = [144]
    dmrs_indexes[1] = new int[free5GRAN::SIZE_SSB_DMRS_SYMBOLS]; // = [144]
    pbch_indexes = new int *[2];
    pbch_indexes[0] = new int[free5GRAN::SIZE_SSB_PBCH_SYMBOLS]; // = [432]
    pbch_indexes[1] = new int[free5GRAN::SIZE_SSB_PBCH_SYMBOLS]; // = [432]


    //Converting input_SSB (double) into vector_input_SSB (float)
    std::vector<std::complex<float>> vector_input_SSB(4 * symbol_duration);
    for (int symbol = 0; symbol < 4; symbol++){
        for (int sc = 0; sc < symbol_duration; sc++){
            vector_input_SSB[symbol * symbol_duration + sc] = input_SSB[symbol][sc];
        }
    }

    std::complex<float> **ssb_symbols = new std::complex<float> *;
    *ssb_symbols = new std::complex<float> [free5GRAN::NUM_SYMBOLS_SSB - 1]; // = [3]
    //Looping over 3 OFDM symbols containing DMRS and PBCH

    /** Below, some modifications to pass from table to vector */

    vector<vector<complex<float>>> ssb_symbols_vector(free5GRAN::NUM_SYMBOLS_SSB - 1, vector<complex<float>>(free5GRAN::NUM_SC_SSB));

    vector<vector<vector<int>>> ref_vector(2, vector<vector<int>>(free5GRAN::SIZE_SSB_DMRS_SYMBOLS, vector<int>(free5GRAN::NUM_SC_SSB)));

    int agg_level;

    vector<vector<vector<int>>> channel_indexes = {vector<vector<int>>(2, vector<int>(agg_level * free5GRAN::NUMBER_REG_PER_CCE * 9)), vector<vector<int>>(2, vector<int>(agg_level * free5GRAN::NUMBER_REG_PER_CCE * 3))};

    int lrb, L;

    vector<vector<complex<float>>> coefficients_vector(L, vector<complex<float>>(12 * lrb));

    /** ################################# */


    for (int symbol = 1; symbol < free5GRAN::NUM_SYMBOLS_SSB; symbol ++){ // loop symbol = 1, symbol = 2 and symbol = 3
        ssb_symbols[symbol-1] = new std::complex<float>[free5GRAN::NUM_SC_SSB]; // = [240]
        ref[0][symbol - 1] = new int [free5GRAN::NUM_SC_SSB]; // = [240]
        ref[1][symbol - 1] = new int [free5GRAN::NUM_SC_SSB]; // = [240]
        // Filling fft input signal with current symbol IQ
        for (int i = 0; i < fft_size; i++){
            fft_in[i][0] = std::real(vector_input_SSB[i + symbol * symbol_duration + cp_length]);
            fft_in[i][1] = std::imag(vector_input_SSB[i + symbol * symbol_duration + cp_length]);
        }
        // Execute the fft
        fftw_execute(fft_plan);
        // Building the RE grid from 0 to 239 from the 256 sized fft output
        for (int i = 0; i < free5GRAN::NUM_SC_SSB / 2; i++){ // loop from i = 0 to 119
            ssb_symbols[symbol-1][free5GRAN::NUM_SC_SSB / 2 + i ] = std::complex<float>(fft_out[i][0],fft_out[i][1]);
            ssb_symbols[symbol-1][free5GRAN::NUM_SC_SSB / 2 - i - 1] = std::complex<float>(fft_out[fft_size - i - 1][0],fft_out[fft_size - i - 1][1]);
        }
        // Creating Resource element de-mapper reference grid
        for (int i = 0; i < free5GRAN::NUM_SC_SSB; i++){
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
            }
        }
    }


    //free5GRAN::phy::signal_processing::channel_demapper(ssb_symbols, ref, new int[2]{free5GRAN::SIZE_SSB_PBCH_SYMBOLS,free5GRAN::SIZE_SSB_DMRS_SYMBOLS}, new std::complex<float>*[2] {pbch_symbols_ay,dmrs_symbols}, new int **[2] {pbch_indexes,dmrs_indexes}, 2, free5GRAN::NUM_SYMBOL_PBCH_SSB, free5GRAN::NUM_SC_SSB);
    free5GRAN::phy::signal_processing::channel_demapper(ssb_symbols_vector, ref_vector,new std::complex<float>*[2] {pbch_symbols_ay,dmrs_symbols}, channel_indexes, 2, free5GRAN::NUM_SYMBOL_PBCH_SSB, free5GRAN::NUM_SC_SSB);

    //Channel estimation and equalization
    //Creating coefficients arrays


    std::complex<float> **coefficients[free5GRAN::MAX_I_BAR_SSB];
    for (int p = 0; p < free5GRAN::MAX_I_BAR_SSB; p ++){
        coefficients[p] = new std::complex<float> * [free5GRAN::NUM_SYMBOL_PBCH_SSB];
        for (int i = 0 ; i < free5GRAN::NUM_SYMBOL_PBCH_SSB; i ++){
            coefficients[p][i] = new std::complex<float> [free5GRAN::NUM_SC_SSB];
        }
    }

    std::complex<float> * dmrs_sequence;
    float snr[free5GRAN::MAX_I_BAR_SSB]; // = [8]


//For each possible iBarSSB value, estimate the corresponding channel


    for (int i = 0; i < free5GRAN::MAX_I_BAR_SSB; i ++){
        dmrs_sequence = new std::complex<float>[free5GRAN::SIZE_SSB_DMRS_SYMBOLS];
        free5GRAN::utils::sequence_generator::generate_pbch_dmrs_sequence(pci,i,dmrs_sequence);
        free5GRAN::phy::signal_processing::channelEstimation(dmrs_symbols, dmrs_sequence, channel_indexes[1],coefficients_vector, snr[i], free5GRAN::NUM_SC_SSB, free5GRAN::NUM_SYMBOL_PBCH_SSB , free5GRAN::SIZE_SSB_DMRS_SYMBOLS);
    }

//Choose the iBarSSB value that maximizes the SNR


    max_snr = snr[0];
    int i_b_ssb = 0;
    for (int i = 1; i < free5GRAN::MAX_I_BAR_SSB ; i ++){
        if (snr[i] > max_snr){
            max_snr = snr[i];
            i_b_ssb = i;
        }
    }

// Equalize the channel
    std::vector<std::complex<float>> pbch_symbols = {};

    for (int i = 0; i < free5GRAN::SIZE_SSB_PBCH_SYMBOLS; i ++){ // = [432]

//pbch_symbols.push_back({8,7});
//pbch_symbols[i] = {8,7};
        pbch_symbols.push_back(  (pbch_symbols_ay[i]) * std::conj(coefficients[i_b_ssb][pbch_indexes[0][i]][pbch_indexes[1][i]]) / (float) pow(std::abs(coefficients[i_b_ssb][pbch_indexes[0][i]][pbch_indexes[1][i]]),2)   );
    }
//pbch_symbols.push_back({3,3});
/*
*/
    return pbch_symbols;


    /*
    this->i_b_ssb = i_b_ssb;
    //this-> i_ssb = i_b_ssb%4 ;
    if (l_max == 4){
        this-> i_ssb = i_b_ssb % 4;
    }else {
        this-> i_ssb = i_b_ssb;
    }
    this->pci = pci;

    /*
     * Setting pbch symbols to pbch object and running decoding process

    this->pbch_symbols_ay = final_pbch_modulation_symbols;
    //return 0;
     */
}





int * phy::AY_decode_pbch(int pci, std::vector<std::complex<float>> pbch_symbols) {
    /*
    std::ofstream out_file, out_bits, out_scrambled;
    out_file.open("out_pbch.txt");
    out_bits.open("out_bits.txt");
    out_scrambled.open("out_scrambled.txt");

    for (auto modulation_symbol : pbch_symbols2){
        out_file << modulation_symbol;
        out_file << "\n";
    }
    out_file.close();
    */
    int * bch_bits5 = new int[free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2];
    int * pbch_bits;
    pbch_bits = new int[free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2];
    /*
     * Demodulate PBCH Signal
     */
    free5GRAN::phy::signal_processing::hard_demodulation(pbch_symbols, pbch_bits, free5GRAN::SIZE_SSB_PBCH_SYMBOLS, 1);

    if (display_variable){
        display_table(pbch_bits, free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2, "AY_pbch_bits");}

    // Generate de-scrambling sequence
    //c_sequence seq_object = c_sequence(pci, free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2 * (1 + i_b_ssb) );
    //int * c_seq = seq_object.get_sequence();
    int * c_seq = new int[free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2 * (1 + i_ssb)];
    free5GRAN::utils::sequence_generator::generate_c_sequence(pci, free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2 * (1 + i_ssb), c_seq, 0);

    //int * scrambled_bits;
    //scrambled_bits = new int[free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2];

    /*
     * De-scramble pbch_bits to scrambled_bits
     */
    free5GRAN::utils::common_utils::scramble(pbch_bits, c_seq, bch_bits5, free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2, i_ssb * free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2);
    if (display_variable){
        display_table(bch_bits5, 864, "AY_bch_bits5 from phy");}
    return bch_bits5;
    /*
    for (int i = 0; i < free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2; i ++){
        out_bits << pbch_bits[i];
        out_bits << "\n";
        out_scrambled << bch_bits[i];
        out_scrambled << "\n";
    }
    out_bits.close();
    out_scrambled.close();
*/
}





void phy::AY_decode_bch(int* bch_bits, int pci, int* mib_bits) {

    char* table_name8 = "AY_bch_bits";
    if (display_variable){
        display_table(bch_bits, 864, table_name8);}

    int n = free5GRAN::phy::transport_channel::compute_N_polar_code(free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2,free5GRAN::SIZE_PBCH_POLAR_DECODED,9);
    int N = pow(2,n);
    if (display_variable){std::cout << "N_AY =" << N << std::endl;}
    int * rate_recovered_bits, *polar_decoded_bits, *remainder, *bch_payload, *bch_crc_recomputed, *bch_crc, *crc_masq;
    rate_recovered_bits = new int[N];
    polar_decoded_bits = new int[free5GRAN::SIZE_PBCH_POLAR_DECODED];
    remainder = new int[free5GRAN::BCH_CRC_LENGTH + 1];
    bch_payload = new int[free5GRAN::BCH_PAYLOAD_SIZE];
    bch_crc_recomputed = new int[free5GRAN::BCH_CRC_LENGTH];
    bch_crc = new int[free5GRAN::BCH_CRC_LENGTH];
    crc_masq = new int[free5GRAN::BCH_CRC_LENGTH];
    // Rate recover bch_bits to rate_recovered_bits
    free5GRAN::phy::transport_channel::rate_recover(bch_bits,rate_recovered_bits,0,free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2,N, 0); /** 0 corresponds to 'K' but was choosen randomly */
    char* table_name9 = "AY_rate_recovered_bits";
    if (display_variable){
        display_table(rate_recovered_bits, 512, table_name9);}

    // Polar decode rate_recovered_bits to polar_decoded_bits
    free5GRAN::phy::transport_channel::polar_decode(rate_recovered_bits,polar_decoded_bits,N,free5GRAN::SIZE_PBCH_POLAR_DECODED,9,1,0,0, 0); /** The last 0 corresponds to 'E' but is choosen randomly */

    // Validate polar_decoded_bits CRC (compute the remainder and check that t is equal to 0)
    free5GRAN::phy::transport_channel::crc_validate(polar_decoded_bits, free5GRAN::G_CRC_24_C,remainder,free5GRAN::SIZE_PBCH_POLAR_DECODED,free5GRAN::BCH_CRC_LENGTH + 1);
    crc_validated = true;
    for (int i = 1; i < free5GRAN::BCH_CRC_LENGTH + 1; i ++){
        if (remainder[i] ==1){
            crc_validated = false;
            std::cout << "CRC IS NOT VALIDATED" << std::endl;
            break;
        }
    }
    // Split polar_decoded_bits into bch_payload and bch_crc
    for (int i = 0; i < free5GRAN::BCH_PAYLOAD_SIZE; i ++){
        bch_payload[i] = polar_decoded_bits[i];
    }
    for (int i = 0; i < free5GRAN::BCH_CRC_LENGTH; i ++){
        bch_crc[i] = polar_decoded_bits[free5GRAN::BCH_PAYLOAD_SIZE + i];
    }

    if (display_variable) {
        char *table_name11 = "AY_bch_payload";
        display_table(bch_payload, 32, table_name11);
    }

    // Re-compute the bch_payload CRC
    free5GRAN::phy::transport_channel::compute_crc(bch_payload, free5GRAN::G_CRC_24_C, bch_crc_recomputed, 32, 25);


    // XOR recomputed CRC with received CRC to determine CRC masq
    for (int i = 0; i < free5GRAN::BCH_CRC_LENGTH; i ++){
        crc_masq[i] = bch_crc[i] ^ bch_crc_recomputed[i];
    }

    /*
     * PBCH payload recovering (TS38.212 7.1.1)
     */
    int A = free5GRAN::BCH_PAYLOAD_SIZE;
    int A_bar = free5GRAN::BCH_PAYLOAD_SIZE - 8;
    int M = A - 3;
    int * s_sequence, *c_seq, *bch_descrambled;
    int sfn_bits[4][2] = {
            {0,0},
            {0,1},
            {1,0},
            {1,1}
    };
    // Find the correct value of v
    for (int v = 0 ; v < 4; v ++){
        // Generate de-scrambling sequence
        c_seq = new int[free5GRAN::BCH_PAYLOAD_SIZE + v * M];
        free5GRAN::utils::sequence_generator::generate_c_sequence(pci, free5GRAN::BCH_PAYLOAD_SIZE + v * M, c_seq, 0);

        s_sequence = new int[A];
        int j = 0;
        // Generate s sequence
        for (int i = 0; i < A; i ++){
            if ( i == 0 || i == 6 || i == 24){
                s_sequence[i] = 0;
            }else {
                s_sequence[i] = c_seq[j + v * M];
                j ++;
            }
        }

        bch_descrambled = new int[free5GRAN::BCH_PAYLOAD_SIZE];
        // De-scramble bch_payload to bch_descrambled
        free5GRAN::utils::common_utils::scramble(bch_payload, s_sequence, bch_descrambled, free5GRAN::BCH_PAYLOAD_SIZE, 0);

        // BCH payload de-interleaving

        //mib_bits = new int[free5GRAN::BCH_PAYLOAD_SIZE];
        int j_sfn = 0;
        int j_hrf = 10;
        int j_ssb = 11;
        int j_other = 14;
        for (int i = 0; i < 32; i ++){
            if (i == 24 || i == 25 || i == 26 || i ==27|| i==1||i==2||i==3||i==4||i==5||i==6){
                mib_bits[i] = bch_descrambled[free5GRAN::PBCH_PAYLOAD_INTERLEAVER[j_sfn]];
                j_sfn ++;
            }else if (i == 28){
                mib_bits[i] = bch_descrambled[free5GRAN::PBCH_PAYLOAD_INTERLEAVER[j_hrf]];
            }else if ( i >= A_bar + 5 && i <= A_bar + 7){
                mib_bits[i] = bch_descrambled[free5GRAN::PBCH_PAYLOAD_INTERLEAVER[j_ssb]];
                j_ssb ++;
            }else {
                mib_bits[i] = bch_descrambled[free5GRAN::PBCH_PAYLOAD_INTERLEAVER[j_other]];
                j_other ++;
            }
        }


        // If final bits correspond to 3rd and 2nd LSB of SFN, correct v was found
        if (sfn_bits[v][0] == mib_bits[25] && sfn_bits[v][1] == mib_bits[26]){
            //if(v==3){

            break;
        }

    }



}

void phy::AY_decode_mib(int* mib_bits, free5GRAN::mib &mib_object) {
    /**
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

phy::phy() {

}