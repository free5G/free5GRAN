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

#include "../common_structures/common_structures.h"

#ifndef FREE5GRAN_COMMON_VARIABLES_H
#define FREE5GRAN_COMMON_VARIABLES_H


namespace free5GRAN{

    /*
     * CRC generated with gcrc24c method.
     * TS38.212 5.1
     */
    extern int G_CRC_24_C[25];

    extern int G_CRC_24_A[25];

    extern int G_CRC_16[27];

    /*
     * TS 38.212 Table 7.1.1-1
     */
    extern int PBCH_PAYLOAD_INTERLEAVER[32];

    /*
     * TS38.212 Table 5.4.1.1-1
     */
    extern int SUB_BLOCK_INTERLEAVER_PATTERN[32];
    /*
     * TS38.212 Table 5.3.1.1-1
     */
    extern int INTERLEAVING_PATTERN[164];

    /*
     * TS38.212 Table 5.3.1.2-1
     */
    extern int POLAR_SEQUENCE_QNMAX_AND_RELIABILITY[1024];

    /*
     * Number of PBCH symbols in SSB
     */
    extern int SIZE_SSB_PBCH_SYMBOLS;

    /*
     * Number of DMRS symbols in SSB
     */
    extern int SIZE_SSB_DMRS_SYMBOLS;

    /*
     * Number of subcarriers in SSB
     */
    extern int NUM_SC_SSB;

    /*
     * Interval in 2nd SSB symbol with no PBCH & DMRS
     */
    extern int INTERVAL_SSB_NO_PBCH_DMRS[2];

    /*
     * Maximum value for I_BAR_SSB
     */
    extern int MAX_I_BAR_SSB;

    /*
     * Number of symbols with PBCH & DMRS in SSB
     */
    extern int NUM_SYMBOL_PBCH_SSB;

    /*
     * Size of PSS & SSS sequences
     */
    extern int SIZE_PSS_SSS_SIGNAL;

    /*
     * Maximum value for variable N_ID_1
     */
    extern int MAX_N_ID_1;

    /*
     * PBCH payload size after polar decoding (or before polar coding :-) )
     */
    extern int SIZE_PBCH_POLAR_DECODED;

    /*
     * BCH payload CRC size
     */
    extern int BCH_CRC_LENGTH;

    /*
     * BCH payload size
     */
    extern int BCH_PAYLOAD_SIZE;

    /*
     * Base sequence for generating PSS
     */
    extern int PSS_BASE_SEQUENCE[7];

    /*
     * X0 base sequence for generating SSS
     */
    extern int SSS_BASE_X0_SEQUENCE[7];

    /*
     * X1 base sequence for generating SSS
     */
    extern int SSS_BASE_X1_SEQUENCE[7];

    /*
     * Number of symbols in SSBlock
     */
    extern int NUM_SYMBOLS_SSB;

    /*
     * X1 base sequence for generating for PBCH DMRS
     */
    extern int DMRS_BASE_X1_SEQUENCE[32];

    /*
     * Number of supported bands
     */
    extern int NUM_SUPPORTED_BANDS;
    /*
     * Array to store available bands
     */
    extern free5GRAN::band AVAILABLE_BANDS[7];

    /*
     * TS 38.213 Table 13-1
     */
    extern int TS_38_213_TABLE_13_1[16][4];

    /*
     * TS 38.213 Table 13-2
     */
    extern int TS_38_213_TABLE_13_2[16][4];

    /*
     * TS 38.213 Table 13-3
     */
    extern int TS_38_213_TABLE_13_3[16][4];

    /*
     * TS 38.213 Table 13-4
     */
    extern int TS_38_213_TABLE_13_4[16][4];

    /*
     * TS 38.213 Table 13-11
     */
    extern float TS_38_213_TABLE_13_11[16][4];

    extern int TS_38_214_TABLE_5_1_2_1_1_2[16][2][4];

    extern int TS_38_214_TABLE_5_1_3_1_1[29][2];

    extern int TS_38_211_TABLE_7_4_1_1_2_3[15][4];

    extern int TS_38_214_TABLE_5_1_3_2_1[93];

    extern int TS_38_212_TABLE_5_3_2_1[8][8];

    extern int NUMBER_SYMBOLS_PER_SLOT_NORMAL_CP;

    extern int NUMBER_REG_PER_CCE;

    extern int SI_RNTI[16];

    /**
* FROM HERE, IT IS ADDITION FROM BENOIT. BE CAREFUL WHEN MERGING !!
*/


    extern int INDEX_OF_UNUSED_BITS_IN_MIB[5];
    extern int INDEX_OF_SFN_BITS_IN_MIB[10];
    extern int INDEX_OF_PDDCHC_CONFIG_BITS_IN_MIB[8];
    extern int INDEX_OF_K_SSB_BITS_IN_MIB[5];
    extern int INDEX_OF_AVAILABLE_SCS_IN_MIB[1];
    extern int INDEX_OF_CELL_BARRED_BITS_IN_MIB[1];
    extern int INDEX_OF_DMRS_TYPE_A_POSITION_BITS_IN_MIB[1];
    extern int INDEX_OF_INTRA_FREQ_RESELECTION_BITS_IN_MIB[1];

    extern int INTERVAL_SSB_PSS[2];

    extern int INTERVAL_SSB_SSS[2];

    extern int SIZE_IFFT_SSB; /** This indicates the number of element in each symbols of SSB, before and after ifft */

    extern int MIB_BITS_SIZE;
}
#endif
