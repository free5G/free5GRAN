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

#include "common_structures.h"

#include "../common_variables/common_variables.h"
using namespace std;

free5GRAN::ssb_pattern_t free5GRAN::SSB_PATTERN_CASE_A_FR1_INF_3_GHZ = {
    .scs = (int)15e3,
    .l_max = 4,
    .ssb_start_symbols = vector<int>{2, 8, 16, 22},
};
free5GRAN::ssb_pattern_t free5GRAN::SSB_PATTERN_CASE_A_FR1_SUP_3_GHZ = {
    .scs = (int)15e3,
    .l_max = 8,
    .ssb_start_symbols = vector<int>{2, 8, 16, 22, 30, 36, 44, 50},
};
free5GRAN::ssb_pattern_t free5GRAN::SSB_PATTERN_CASE_B_FR1_INF_3_GHZ = {
    .scs = (int)30e3,
    .l_max = 4,
    .ssb_start_symbols = vector<int>{4, 8, 16, 20},
};
free5GRAN::ssb_pattern_t free5GRAN::SSB_PATTERN_CASE_B_FR1_SUP_3_GHZ = {
    .scs = (int)30e3,
    .l_max = 8,
    .ssb_start_symbols = vector<int>{4, 8, 16, 20, 32, 36, 44, 48},
};
free5GRAN::ssb_pattern_t free5GRAN::SSB_PATTERN_CASE_C_FR1_INF_3_GHZ = {
    .scs = (int)30e3,
    .l_max = 4,
    .ssb_start_symbols = vector<int>{2, 8, 16, 22},
};
free5GRAN::ssb_pattern_t free5GRAN::SSB_PATTERN_CASE_C_FR1_SUP_3_GHZ = {
    .scs = (int)30e3,
    .l_max = 8,
    .ssb_start_symbols = vector<int>{2, 8, 16, 22, 30, 36, 44, 50},
};

// https://medium.com/@mishra.eric/understanding-nr5g-synchronisation-signal-block-ssb-7a8d56a48e16
free5GRAN::band free5GRAN::BAND_N_1 = {
    1, 5279, 5419, free5GRAN::SSB_PATTERN_CASE_A_FR1_INF_3_GHZ};
free5GRAN::band free5GRAN::BAND_N_2 = {
    2, 4829, 4969, free5GRAN::SSB_PATTERN_CASE_A_FR1_INF_3_GHZ};
free5GRAN::band free5GRAN::BAND_N_3 = {
    3, 4517, 4693, free5GRAN::SSB_PATTERN_CASE_A_FR1_INF_3_GHZ};
free5GRAN::band free5GRAN::BAND_N_7 = {
    7, 6554, 6718, free5GRAN::SSB_PATTERN_CASE_A_FR1_INF_3_GHZ};
free5GRAN::band free5GRAN::BAND_N_8 = {
    8, 2318, 2395, free5GRAN::SSB_PATTERN_CASE_A_FR1_INF_3_GHZ};
free5GRAN::band free5GRAN::BAND_N_28 = {
    28, 1901, 2002, free5GRAN::SSB_PATTERN_CASE_A_FR1_INF_3_GHZ};
free5GRAN::band free5GRAN::BAND_N_78 = {
    78, 7711, 8051, free5GRAN::SSB_PATTERN_CASE_C_FR1_SUP_3_GHZ};
// TEST n78
// free5GRAN::band free5GRAN::BAND_N_78 = {78, 7837, 7839,
// free5GRAN::SSB_PATTERN_CASE_C_FR1_SUP_3_GHZ};
// TEST n8
// free5GRAN::band free5GRAN::BAND_N_8 = {8, 2355, 2395,
// free5GRAN::SSB_PATTERN_CASE_A_FR1_INF_3_GHZ};

free5GRAN::bandwidth_info free5GRAN::BANDWIDTH_15_KHZ = {(int)15e3};

free5GRAN::bandwidth_info free5GRAN::BANDWIDTH_30_KHZ = {(int)30e3};
