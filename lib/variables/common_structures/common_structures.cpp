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

#include "common_structures.h"
#include "../common_variables/common_variables.h"

free5GRAN::band free5GRAN::BAND_N_1 = {1, (int) 15e3, 5279, 5419, 4};
free5GRAN::band free5GRAN::BAND_N_2 = {2, (int) 15e3, 4829, 4969, 4};
free5GRAN::band free5GRAN::BAND_N_3 = {3, (int) 15e3, 4517, 4693, 4};
free5GRAN::band free5GRAN::BAND_N_7 = {7,(int) 15e3, 6554, 6718, 4,new int[8]{2,8,16,22}};
free5GRAN::band free5GRAN::BAND_N_8 = {8, (int) 15e3, 2318, 2395, 4, new int[8]{2,8,16,22}};
free5GRAN::band free5GRAN::BAND_N_28 = {28, (int) 15e3, 1901, 2002, 4, new int[8]{2,8,16,22}};
free5GRAN::band free5GRAN::BAND_N_78 = {78, (int) 30e3, 7711, 8051, 8, new int[8]{2,8,16,22,30,36,44,50}};
// TEST n78
//free5GRAN::band free5GRAN::BAND_N_78 = {78, (int) 30e3, 7837, 7839, 8, new int[8]{2,8,16,22,30,36,44,50}};



free5GRAN::bandwidth_info free5GRAN::BANDWIDTH_15_KHZ = {(int) 15e3,
                                                         free5GRAN::FFT_SIZE_1_92_MHZ_SCS_15_KHZ,
                                                         free5GRAN::FFT_SIZE_3_84_MHZ_SCS_15_KHZ,
                                                         free5GRAN::CP_LENGTH_128,
                                                         free5GRAN::CP_LENGTH_256,
                                                     1.92e6,
                                                     3.84e6
};

free5GRAN::bandwidth_info free5GRAN::BANDWIDTH_30_KHZ = {(int) 30e3,
                                                         free5GRAN::FFT_SIZE_3_84_MHZ_SCS_30_KHZ,
                                                         free5GRAN::FFT_SIZE_7_68_MHZ_SCS_30_KHZ,
                                                         free5GRAN::CP_LENGTH_128,
                                                         free5GRAN::CP_LENGTH_256,
                                                    3.84e6,
                                                    7.68e6
};