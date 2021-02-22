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

 * \author Télécom Paris, P5G Lab ; Benoit Oehmicen & Aymeric de Javel
 * \version 0.2
 * \date February 2021
 */

#ifndef FREE5GRAN_PHY_H
#define FREE5GRAN_PHY_H

#include "../../lib/variables/common_structures/common_structures.h"
#include "../../lib/phy/transport_channel/transport_channel.h"
#include "../../lib/variables/common_variables/common_variables.h"
#include "../rf/rf.h"
#include_next <math.h>
#include <fftw3.h>
#include <vector>
#include <complex>
#include <fstream>

class phy {

private:
    bool crc_validated;
    int i_ssb;
    int l_max;


public:

    void generate_frame(free5GRAN::mib mib_object, int num_SSB_in_this_frame, int num_symbols_frame, int *cp_lengths_one_frame, int sfn,int pci, int N, int i_b_ssb, float scaling_factor, std::vector<std::complex<float>> &buff_phy);
    void compute_num_sample_per_frame(free5GRAN::mib mib_object, int &Num_samples_in_frame);
    void execute_IFFT();
};


#endif //FREE5GRAN_PHY_H
