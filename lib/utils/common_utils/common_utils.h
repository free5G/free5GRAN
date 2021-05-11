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

#include <uhd.h>

#include <condition_variable>
#include <uhd/usrp/multi_usrp.hpp>
#include <vector>

#include "../../variables/common_structures/common_structures.h"
using namespace std;

namespace free5GRAN::utils::common_utils {
void parse_mib(int* mib_bits, free5GRAN::mib& mib_object);

void scramble(int* input_bits,
              int* c_seq,
              int* output_bits,
              int length,
              int offset);

void scramble(double* input_bits,
              int* c_seq,
              double* output_bits,
              int length,
              int offset);

void get_usrp_devices(vector<free5GRAN::rf_device>& rf_devices_list);

void select_rf_device(free5GRAN::rf_device& rf_device_obj, string identifier);

auto create_bwp(int id, double scs, int size_rb, double sampling_rate)
    -> free5GRAN::bwp;

void init_fft_plans(double sampling_rate);

void destroy_fft_plans();

}  // namespace free5GRAN::utils::common_utils
