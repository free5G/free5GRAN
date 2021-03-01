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

#include <cmath>
#include "common_utils.h"
#include <uhd.h>
#include <uhd/utils/thread.hpp>
using namespace std;

void free5GRAN::utils::common_utils::parse_mib(int *mib_bits, free5GRAN::mib &mib_object) {
    /**
     * \fn parse_mib
     * \brief MIB parser
     * \standard TS 38.331 V15.3.0 Section 6.2
     *
     * \param[in] mib_bits: Input bits sequence
     * \param[out] mib_object: Output MIB object
     */
    /*
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

/*
 * Scrambling implementation: output = (input + c_seq)[2]
 */
void free5GRAN::utils::common_utils::scramble(int *input_bits, int *c_seq, int *output_bits, int length, int offset) {
    /**
     * \fn scramble
     * \brief Hard bits scrambling
     * \param[in] input_bits: Input bits sequence
     * \param[in] c_seq: Scrambling sequence
     * \param[out] output_bits: Output sequence
     * \param[in] length: Input bits sequence length
     * \param[in] offset: Scrambling offset
     */
    for (int i = 0; i < length; i ++){
        output_bits[i] = (input_bits[i] + c_seq[i + offset]) % 2;
    }
}

void free5GRAN::utils::common_utils::scramble(double *input_bits, int *c_seq, double *output_bits, int length, int offset) {
    /**
     * \fn scramble
     * \brief Soft bits scrambling
     * \param[in] input_bits: Input bits sequence
     * \param[in] c_seq: Scrambling sequence
     * \param[out] output_bits: Output sequence
     * \param[in] length: Input bits sequence length
     * \param[in] offset: Scrambling offset
     */
    for (int i = 0; i < length; i ++){
        output_bits[i] = input_bits[i] * c_seq[i + offset];
    }
}

void free5GRAN::utils::common_utils::get_usrp_devices(vector<free5GRAN::rf_device> &rf_devices_list){
    /**
     * \fn get_usrp_devices
     * \brief List all available USRP devices
     * \param[out] rf_devices_list: Output vector containing serial and type for connected devices
     */
    uhd::device_addrs_t device_addrs = uhd::device::find(string(""), uhd::device::USRP);
    for (auto it = device_addrs.begin(); it != device_addrs.end(); ++it) {
        rf_devices_list.push_back({
                .type= (*it)["type"],
                .serial= (*it)["serial"]
        }
        );
    }
}

void free5GRAN::utils::common_utils::select_rf_device(free5GRAN::rf_device &rf_device_obj, string identifier){
    /**
     * \fn select_rf_device
     * \brief List all available RF devices and select one
     * \param[out] rf_device_obj: Output RF device object containing information about selected device
     * \param[in] identifier: Preferred device identifier
     */
    vector<free5GRAN::rf_device> usrp_devices;
    free5GRAN::utils::common_utils::get_usrp_devices(usrp_devices);
    if (!identifier.empty()){
        for (const auto& usrp_device: usrp_devices) {
            if (usrp_device.serial == identifier){
                rf_device_obj = usrp_device;
                break;
            }
        }
    }else{
        for (const auto& usrp_device: usrp_devices) {
            if (usrp_device.type == "x300" || usrp_device.type == "b200" || usrp_device.type == "usrp2"){
                rf_device_obj = usrp_device;
                break;
            }
        }
    }
}