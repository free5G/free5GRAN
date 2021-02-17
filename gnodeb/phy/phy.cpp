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

#include "phy.h"
#include <iostream>
#include "../../lib/utils/sequence_generator/sequence_generator.h"
#include "../../lib/phy/libphy/libphy.h"
#include <complex>
#include <vector>
#include "../../lib/utils/sequence_generator/sequence_generator.h"
#include "../../lib/variables/common_variables/common_variables.h"
#include "../../lib/utils/common_utils/common_utils.h"
#include "../../lib/phy/libphy/libphy.h"
#include "../../lib/phy/transport_channel/transport_channel.h"
#include "../../lib/phy/physical_channel/physical_channel.h"
#include <boost/format.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/file.hpp>


void phy::generate_frame(free5GRAN::mib mib_object, int index_symbol_ssb, int num_symbols_frame, int *cp_lengths_one_frame, int sfn, double ssb_period,int pci, int N, int gscn, int i_b_ssb, float scaling_factor, std::vector<std::complex<float>> &one_frame_vector) {

    mib_object.sfn = sfn;
    BOOST_LOG_TRIVIAL(warning) << "SFN = " + std::to_string(sfn);

    /** MIB GENERATION -> Generate mib_bits sequence (32 bits long in our case) from mib_object. TS38.331 V15.11.0 Section 6.2.2*/
    int mib_bits[free5GRAN::BCH_PAYLOAD_SIZE];
    free5GRAN::utils::common_utils::encode_mib(mib_object, mib_bits);
    BOOST_LOG_TRIVIAL(info) << "MIB GENERATION from generate_frame done";

    /** ENCODE BCH -> Generate rate_matched_bch (864 bits in our case) from mib_bits. TS38.212 V15.2.0 Section 5 */
    int *rate_matched_bch = new int[free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2];
    free5GRAN::phy::transport_channel::bch_encoding(mib_bits, pci, N, rate_matched_bch);
    BOOST_LOG_TRIVIAL(info) << "ENCODE BCH from generate_frame";

    /** ENCODE PBCH -> Generate pbch_symbols (432 symbols in our case) from rate_matched_bch. TS38.212 V15.2.0 Section 7.3.3.1 and 5.1.3 */
    std::complex<float> *pbch_symbols;
    pbch_symbols = new std::complex<float>[free5GRAN::SIZE_SSB_PBCH_SYMBOLS];
    free5GRAN::phy::physical_channel::pbch_encoding(rate_matched_bch, pci, gscn, i_b_ssb, pbch_symbols);
    BOOST_LOG_TRIVIAL(info) << "ENCODE PBCH from generate_frame";

    /** GENERATE SSB -> Generate SSB_signal_frequency_domain from pbch_symbols. TS38.211 V15.2.0 Section 7.4 */

    vector<vector<complex<float>>> SSB_signal_extended(free5GRAN::NUM_SYMBOLS_SSB,
                                                       vector<complex<float>>(free5GRAN::SIZE_IFFT_SSB));

    free5GRAN::phy::signal_processing::generate_freq_domain_ssb(pbch_symbols, mib_object, pci, i_b_ssb, free5GRAN::SIZE_IFFT_SSB, SSB_signal_extended);
    BOOST_LOG_TRIVIAL(info) << "GENERATE SSB_signal_time_domain";

    /** IFFT -> This function are in 4 STEP: Place SSB in an empty frame ; reverse symbols ; ifft for each symbols ; adding CP for each symbols */
    free5GRAN::phy::signal_processing::IFFT(SSB_signal_extended, index_symbol_ssb, cp_lengths_one_frame, free5GRAN::NUM_SYMBOLS_SSB, num_symbols_frame, scaling_factor, pci, i_b_ssb, one_frame_vector);
    BOOST_LOG_TRIVIAL(info) << "IFFT from SSB_signal_extended to get one_frame_vector";
}


void phy::compute_num_sample_per_frame(free5GRAN::mib mib_object, int &Num_samples_in_frame) {

    /** Calculate number of symbols per subframe */
    int Num_symbols_per_subframe;
    if (mib_object.scs == 15000) {
        Num_symbols_per_subframe = 14;
    } else if (mib_object.scs == 30000) {
        Num_symbols_per_subframe = 28;
    }

    int Num_symbols_per_frame = Num_symbols_per_subframe * 10;

    /** Calculate cp_length */
    int cp_lengths[Num_symbols_per_subframe], cum_sum_cp_lengths[Num_symbols_per_subframe];
    free5GRAN::phy::signal_processing::compute_cp_lengths(mib_object.scs / 1000, free5GRAN::SIZE_IFFT_SSB, 0,
                                                          Num_symbols_per_subframe, &cp_lengths[0],
                                                          &cum_sum_cp_lengths[0]);

    /** Initialize cp_length for each symbols of a frame */
    int cp_lengths_one_frame[Num_symbols_per_frame];
    for (int sub_frame = 0; sub_frame < 10; sub_frame++) {
        for (int symbol = 0; symbol < Num_symbols_per_subframe; symbol++) {
            cp_lengths_one_frame[Num_symbols_per_subframe * sub_frame + symbol] = cp_lengths[symbol];
        }
    }

    /** Calculate size of each symbol in a frame */
    int symbols_size_one_frame[Num_symbols_per_frame];
    for (int symbol = 0; symbol < Num_symbols_per_frame; symbol++) {
        symbols_size_one_frame[symbol] = free5GRAN::SIZE_IFFT_SSB + cp_lengths_one_frame[symbol];
    }

    /** Calculate Num_samples_in_frame */
    Num_samples_in_frame = 0;
    for (int symbol = 0; symbol < Num_symbols_per_frame; symbol++) {
        Num_samples_in_frame = Num_samples_in_frame + symbols_size_one_frame[symbol];
    }
}



