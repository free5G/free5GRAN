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

 * \author Télécom Paris, P5G Lab ; Benoit Oehmichen & Aymeric de Javel
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


void phy::generate_frame(free5GRAN::mib mib_object, int num_SSB_in_this_frame, int num_symbols_frame, int *cp_lengths_one_frame, int sfn, int pci, int N, int i_b_ssb, float scaling_factor, std::vector<std::complex<float>> &one_frame_vector) {
    /**
    * \fn generate_frame(free5GRAN::mib mib_object, int num_SSB_in_this_frame, int num_symbols_frame, int *cp_lengths_one_frame, int sfn, double ssb_period,int pci, int N, int gscn, int i_b_ssb, float scaling_factor, std::vector<std::complex<float>> &one_frame_vector)
    * \brief From mib_object and many other parameters, generates a frame of 10 ms containing SSB.
    * \details
               * -step 1: MIB GENERATION. Generate mib_bits from mib_object
               * -step 2: ENCODE BCH. Generate bch bits sequence.
               * -step 3: ENCODE PBCH. Generate pbch symbols
               * -step 4: GENERATE FREQUENCY DOMAIN FRAME. Generate a frequency domain frame with SSB placed in it
               * -step 5: IFFT. Perform ifft for each symbols to get the final 10ms time_domain frame.
    * \standard !! TS to be added !!
    * \param[in] mib_object object MIB created in common_structures.h, including cell_barred, k_ssb, pddchc_config...
    * \param[in] num_SSB_in_this_frame. Number of SSB that the frame will contain. Is calculated in function of ssb_period and sfn. Should be equal to 0, 1 or 2
    * \param[in] num_symbols_frame. Number of symbols that the frame will contain (eg 140 or 280).
    * \param[in] *cp_lengths_one_frame. Cyclic Prefix length for each symbol of a frame
    * \param[in] sfn. Sequence Frame Number. Varies between 0 and 1023
    * \param[in] pci. Physical Cell ID. Should be between 0 and 1007.
    * \param[in] N. Length of BCH after polar encoding.
    * \param[in] i_b_ssb. SSB index (between 0 and 7). Indicates the position of SSB in the frame.
    * \param[in] scaling_factor. Multiplication factor applied to each values before performing ifft
    * \param[out] &one_frame_vector. One dimension vector containing, in time domain, 10 ms of signal with SSB included in it.
    */


    mib_object.sfn = sfn;
    BOOST_LOG_TRIVIAL(warning) << "SFN (from generate_frame) = " + std::to_string(sfn);

    /** Step 1: MIB GENERATION -> Generate mib_bits sequence (32 bits long in our case) from mib_object. TS38.331 V15.11.0 Section 6.2.2*/
    int mib_bits[free5GRAN::BCH_PAYLOAD_SIZE];
    free5GRAN::utils::common_utils::encode_mib(mib_object, mib_bits);
    BOOST_LOG_TRIVIAL(info) << "MIB GENERATION from generate_frame done";

    /** Step 2: ENCODE BCH -> Generate rate_matched_bch (864 bits in our case) from mib_bits. TS38.212 V15.2.0 Section 5 */
    vector<int> rate_matched_bch_vector(free5GRAN::SIZE_SSB_PBCH_SYMBOLS*2, 0);
    free5GRAN::phy::transport_channel::bch_encoding(mib_bits, pci, N, rate_matched_bch_vector);
    BOOST_LOG_TRIVIAL(info) << "ENCODE BCH from generate_frame";

    /** Step 3: ENCODE PBCH -> Generate pbch_symbols (432 symbols in our case) from rate_matched_bch. TS38.212 V15.2.0 Section 7.3.3.1 and 5.1.3 */

    vector<complex<float>> pbch_symbols_vector(free5GRAN::SIZE_SSB_PBCH_SYMBOLS);
    free5GRAN::phy::physical_channel::pbch_encoding(rate_matched_bch_vector, pci, i_b_ssb, pbch_symbols_vector);
    BOOST_LOG_TRIVIAL(info) << "ENCODE PBCH from generate_frame";

    /** Step 4: GENERATE FREQUENCY DOMAIN FRAME -> Generate freq_domain_frame from pbch_symbols. TS38.211 V15.2.0 Section 7.4 */
    /** Calculate the position of ssb block in a frame */
    int index_symbol_ssb = free5GRAN::BAND_N_78.ssb_symbols[free5GRAN::gnodeB_config_globale.i_b_ssb];

    vector<vector<complex<float>>> ONEframe_SSB_freq(free5GRAN::num_symbols_frame, vector<complex<float>>(free5GRAN::SIZE_IFFT_SSB));


    free5GRAN::phy::signal_processing::generate_freq_domain_frame(pbch_symbols_vector, pci, index_symbol_ssb,
                                                                  num_SSB_in_this_frame, i_b_ssb,ONEframe_SSB_freq);
    BOOST_LOG_TRIVIAL(info) << "GENERATE ONEframe_SSB_freq";


    /** Step 5: IFFT. Perform ifft for each symbols to get the final 10ms time_domain frame */

    /** data_symbols will indicates which symbols of the frame is not nul.
     * If data_symbols[symbol] = 1, a signal processing will be apply to this symbol (reverse, scaling factor and ifft)
     * If data_symbols[symbol] = 0, nothing will be done for this symbol
    */
    vector<int> data_symbols(num_symbols_frame, 0);
    int count = index_symbol_ssb;
    for (int symbol = 0; symbol < free5GRAN::NUM_SYMBOLS_SSB; symbol++){
        /** for each  4 symbols containing SSB, data_symbols[symbol] = 1 */
        data_symbols[count] = 1;
        count ++;
    }

    /** If frame contains a second SSB, 4 more data_symbols */
    if (num_SSB_in_this_frame == 2){
        int count2 = index_symbol_ssb + (num_symbols_frame/2);
        for (int symbol = 0; symbol < free5GRAN::NUM_SYMBOLS_SSB; symbol++){
            data_symbols[count2] = 1;
            count2++;
        }
    }

    /** ifft -> This function are in 4 STEP: Place SSB in an empty frame ; reverse symbols ; ifft for each symbols ; adding CP for each symbols */
    free5GRAN::phy::signal_processing::ifft(ONEframe_SSB_freq, cp_lengths_one_frame, data_symbols, num_symbols_frame, scaling_factor,
                                            one_frame_vector);
    BOOST_LOG_TRIVIAL(info) << "function ifft done";
}






void phy::compute_num_sample_per_frame(free5GRAN::mib mib_object, int &Num_samples_in_frame) {

    /**
   * \fn compute_num_sample_per_frame(free5GRAN::mib mib_object, int &Num_samples_in_frame)
   * \brief Calculates the number of samples (IQ) that a 10 ms radio-frame will contain.
   * \standard !! TS TO BE ADDED !!
   * \param[in] mib_object. parameter SCS will be used
   * \param[out] &Num_samples_in_frame
   */

    /** Calculate number of symbols per subframe */
    int Num_symbols_per_subframe;
    if (mib_object.scs == 15000) {
        Num_symbols_per_subframe = 14;
    } else if (mib_object.scs == 30000) {
        Num_symbols_per_subframe = 28;
    }

    int Num_symbols_per_frame = Num_symbols_per_subframe * 10;

    /** Calculate cp_length for a subframe */
    int cp_lengths[Num_symbols_per_subframe], cum_sum_cp_lengths[Num_symbols_per_subframe];
    free5GRAN::phy::signal_processing::compute_cp_lengths(mib_object.scs / 1000, free5GRAN::SIZE_IFFT_SSB, 0,
                                                          Num_symbols_per_subframe, &cp_lengths[0],
                                                          &cum_sum_cp_lengths[0]);

    /** Calculate cp_length for each symbols of a frame */
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

void phy::reduce_main(bool run_with_usrp, ) {


}



