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

#include "phy.h"
#include "../../lib/phy/synchronization/synchronization.h"
#include "../../lib/phy/libphy/libphy.h"
#include "../../lib/variables/common_variables/common_variables.h"
#include "../../lib/utils/sequence_generator/sequence_generator.h"
#include "../../lib/phy/transport_channel/transport_channel.h"
#include <iostream>
#include <vector>
#include <fftw3.h>
#include <fstream>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include "../../lib/asn1c/nr_rrc/BCCH-DL-SCH-Message.h"
#include "../../lib/variables/common_structures/common_structures.h"
#include "../../lib/phy/physical_channel/physical_channel.h"
#include "../../lib/utils/common_utils/common_utils.h"
#include "../../lib/asn1c/nr_rrc/BCCH-DL-SCH-Message.h"
#include "../../lib/asn1c/nr_rrc/FrequencyInfoUL-SIB.h"

using namespace std;

phy::phy(rf *rf_dev, double ssb_period, int fft_size, int scs, free5GRAN::band band_obj) {
    /**
     * \fn phy
     * \param[in] rf_dev: RF device. (Only USRP B210 is currently supported)
     * \param[in] ssb_period: SSB periodicity. Default value is 0.02 (20 ms)
     * \param[in] fft_size: FFT/iFFT size. Represents the total number of os subcarriers to be decoded
     * \param[in] scs: Subcarrier spacing
     * \param[in] band_obj: Band object for getting Lmax value
    */
    this->rf_device = rf_dev;
    this->ssb_period = ssb_period;
    this->fft_size = fft_size;
    this->scs = scs;
    this->band_object = band_obj;
    l_max = band_obj.l_max;
    this->is_extended_cp = 0;
}

int phy::cell_synchronization(float &received_power) {
    /**
     * \fn cell_synchronization
     * \brief Perform time synchronization
     * \details
     * - PSS cross-correlation to retrieve N_ID_2
     * - SSS correlation to retrieve N_ID_1
     * - PCI computation based on N_ID_1 and N_ID_2
     *
     * \param[in] received_power: PSS received power. Used for power ramping.
    */
    BOOST_LOG_TRIVIAL(trace) << "PSS synchronization";

    int n_id_2,synchronisation_index;
    float peak_value;
    double time_first_sample;
    received_power = 0;

    size_t num_samples = 2 * ssb_period * rf_device->getSampleRate();

    // Create buffer
    vector<complex<float>> buff_2_ssb_periods(num_samples);
    buff.clear();
    buff.resize(num_samples / 2);

    complex<float> j(0, 1);
    // Get samples from RF layer and put them in buff variable
    time_first_pss = chrono::high_resolution_clock::now();
    try {
        rf_device->get_samples(&buff_2_ssb_periods, time_first_sample);
    }catch (const exception& e) {
        return 1;
    }
    int num_symbols_per_subframe_pbch = free5GRAN::NUMBER_SYMBOLS_PER_SLOT_NORMAL_CP * scs/15e3;
    int cp_lengths_pbch[num_symbols_per_subframe_pbch];
    int cum_sum_pbch[num_symbols_per_subframe_pbch];

    free5GRAN::phy::signal_processing::compute_cp_lengths((int) scs/1e3, fft_size, 0, num_symbols_per_subframe_pbch, cp_lengths_pbch, cum_sum_pbch);
    /*
     * Take second symbol CP as common CP as SSB is never transmitted at long CP symbols (long CP are transmitted every 0.5ms)
     */
    common_cp_length = cp_lengths_pbch[1];
    /*
     * Extract first half of buffer (= 1 SSB period)
     */
    for (int i = 0; i < num_samples / 2; i ++){
        buff[i] = buff_2_ssb_periods[i];
    }

    /*
     * Get PSS correlation result
     */
    free5GRAN::phy::synchronization::search_pss(n_id_2,synchronisation_index,peak_value, common_cp_length, buff, fft_size);
    BOOST_LOG_TRIVIAL(trace) << "Peak value: "+ to_string(peak_value/common_cp_length);
    /*
     * Computing symbol length and first sample index of PSS in buff
     */
    int symbol_duration = fft_size + common_cp_length;
    int pss_start_index = synchronisation_index - symbol_duration + 1;
    int sss_init_index = pss_start_index + 2 * symbol_duration + common_cp_length; // = (synchronisation_index - symbol_duration + 1) + 2 * symbol_duration;
    /*
     * If highest correlation peak is not fully in buffer, cell is not found
     */
    if (pss_start_index < 0){
        return 1;
    }

    index_first_pss = pss_start_index;

    vector<complex<float>> sss_signal(fft_size);
    /*
     * Extracting the SSS signal based on sss_init_index and cp_length
     */
    for (int i = 0; i < fft_size; i++){
        sss_signal[i] = buff[i + sss_init_index];
    }
    /*
     * Computing received power
     */
    for (int i = 0; i < 4 * symbol_duration; i ++){
        received_power += pow(abs(buff[pss_start_index + i]),2);
    }
    received_power /= 4 * symbol_duration;
    received_power = 10 * log10(received_power);
    int n_id_1;
    float peak_value_sss;

    /*
     * Get SSS correlation result
     */
    BOOST_LOG_TRIVIAL(trace) << "SSS synchronization";
    free5GRAN::phy::synchronization::get_sss(n_id_1, peak_value_sss, sss_signal, fft_size, n_id_2);
    BOOST_LOG_TRIVIAL(trace) << "Peak value: "+ to_string(peak_value_sss);
    pci = 3 * n_id_1 + n_id_2;
    BOOST_LOG_TRIVIAL(trace) << "PCI : "+ to_string(pci);


    /*
     * Retreive first SSB symbol of second SSB period
     */
    vector<complex<float>> second_pss(fft_size + common_cp_length), second_sss(fft_size);
    int second_pss_index = pss_start_index + num_samples / 2;
    int n_id_1_2, n_id_2_2, sync_index_pss_2;
    float peak_value_pss_2;
    for (int i = 0; i < fft_size + common_cp_length; i++){
        second_pss[i] = buff_2_ssb_periods[i + second_pss_index];
    }
    /*
     * Retrieve N ID 2 value from second SSB
     */
    free5GRAN::phy::synchronization::search_pss(n_id_2_2,sync_index_pss_2,peak_value_pss_2, common_cp_length, second_pss, fft_size);

    /*
     * Extract SSS symbol from second SSB
     */
    for (int i = 0; i < fft_size; i++){
        second_sss[i] = buff_2_ssb_periods[i + second_pss_index + 2 * symbol_duration + common_cp_length];
    }

    free5GRAN::phy::synchronization::get_sss(n_id_1_2, peak_value_sss, second_sss, fft_size, n_id_2_2);

    if (3 * n_id_1_2 + n_id_2_2 == pci){
        return 0;
    }else {
        return 1;
    }
}

int phy::extract_pbch() {
    /**
     * \fn extract_pbch
     * \brief Time resynchronization, frequency synchronization, PBCH extraction and decoding.
     * \details
     * - Getting 3ms signal from RF device
     * - PSS cross-correlation to retrieve N_ID_2
     * - SSS correlation to retrieve N_ID_1
     * - PCI computation based on N_ID_1 and N_ID_2
     * - Function ends if recomputed PCI differs from to initially computed one
     * - Fine frequency synchronization by correlating cyclic prefixes and corresponding symbol part
     * - Signal extraction and FFT
     * - Resource element de-mapper
     * - Channel estimation based on different values of i_ssb
     * - Channel equalization based on best SNR value
     * - PBCH decoding
     * - BCH decoding
     * - MIB parsing
    */
    BOOST_LOG_TRIVIAL(trace) << "Extracting PBCH";
    // Get at least 30ms of signal (=3 frames, at least 2 complete ones)
    size_t num_samples = max(0.03, ssb_period) * rf_device->getSampleRate();
    //vector<complex<float>> buff(num_samples);
    buff.clear();
    buff.resize(num_samples);

    double second_frame_time;
    // Getting samples
    auto now = chrono::high_resolution_clock::now();
    try {
        rf_device->get_samples(&buff, second_frame_time);
    }catch (const exception& e) {
        return 1;
    }

    /*
     * SYNCHRONIZING IN THE NEW RECEIVED FRAME
     * Computing approximate PSS index inside the received buffer using the time reference of the first PSS index SSB initial search
     */
    auto time_window = chrono::duration_cast<chrono::microseconds>(now - time_first_pss);
    int offset_to_ssb_period = (int)(time_window.count() - index_first_pss / (128*scs *1e-6)) % ((int) (ssb_period * 1e6));
    int index_second_pss = (ssb_period * 1e6 - offset_to_ssb_period) * rf_device->getSampleRate() * 1e-6;

    ofstream data;
    data.open("data.txt");
    for (int i = 0; i < num_samples; i ++){
        data << buff[i];
        data << "\n";
    }
    data.close();

    /*
     * Compute PBCH CP length
     */
    int num_symbols_per_subframe_pbch = free5GRAN::NUMBER_SYMBOLS_PER_SLOT_NORMAL_CP * scs/15e3;
    int cp_lengths_pbch[num_symbols_per_subframe_pbch];
    int cum_sum_pbch[num_symbols_per_subframe_pbch];

    free5GRAN::phy::signal_processing::compute_cp_lengths((int) scs/1e3, fft_size, 0, num_symbols_per_subframe_pbch, cp_lengths_pbch, cum_sum_pbch);
    common_cp_length = cp_lengths_pbch[1];
    int symbol_duration = fft_size + common_cp_length;

    vector<complex<float>> pss_signal(48 * symbol_duration);

    /*
     * Isolate PSS signal around calculated new PSS occurence (based on timestamp of first synchronization step)
     */
    int begin_offset, end_offset;
    if (pss_signal.size()/2 <= index_second_pss && pss_signal.size()/2  <= num_samples - index_second_pss){
        begin_offset = pss_signal.size()/2;
        end_offset = pss_signal.size()/2;
    } else if (pss_signal.size()/2 > index_second_pss){
        begin_offset = index_second_pss;
        end_offset = pss_signal.size() - index_second_pss;
    } else if (pss_signal.size()/2  > num_samples - index_second_pss) {
        end_offset = num_samples - index_second_pss;
        begin_offset = pss_signal.size() - (num_samples - index_second_pss);
    }

    // Extracting the signal around the PSS approximation
    int count = 0;
    for (int i = -begin_offset; i < end_offset; i ++){
        pss_signal[count] = buff[(int) index_second_pss + i];
        count ++;
    }

    int synchronisation_index;
    float peak_value;

    /*
     * Downsample signal for better performance
     */
    int downsampling_factor = fft_size / free5GRAN::PSS_SSS_FFT_SIZE;
    BOOST_LOG_TRIVIAL(trace) << "PSS synchronization downsampling factor: " << downsampling_factor;
    int symbol_duration_downsampled = symbol_duration / downsampling_factor;

    vector<complex<float>> pss_signal_downsampled(48 * symbol_duration_downsampled);
    for (int i = 0; i < 48 * symbol_duration_downsampled; i ++){
        pss_signal_downsampled[i] = pss_signal[downsampling_factor * i];
    }

    free5GRAN::phy::synchronization::search_pss(this->n_id_2,synchronisation_index,peak_value, common_cp_length / downsampling_factor, pss_signal_downsampled,fft_size / downsampling_factor);

    int pss_start_index = downsampling_factor * (synchronisation_index - symbol_duration_downsampled + 1);

    /*
     * Once synchronization is made on downsampled signal, it can be performed on full signal for finer results
     */
    vector<complex<float>> fine_pss_signal(symbol_duration + (2 * downsampling_factor + 1));
    count = 0;
    for (int i = pss_start_index - downsampling_factor; i < pss_start_index + symbol_duration + (downsampling_factor + 1); i ++){
        fine_pss_signal[count] = pss_signal[i];
        count ++;
    }
    free5GRAN::phy::synchronization::search_pss(this->n_id_2,synchronisation_index,peak_value, common_cp_length, fine_pss_signal,fft_size);

    pss_start_index = pss_start_index + (synchronisation_index - symbol_duration + 1  - downsampling_factor);
    int buffer_pss_index = pss_start_index + index_second_pss - begin_offset;
    index_first_pss = buffer_pss_index;
    int sss_init_index = buffer_pss_index + 2 * symbol_duration + common_cp_length;

    vector<complex<float>> sss_signal(fft_size);
    /*
     * Extracting the SSS signal based on sss_init_index and common_cp_length
     */
    for (int i = 0; i < fft_size; i++){
        sss_signal[i] = buff[i + sss_init_index];
    }

    float peak_value_sss;
    /*
     * Get SSS correlation result
     */
    free5GRAN::phy::synchronization::get_sss(this->n_id_1, peak_value_sss, sss_signal,fft_size,this->n_id_2);
    if (pci == 3 * n_id_1 + n_id_2){
        BOOST_LOG_TRIVIAL(trace) << "PCI confirmed";
        cell_confirmed = true;
    }else{
        BOOST_LOG_TRIVIAL(trace) << "PCI not confirmed";
        cell_confirmed = false;
        return 1;
    }
    /*
     * WE ARE NOW SYNCHONIZED IN OUR NEW FRAME
     * Trying to extract DMRS AND PBCH
     */

    vector<complex<float>> ssb_signal(4 * symbol_duration), new_ssb_signal(4 * symbol_duration);

    // Extract SSB signal
    for (int i = 0; i < free5GRAN::NUM_SYMBOLS_SSB * symbol_duration; i ++){
        ssb_signal[i] = buff[i + buffer_pss_index];
    }

    /*
     * Fine frequency correlation
     * Getting phase offset between CP and corresponding part of the OFDM symbol for each of the 4 symbols.
     * phase_offset is the mean phase offset
     */
    free5GRAN::phy::signal_processing::compute_fine_frequency_offset(ssb_signal, symbol_duration, fft_size, common_cp_length, scs, freq_offset, free5GRAN::NUM_SYMBOLS_SSB);

    // Correcting signal based on frequency offset
    free5GRAN::phy::signal_processing::transpose_signal(&buff, freq_offset, rf_device->getSampleRate(), buff.size());

    vector<complex<float>> pbch_modulation_symbols(free5GRAN::SIZE_SSB_PBCH_SYMBOLS), final_pbch_modulation_symbols(free5GRAN::SIZE_SSB_PBCH_SYMBOLS);

    /*
     * Extracting DMRS AND PBCH modulation symbols
     * ref is the reference grid for resource element demapper
     */
    vector<complex<float>> temp_mod_symbols, temp_mod_symbols2,temp_mod_symbols_dmrs;

    complex<float> pbch_symbols[free5GRAN::SIZE_SSB_PBCH_SYMBOLS];
    complex<float> dmrs_symbols[free5GRAN::SIZE_SSB_DMRS_SYMBOLS];
    complex<float> sss_symbols[free5GRAN::SIZE_PSS_SSS_SIGNAL];
    /*
     * ref[0] -> indexes of PBCH resource elements
     * ref[1] -> indexes of DMRS resource elements
     */
    vector<vector<vector<int>>> ref(3, vector<vector<int>>(free5GRAN::SIZE_SSB_DMRS_SYMBOLS, vector<int>(free5GRAN::NUM_SC_SSB)));
    /*
     * channel_indexes[0] contains PBCH samples indexes
     * channel_indexes[1] contains DMRS samples indexes
     * channel_indexes[2] contains SSS samples indexes
     */
    vector<vector<vector<int>>> channel_indexes = {vector<vector<int>>(2, vector<int>(free5GRAN::SIZE_SSB_PBCH_SYMBOLS)), vector<vector<int>>(2, vector<int>(free5GRAN::SIZE_SSB_DMRS_SYMBOLS)), vector<vector<int>>(2, vector<int>(free5GRAN::SIZE_PSS_SSS_SIGNAL))};

    vector<vector<complex<float>>> ssb_symbols(free5GRAN::NUM_SYMBOLS_SSB - 1, vector<complex<float>>(free5GRAN::NUM_SC_SSB));

    int cum_sum_fft[free5GRAN::NUM_SYMBOLS_SSB];
    for (int symbol = 0; symbol < free5GRAN::NUM_SYMBOLS_SSB; symbol ++){
        cum_sum_fft[symbol] = symbol * symbol_duration;
    }

    /*
     * Recover RE grid from time domain signal
     */
    free5GRAN::phy::signal_processing::fft(ssb_signal, ssb_symbols,fft_size,cp_lengths_pbch,&cum_sum_fft[0],free5GRAN::NUM_SYMBOLS_SSB - 1,free5GRAN::NUM_SC_SSB,1,0);

    free5GRAN::phy::physical_channel::compute_pbch_indexes(ref, pci);
    /*
     * Channel demapping using computed ref grid
     */
    complex<float>* output_channels[] = {pbch_symbols, dmrs_symbols, sss_symbols};
    free5GRAN::phy::signal_processing::channel_demapper(ssb_symbols, ref, output_channels, channel_indexes, 3, free5GRAN::NUM_SYMBOL_PBCH_SSB, free5GRAN::NUM_SC_SSB);

    /*
     * Channel estimation and equalization
     * Creating coefficients arrays
     */
    vector<vector<vector<complex<float>>>> coefficients(free5GRAN::MAX_I_BAR_SSB, vector<vector<complex<float>>>(free5GRAN::NUM_SYMBOL_PBCH_SSB, vector<complex<float>>(free5GRAN::NUM_SC_SSB)));


    complex<float> dmrs_sequence[free5GRAN::SIZE_SSB_DMRS_SYMBOLS];
    float snr[free5GRAN::MAX_I_BAR_SSB];

    /*
     * For each possible iBarSSB value, estimate the corresponding transport_channel
     */
    for (int i = 0; i < free5GRAN::MAX_I_BAR_SSB; i ++){
        free5GRAN::utils::sequence_generator::generate_pbch_dmrs_sequence(pci,i,dmrs_sequence);
        free5GRAN::phy::signal_processing::channelEstimation(dmrs_symbols, dmrs_sequence, channel_indexes[1],coefficients[i], snr[i], free5GRAN::NUM_SC_SSB, free5GRAN::NUM_SYMBOL_PBCH_SSB , free5GRAN::SIZE_SSB_DMRS_SYMBOLS);
    }
    /*
     * Choose the iBarSSB value that maximizes the SNR
     */
    max_snr = snr[0];
    int i_b_ssb = 0;
    for (int i = 1; i < free5GRAN::MAX_I_BAR_SSB ; i ++){
        if (snr[i] > max_snr){
            max_snr = snr[i];
            i_b_ssb = i;
        }
    }

    // Equalize transport_channel
    for (int i = 0; i < free5GRAN::SIZE_SSB_PBCH_SYMBOLS; i ++){
        final_pbch_modulation_symbols[i] = (pbch_symbols[i]) * conj(coefficients[i_b_ssb][channel_indexes[0][0][i]][channel_indexes[0][1][i]]) / (float) pow(abs(coefficients[i_b_ssb][channel_indexes[0][0][i]][channel_indexes[0][1][i]]),2);
    }

    ss_pwr.ss_rsrp = 0;
    for (int i = 0; i < free5GRAN::SIZE_PSS_SSS_SIGNAL; i ++){
        ss_pwr.ss_rsrp +=pow(abs(sss_symbols[i]),2);
    }
    for (int i = 0; i < free5GRAN::SIZE_SSB_DMRS_SYMBOLS; i ++){
        ss_pwr.ss_rsrp +=pow(abs(dmrs_symbols[i]),2);
    }
    ss_pwr.ss_rsrp /= (free5GRAN::SIZE_PSS_SSS_SIGNAL + free5GRAN::SIZE_SSB_DMRS_SYMBOLS);

    ss_pwr.ss_rssi = 0;
    for (int symb = 0 ; symb < free5GRAN::NUM_SYMBOLS_SSB - 1; symb++){
        for (int sc = 0; sc < free5GRAN::NUM_SC_SSB; sc ++){
            ss_pwr.ss_rssi += pow(abs(ssb_symbols[symb][sc]),2);
        }
    }
    // 20 is the number of RB in SSB block
    int n_rb = 20;
    ss_pwr.ss_rssi /= n_rb;
    ss_pwr.ss_rsrq = 10 * log(n_rb * ss_pwr.ss_rsrp / ss_pwr.ss_rssi);
    // Converting RSRP and RSSI from W to dBm
    ss_pwr.ss_rsrp  = 10 * log10(ss_pwr.ss_rsrp) + 30;
    ss_pwr.ss_rssi = 10 * log10(ss_pwr.ss_rssi) + 30;
    ss_pwr.ss_sinr = max_snr;

    this->i_b_ssb = i_b_ssb;
    if (l_max == 4){
        this-> i_ssb = i_b_ssb % 4;
    }else {
        this-> i_ssb = i_b_ssb;
    }
    this->pci = pci;

    /*
     * Physical and transport channel decoding
     * MIB parsing
     */
    int bch_bits[free5GRAN::SIZE_SSB_PBCH_SYMBOLS * 2];
    free5GRAN::phy::physical_channel::decode_pbch(final_pbch_modulation_symbols, i_ssb, pci, bch_bits);
    int mib_bits[free5GRAN::BCH_PAYLOAD_SIZE];
    free5GRAN::phy::transport_channel::decode_bch(bch_bits, crc_validated, mib_bits, pci);
    free5GRAN::utils::common_utils::parse_mib(mib_bits, mib_object);
    return 0;
}

phy::phy() {

}

void phy::print_cell_info() {
    /**
     * \fn print_cell_info
     * \brief Print cells global informations and MIB.
    */
    cout << "\n";
    cout << "###### RADIO" << endl;
    cout << "# SS-RSRP: " + to_string(ss_pwr.ss_rsrp) + " dbm" << endl;
    cout << "# SS-RSSI: " + to_string(ss_pwr.ss_rssi) + " dbm" << endl;
    cout << "# SS-RSRQ: " + to_string(ss_pwr.ss_rsrq) + " db" << endl;
    cout << "# SS-SNR: " + to_string(ss_pwr.ss_sinr) + " db" << endl;
    cout << "# Frequency offset: " + to_string(freq_offset) + " Hz" << endl;
    cout << "\n";
    cout << "###### CELL" << endl;
    cout << "## PCI: " + to_string(pci) + ((cell_confirmed) ? " (confirmed)" :  " (not confirmed)") << endl;
    cout << "## CP: ";
    cout << ((is_extended_cp == 0 ) ? "Normal" :  "Extended") << endl;
    cout << "## I_B_SSB: " + to_string(i_b_ssb) << endl;
    cout << "## I_SSB: " + to_string(i_ssb) << endl;
    cout << "\n";
    cout << "###### MIB" << endl;
    cout << "## Frame number: " + to_string(mib_object.sfn) << endl;
    cout << "## PDCCH configuration: " + to_string(mib_object.pdcch_config) << endl;
    cout << "## Subcarrier spacing common: " + to_string(mib_object.scs) << endl;
    cout << "## Cell barred: " + to_string(mib_object.cell_barred) << endl;
    cout << "## DMRS type A position : " + to_string(mib_object.dmrs_type_a_position) << endl;
    cout << "## k SSB: " + to_string(mib_object.k_ssb) << endl;
    cout << "## Intra freq reselection: " + to_string(mib_object.intra_freq_reselection) << endl;
    cout << "## CRC ";
    cout << ((crc_validated) ? "validated" :  "not validated") << endl;
    cout << "\n";
    cout << "#######################################################################" << endl;
    cout << "\n";
}

void phy::reconfigure(int fft_size) {
    this->fft_size = fft_size;
}

void phy::search_pdcch(bool &dci_found) {
    /**
     * \fn search_pdcch
     * \brief PDCCH config extraction, PDCCH blind search and DCI decoding
     * \standard TS 38.213 13
     * \details
     * - Read PDCCH config from MIB
     * - Detect frame beginning
     * - Select frame containing PDCCH and PDSCH based of SFN
     * - Frequency calibration to retrieve center on CORESET0
     * - Compute CCE to REG mapping
     * - Blind search DCI decoding over different candidates:
     *  -# Select a candidate
     *  -# Perform resource element de-mapping and FFT
     *  -# Channel estimation & equalization
     *  -# PDCCH decoding
     *  -# DCI decoding
     *  -# If CRC is validated, candidate is selected and function ends
     *  -# Otherwise, function continues with another candidates
     *
     * \param[out] dci_found: returns true if blind decode succeeds.
    */

    /*
     * If SSB offset is greater than 23, PDCCH is not present in the current BWP
     */
    if(mib_object.k_ssb > 23){
        dci_found = false;
        return;
    }
    mu = log2(mib_object.scs/15);
    int symbol_in_frame = band_object.ssb_symbols[this->i_ssb];
    frame_size = 0.01 * rf_device->getSampleRate();
    num_slots_per_frame = 10 * mib_object.scs/15;


    /*
     * Computing CP lengths of SSB/PBCH and recovering SSB position in frame
     */
    int num_symbols_per_subframe_pbch = free5GRAN::NUMBER_SYMBOLS_PER_SLOT_NORMAL_CP * (int) (scs/15e3);
    int cp_lengths_pbch[num_symbols_per_subframe_pbch];
    int cum_sum_pbch[num_symbols_per_subframe_pbch];

    free5GRAN::phy::signal_processing::compute_cp_lengths((int) scs/1e3, fft_size, is_extended_cp, num_symbols_per_subframe_pbch, &cp_lengths_pbch[0], &cum_sum_pbch[0]);

    int num_samples_before_pss = (symbol_in_frame / free5GRAN::NUMBER_SYMBOLS_PER_SLOT_NORMAL_CP) * (15e3/scs * frame_size / 10.0) + cum_sum_pbch[symbol_in_frame % free5GRAN::NUMBER_SYMBOLS_PER_SLOT_NORMAL_CP];
    int num_samples_after_pss = frame_size - num_samples_before_pss;

    /*
     * Computing new FFT size, based on MIB common SCS
     */
    fft_size = (int) (rf_device->getSampleRate() / (1e3 * mib_object.scs));

    BOOST_LOG_TRIVIAL(trace) << "###### PDCCH Search & decode";
    BOOST_LOG_TRIVIAL(trace) << "## INDEX_1: " + to_string(mib_object.pdcch_config/16);
    BOOST_LOG_TRIVIAL(trace) << "## INDEX_2: " + to_string(mib_object.pdcch_config%16);
    BOOST_LOG_TRIVIAL(trace) << "## SYMBOL IN FRAME: " + to_string(symbol_in_frame);
    BOOST_LOG_TRIVIAL(trace) << "## FRAME SIZE: " + to_string(frame_size);
    BOOST_LOG_TRIVIAL(trace) << "## INDEX PSS: " + to_string(index_first_pss);
    BOOST_LOG_TRIVIAL(trace) << "## SLOTS PER FRAME: " + to_string(num_slots_per_frame);
    BOOST_LOG_TRIVIAL(trace) << "## SAMPLES AFTER PSS: " + to_string(num_samples_after_pss);
    BOOST_LOG_TRIVIAL(trace) << "## BUFFER SIZE: " + to_string(buff.size());
    BOOST_LOG_TRIVIAL(trace) << "## FFT SIZE: " + to_string(fft_size);

    /*
     * Getting two candidate frames in received signal.
     * frame_indexes are the beginning and ending indexes of the two candidate frames
     * frame_numbers stores SFN for each candidate frame
     */
    vector<vector<int>> frame_indexes(2, vector<int>(2));
    int frame_numbers[2];
    free5GRAN::phy::signal_processing::get_candidates_frames_indexes(frame_indexes,frame_numbers,mib_object.sfn, index_first_pss,num_samples_before_pss, frame_size);

    BOOST_LOG_TRIVIAL(trace) << "## FRAME 1 FROM: " + to_string(1e3 * frame_indexes[0][0]/rf_device->getSampleRate()) + " TO: " + to_string(1e3 * frame_indexes[0][1]/rf_device->getSampleRate()) + " ms";
    BOOST_LOG_TRIVIAL(trace) << "## FRAME 2 FROM: " + to_string(1e3 * frame_indexes[1][0]/rf_device->getSampleRate()) + " TO: " + to_string(1e3 * frame_indexes[1][1]/rf_device->getSampleRate()) + " ms";

    /*
     * Computing PDCCH Search Space information
     */
    pdcch_ss_mon_occ = free5GRAN::phy::signal_processing::compute_pdcch_t0_ss_monitoring_occasions(mib_object.pdcch_config, scs, mib_object.scs * 1e3, i_ssb);
    pdcch_ss_mon_occ.n0 = (int)(pdcch_ss_mon_occ.O * pow(2, mu) + floor(i_ssb * pdcch_ss_mon_occ.M)) % num_slots_per_frame;
    pdcch_ss_mon_occ.sfn_parity = (int)((pdcch_ss_mon_occ.O * pow(2, mu) + floor(i_ssb * pdcch_ss_mon_occ.M)) / num_slots_per_frame) % 2;

    BOOST_LOG_TRIVIAL(trace) << "## n0: " + to_string(pdcch_ss_mon_occ.n0) ;
    BOOST_LOG_TRIVIAL(trace) << "## ODD/EVEN ?: " + to_string(pdcch_ss_mon_occ.sfn_parity);

    /*
     * Getting candidate frame which satisfies Search Space SFN parity
     */
    int frame;
    if (frame_numbers[0] % 2 == pdcch_ss_mon_occ.sfn_parity){
        frame = 0;
    }else {
        frame = 1;
    }

    BOOST_LOG_TRIVIAL(trace) << "## FRAME: " + to_string(frame);

    /*
     * Normalizing signal
     */
    complex<float> rms = 0;
    frame_data.resize(frame_size);
    for (int i = 0; i < frame_size; i ++){
        frame_data[i] = buff[i + frame_indexes[frame][0]];
        rms += abs(pow(frame_data[i],2));
    }
    rms = sqrt(rms/complex<float>(frame_size,0));
    for (int i = 0; i < frame_size; i ++){
        frame_data[i] = frame_data[i] / rms;
    }

    /*
     * Computing phase offset from SSB, based on RB offset and k_ssb and transposing signal to center on current BWP (which is here CORESET0)
     */
    float freq_diff = 12 * 1e3 * mib_object.scs * (pdcch_ss_mon_occ.n_rb_coreset / 2 - (10 * ((float) scs / (1e3*mib_object.scs)) + pdcch_ss_mon_occ.offset));
    float freq_diff2 = - 15e3 * mib_object.k_ssb;
    free5GRAN::phy::signal_processing::transpose_signal(&frame_data, freq_diff + freq_diff2 , rf_device->getSampleRate(), frame_size);

    BOOST_LOG_TRIVIAL(trace) << "## FREQ DIFF 1: " + to_string(freq_diff);
    BOOST_LOG_TRIVIAL(trace) << "## FREQ DIFF 2: " + to_string(freq_diff2);

    /*
     * Logging frame data to text file for plotting
     */
    ofstream data;
    data.open("output_files/studied_frame.txt");
    for (int i = 0; i < frame_size; i ++){
        data << frame_data[i];
        data << "\n";
    }
    data.close();

    /*
     * Plotting 4 slots around PDCCH monitoring slots
     */
    int begin_index = (pdcch_ss_mon_occ.n0-1) * frame_size / num_slots_per_frame;
    begin_index = max(begin_index,0);
    ofstream data2;
    data2.open("output_files/moniroting_slots.txt");
    for (int i = 0; i < 4 * frame_size / num_slots_per_frame; i ++){
        data2 << frame_data[i + begin_index];
        data2 << "\n";
    }
    data2.close();

    /*
     * Initialize arrays
     */
    int num_sc_coreset_0 = 12 * pdcch_ss_mon_occ.n_rb_coreset;

    /*
     * Compute CCE-to-REG mapping From TS38.211 7.3.2.2
     */
    int height_reg_rb = free5GRAN::NUMBER_REG_PER_CCE / pdcch_ss_mon_occ.n_symb_coreset;
    int R = 2;
    int C = pdcch_ss_mon_occ.n_rb_coreset / (height_reg_rb * R);
    int j;
    int reg_index[C * R];
    for (int c = 0; c < C; c ++){
        for (int r = 0; r < R; r ++){
            j = c * R + r;
            reg_index[j] = (r * C + c + this->pci) % (pdcch_ss_mon_occ.n_rb_coreset/height_reg_rb);
        }
    }
    for (int i = 0 ; i < C * R ; i ++){
        BOOST_LOG_TRIVIAL(trace) << "## CCE"+ to_string(i) + ": REG" + to_string(reg_index[i]);
    }

    /*
     * Computing current BWP CP lengths
     */
    int num_symbols_per_subframe_pdcch = free5GRAN::NUMBER_SYMBOLS_PER_SLOT_NORMAL_CP * mib_object.scs/15;
    int cp_lengths_pdcch[num_symbols_per_subframe_pdcch];
    int cum_sum_pdcch[num_symbols_per_subframe_pdcch];
    free5GRAN::phy::signal_processing::compute_cp_lengths(mib_object.scs, fft_size, is_extended_cp, num_symbols_per_subframe_pdcch, &cp_lengths_pdcch[0], &cum_sum_pdcch[0]);

    ofstream data_pdcch;

    int K, freq_domain_ra_size;
    /*
     * Number of bits for Frequency domain allocation in DCI
     */
    freq_domain_ra_size = ceil(log2(pdcch_ss_mon_occ.n_rb_coreset*(pdcch_ss_mon_occ.n_rb_coreset+1) / 2));
    /*
     * K is the DCI payload size including CRC
     */
    K = freq_domain_ra_size + 4 + 1 + 5 + 2 + 1 + 15 + 24;

    float snr;
    bool validated = false;
    int agg_level, num_candidates, dci_decoded_bits[K-24];
    vector<vector<complex<float>>> global_sequence(pdcch_ss_mon_occ.n_symb_coreset, vector<complex<float>>(pdcch_ss_mon_occ.n_rb_coreset * 3));
    vector<vector<vector<int>>> ref(2, vector<vector<int>>(pdcch_ss_mon_occ.n_symb_coreset, vector<int>(12 * pdcch_ss_mon_occ.n_rb_coreset)));
    vector<vector<complex<float>>> coreset_0_samples(pdcch_ss_mon_occ.n_symb_coreset, vector<complex<float>>(num_sc_coreset_0));
    vector<vector<complex<float>>> coefficients(pdcch_ss_mon_occ.n_symb_coreset, vector<complex<float>>(num_sc_coreset_0));

    /*
     * PDCCH blind search. First, loop over every monitoring slot
     */
    BOOST_LOG_TRIVIAL(trace) << "### PDCCH BLIND SEARCH";
    for (int monitoring_slot = 0; monitoring_slot < 2; monitoring_slot ++){
        pdcch_ss_mon_occ.monitoring_slot = monitoring_slot;
        BOOST_LOG_TRIVIAL(trace) << "## MONITORING SLOT: "+ to_string(monitoring_slot);

        /*
         * Extract corresponding CORESET0 samples. CORESET0 number of symbols is given by PDCCH config in MIB
         * Recover RE grid from time domain signal
         */
        free5GRAN::phy::signal_processing::fft(frame_data, coreset_0_samples,fft_size,cp_lengths_pdcch,cum_sum_pdcch,pdcch_ss_mon_occ.n_symb_coreset,num_sc_coreset_0,pdcch_ss_mon_occ.first_symb_index, (pdcch_ss_mon_occ.n0 + monitoring_slot) * frame_size / num_slots_per_frame);
        for (int symb = 0; symb < pdcch_ss_mon_occ.n_symb_coreset; symb ++){
            /*
             * Generate DMRS sequence for corresponding symbols
             */
            free5GRAN::utils::sequence_generator::generate_pdcch_dmrs_sequence(pci, pdcch_ss_mon_occ.n0 + monitoring_slot, pdcch_ss_mon_occ.first_symb_index + symb, global_sequence[symb], pdcch_ss_mon_occ.n_rb_coreset * 3);
        }
        /*
         * Loop over possible aggregation level (from 2 to 4 included) and candidates
         */
        for (int i = 2; i < 5; i ++){
            agg_level = pow(2, i);
            if (agg_level <= pdcch_ss_mon_occ.n_rb_coreset / height_reg_rb){
                vector<vector<vector<int>>> channel_indexes = {vector<vector<int>>(2, vector<int>((size_t) agg_level * free5GRAN::NUMBER_REG_PER_CCE * 9)), vector<vector<int>>(2, vector<int>((size_t) agg_level * free5GRAN::NUMBER_REG_PER_CCE * 3))};
                vector<complex<float>> pdcch_symbols((size_t) agg_level * free5GRAN::NUMBER_REG_PER_CCE * 9);
                complex<float> temp_pdcch_symbols[agg_level * free5GRAN::NUMBER_REG_PER_CCE * 9];
                complex<float> dmrs_symbols[agg_level * free5GRAN::NUMBER_REG_PER_CCE * 3];
                complex<float> dmrs_sequence[agg_level * free5GRAN::NUMBER_REG_PER_CCE * 3];
                int reg_bundles[agg_level];
                int reg_bundles_ns[agg_level];
                int dci_bits[agg_level * free5GRAN::NUMBER_REG_PER_CCE * 9 * 2];

                BOOST_LOG_TRIVIAL(trace) << "## AGGREGATION LEVEL"+ to_string(agg_level);
                num_candidates = pdcch_ss_mon_occ.n_rb_coreset / (agg_level * height_reg_rb);
                /*
                 * Loop over candidates of current aggregation level
                 */
                for (int p = 0; p < num_candidates; p ++){
                    BOOST_LOG_TRIVIAL(trace) << "## CANDIDATE "+ to_string(p);
                    /*
                     * Extract REG bundles for current candidate and aggregation level
                     */
                    for (int l = 0; l < agg_level; l ++){
                        reg_bundles[l] = reg_index[l + p * agg_level];
                        reg_bundles_ns[l] = reg_index[l + p * agg_level];
                    }
                    sort(reg_bundles, reg_bundles+agg_level);
                    /*
                     * PDCCH samples extraction
                     */
                    for (int symbol = 0; symbol < pdcch_ss_mon_occ.n_symb_coreset; symbol ++) {
                        for (int sc = 0; sc < 12 * pdcch_ss_mon_occ.n_rb_coreset; sc ++){
                            ref[1][symbol][sc] = 0;
                            ref[0][symbol][sc] = 0;
                        }
                    }
                    /*
                     * Computing PDCCH candidate position in RE grid
                     */
                    free5GRAN::phy::physical_channel::compute_pdcch_indexes(ref, pdcch_ss_mon_occ, agg_level, reg_bundles, height_reg_rb);
                    /*
                     * Channel de-mapping
                     */
                    complex<float>* output_channels[] = {temp_pdcch_symbols,dmrs_symbols};
                    free5GRAN::phy::signal_processing::channel_demapper(coreset_0_samples, ref, output_channels, channel_indexes, 2, pdcch_ss_mon_occ.n_symb_coreset, 12 * pdcch_ss_mon_occ.n_rb_coreset);
                    /*
                     * DMRS CCE-to-REG de-mapping/de-interleaving
                     */
                    for (int k = 0 ; k < agg_level; k ++){
                        for (int reg = 0; reg < free5GRAN::NUMBER_REG_PER_CCE; reg ++){
                            dmrs_sequence[((agg_level * free5GRAN::NUMBER_REG_PER_CCE * 3) / pdcch_ss_mon_occ.n_symb_coreset) * (reg%pdcch_ss_mon_occ.n_symb_coreset) +  k * height_reg_rb * 3 + (reg/pdcch_ss_mon_occ.n_symb_coreset) * 3] = global_sequence[reg%pdcch_ss_mon_occ.n_symb_coreset][reg_bundles[k] * height_reg_rb * 3 + (reg/pdcch_ss_mon_occ.n_symb_coreset) * 3];
                            dmrs_sequence[((agg_level * free5GRAN::NUMBER_REG_PER_CCE * 3) / pdcch_ss_mon_occ.n_symb_coreset) * (reg%pdcch_ss_mon_occ.n_symb_coreset) +  k * height_reg_rb * 3 + (reg/pdcch_ss_mon_occ.n_symb_coreset) * 3 + 1] = global_sequence[reg%pdcch_ss_mon_occ.n_symb_coreset][reg_bundles[k] * height_reg_rb * 3 + (reg/pdcch_ss_mon_occ.n_symb_coreset) * 3 + 1];
                            dmrs_sequence[((agg_level * free5GRAN::NUMBER_REG_PER_CCE * 3) / pdcch_ss_mon_occ.n_symb_coreset) * (reg%pdcch_ss_mon_occ.n_symb_coreset) +  k * height_reg_rb * 3 + (reg/pdcch_ss_mon_occ.n_symb_coreset) * 3 + 2] = global_sequence[reg%pdcch_ss_mon_occ.n_symb_coreset][reg_bundles[k] * height_reg_rb * 3 + (reg/pdcch_ss_mon_occ.n_symb_coreset) * 3 + 2];
                        }
                    }
                    /*
                     * Channel estimation
                     */
                    free5GRAN::phy::signal_processing::channelEstimation(dmrs_symbols, dmrs_sequence, channel_indexes[1],coefficients, snr, 12 * pdcch_ss_mon_occ.n_rb_coreset, pdcch_ss_mon_occ.n_symb_coreset , agg_level * free5GRAN::NUMBER_REG_PER_CCE * 3);
                    /*
                     * Channel equalization
                     */
                    for (int sc = 0; sc < agg_level * free5GRAN::NUMBER_REG_PER_CCE * 9; sc ++){
                        pdcch_symbols[sc] = (temp_pdcch_symbols[sc]) * conj(coefficients[channel_indexes[0][0][sc]][channel_indexes[0][1][sc]]) / (float) pow(abs(coefficients[channel_indexes[0][0][sc]][channel_indexes[0][1][sc]]),2);
                    }
                    /*
                     * PDCCH and DCI decoding
                     */
                    free5GRAN::phy::physical_channel::decode_pdcch(pdcch_symbols,dci_bits,agg_level, reg_bundles_ns, reg_bundles, pci);
                    free5GRAN::phy::transport_channel::decode_dci(dci_bits, agg_level * free5GRAN::NUMBER_REG_PER_CCE * 9 * 2, K, free5GRAN::SI_RNTI, validated, dci_decoded_bits);
                    /*
                     * If DCI CRC is validated, candidate is validated, blind search ends
                     */
                    if (validated){
                        data_pdcch.open("output_files/pdcch_constellation.txt");
                        for (int sc = 0; sc < agg_level * free5GRAN::NUMBER_REG_PER_CCE * 9; sc ++){
                            data_pdcch << pdcch_symbols[sc];
                            data_pdcch << "\n";
                        }
                        data_pdcch.close();
                        goto dci_found_and_validated;
                    }

                }
            }else {
                break;
            }
        }
    }

    dci_found_and_validated:
    BOOST_LOG_TRIVIAL(trace) << "## DCI FOUND AND " << ((validated) ? "VALIDATED" :  "NOT VALIDATED");

    dci_found = false;
    if (validated){
        parse_dci_1_0_si_rnti(dci_decoded_bits,freq_domain_ra_size,dci_1_0_si_rnti);
        n_size_bwp = pdcch_ss_mon_occ.n_rb_coreset;
        /*
         * In current version, only redundancy version 0 and 3 are supported for DL-SCH decoding
         */
        if (dci_1_0_si_rnti.rv == 0 || dci_1_0_si_rnti.rv == 3){
            dci_found = true;
        }

    }
}

void phy::print_dci_info() {
    /**
     * \fn print_dci_info
     * \brief Print DCI decoded informations
    */
    cout << "###### DCI" << endl;
    cout << "# RIV: " + to_string(dci_1_0_si_rnti.RIV)<< endl;
    cout << "# Time Domain RA: " + to_string(dci_1_0_si_rnti.TD_ra) << endl;
    cout << ((dci_1_0_si_rnti.vrb_prb_interleaving == 0 ) ? "# Non-interleaved VRB to PRB" :  "# Interleaved VRB to PRB") << endl;
    cout << "# Modulation coding scheme: " + to_string(dci_1_0_si_rnti.mcs) << endl;
    cout << "# Redudancy version: " + to_string(dci_1_0_si_rnti.rv) << endl;
    cout << ((dci_1_0_si_rnti.si == 0 ) ? "# SIB1 message" :  "# Other SIB message") << endl;
    cout << "#######################################################################" << endl;
    if (dci_1_0_si_rnti.rv == 1 || dci_1_0_si_rnti.rv == 2){
        cout << "WARNING: Redudancy version " + to_string(dci_1_0_si_rnti.rv) << " is not supported by current decoder. To decode SIB1 data on this cell, please use CELL_SEARCH function in config and specify the cell frequency. Retry until redundancy version is not 1 or 2" << endl;
        cout << "#######################################################################" << endl;
    }
    cout << "\n";
}

void phy::parse_dci_1_0_si_rnti(int *dci_bits, int freq_domain_ra_size, free5GRAN::dci_1_0_si_rnti &dci) {
    /**
     * \fn parse_dci_1_0_si_rnti
     * \brief Parse DCI informations
     * \param[in] dci_bits: DCI decoded bits
     * \param[in] freq_domain_ra_size: Number of bits used for frequency allocation in DCI
     * \param[out] dci: Filled DCI object
    */

    dci.RIV = 0;
    for (int i = 0 ; i < freq_domain_ra_size; i ++){
        dci.RIV += dci_bits[i] * pow(2, freq_domain_ra_size - i - 1);
    }
    dci.TD_ra = 0;
    for (int i = 0; i < 4; i ++){
        dci.TD_ra += dci_bits[i + freq_domain_ra_size] * pow(2, 4 - i - 1);
    }

    dci.vrb_prb_interleaving = dci_bits[freq_domain_ra_size + 4];

    dci.mcs = 0;
    for (int i = 0; i < 5; i ++){
        dci.mcs += dci_bits[i + freq_domain_ra_size + 4 + 1] * pow(2, 5 - i - 1);
    }

    dci.rv = 0;
    for (int i = 0; i < 2; i ++){
        dci.rv += dci_bits[i + freq_domain_ra_size + 4 + 1 + 5] * pow(2, 2 - i - 1);
    }

    dci.si = dci_bits[freq_domain_ra_size + 4 + 1 + 5 + 2];
}

void phy::extract_pdsch() {
    /**
     * \fn extract_pdsch
     * \brief PDSCH extraction, PDSCH decoding, DL-SCH decoding and SIB1 parsing
     * \details
     * - Parameters extraction from DCI and standard
     * - Phase de-compensation. Looping over different possible phase compensation:
     *  -# Signal extraction, FFT and resource element de-mapper
     *  -# Channel estimation & equalization
     *  -# PDSCH decoding
     *  -# DL-SCH decoding
     *  -# If CRC is validated, phase de-compensation is validated and functions continues. Otherwise, another phase de-compensation is tried.
     * - SIB1 parsing using ASN1C
    */

    BOOST_LOG_TRIVIAL(trace) << "#### DECODING PDSCH";
    /*
     * Extracting PDSCH time and frequency position
     */
    int lrb, rb_start, k0, S, L, mod_order, code_rate, l0;
    free5GRAN::phy::signal_processing::compute_rb_start_lrb_dci(dci_1_0_si_rnti.RIV, n_size_bwp,lrb,rb_start);
    k0 = free5GRAN::TS_38_214_TABLE_5_1_2_1_1_2[dci_1_0_si_rnti.TD_ra][mib_object.dmrs_type_a_position - 2][1];
    S = free5GRAN::TS_38_214_TABLE_5_1_2_1_1_2[dci_1_0_si_rnti.TD_ra][mib_object.dmrs_type_a_position - 2][2];
    L = free5GRAN::TS_38_214_TABLE_5_1_2_1_1_2[dci_1_0_si_rnti.TD_ra][mib_object.dmrs_type_a_position - 2][3];
    string mapping_type = ((free5GRAN::TS_38_214_TABLE_5_1_2_1_1_2[dci_1_0_si_rnti.TD_ra][mib_object.dmrs_type_a_position - 2][0] == 0 ) ? "A" :  "B");
    mod_order = free5GRAN::TS_38_214_TABLE_5_1_3_1_1[dci_1_0_si_rnti.mcs][0];
    code_rate = free5GRAN::TS_38_214_TABLE_5_1_3_1_1[dci_1_0_si_rnti.mcs][1];
    BOOST_LOG_TRIVIAL(trace) << "## Frequency domain RA: RB Start " + to_string(rb_start) + " and LRB " + to_string(lrb) ;
    BOOST_LOG_TRIVIAL(trace) << "## Time domain RA: K0 " + to_string(k0) + ", S " + to_string(S) + " and L " + to_string(L) + " (mapping type "+mapping_type+")";
    BOOST_LOG_TRIVIAL(trace) << "## MCS: Order " + to_string(mod_order) + " and code rate " + to_string(code_rate);
    BOOST_LOG_TRIVIAL(trace) << "## Slot number " + to_string(pdcch_ss_mon_occ.n0 + pdcch_ss_mon_occ.monitoring_slot + k0);

    /*
     * Compute number of additionnal DMRS positions
     */
    int additionnal_position;
    if (mapping_type == "A"){
        additionnal_position = 2;
    }else {
        if (L == 2 || L == 4){
            additionnal_position = 0;
        }else if(L == 7){
            additionnal_position = 1;
        }
    }

    int *dmrs_symbols, num_symbols_dmrs;
    /*
     * Get PDSCH DMRS symbols indexes
     */
    free5GRAN::phy::signal_processing::get_pdsch_dmrs_symbols(mapping_type, L + S, additionnal_position, mib_object.dmrs_type_a_position, &dmrs_symbols, num_symbols_dmrs);

    float snr;

    complex<float> dmrs_sequence[6 * lrb * num_symbols_dmrs];

    int count_dmrs_symbol = 0;
    bool dmrs_symbol;


    int num_symbols_per_subframe_pdsch = free5GRAN::NUMBER_SYMBOLS_PER_SLOT_NORMAL_CP * mib_object.scs/15;
    int cp_lengths_pdsch[num_symbols_per_subframe_pdsch];
    int cum_sum_pdsch[num_symbols_per_subframe_pdsch];

    vector<vector<complex<float>>> pdsch_ofdm_symbols(L, vector<complex<float>>(12 * pdcch_ss_mon_occ.n_rb_coreset)), pdsch_samples(L, vector<complex<float>>(12 * lrb));
    vector<vector<vector<int>>> ref(2, vector<vector<int>>(L, vector<int>(12 * lrb)));
    vector<vector<vector<int>>> channel_indexes = {vector<vector<int>>(2, vector<int>((size_t) 12 * lrb * (L - num_symbols_dmrs))), vector<vector<int>>(2, vector<int>((size_t) 6 * lrb * num_symbols_dmrs))};
    vector<vector<complex<float>>> coefficients(L, vector<complex<float>>(12 * lrb));
    complex<float> temp_dmrs_sequence[6 * pdcch_ss_mon_occ.n_rb_coreset], pdsch_samples_only[12 * lrb * (L - num_symbols_dmrs)], dmrs_samples_only[6 * lrb * num_symbols_dmrs];

    /*
     * Compute PDSCH CP lengths (same as PDCCH, as it is the same BWP)
     */
    free5GRAN::phy::signal_processing::compute_cp_lengths(mib_object.scs, fft_size, is_extended_cp, num_symbols_per_subframe_pdsch, cp_lengths_pdsch, cum_sum_pdsch);

    /*
     * Recover RE grid from time domain signal
     */
    free5GRAN::phy::signal_processing::fft(frame_data, pdsch_ofdm_symbols,fft_size,cp_lengths_pdsch,cum_sum_pdsch,L,12 * pdcch_ss_mon_occ.n_rb_coreset,S, (pdcch_ss_mon_occ.n0 + pdcch_ss_mon_occ.monitoring_slot + k0) * frame_size / num_slots_per_frame);

    bool dmrs_symbol_array[L];
    /*
     * PDSCH extraction
     */
    for (int symb = 0; symb < L; symb ++){
        dmrs_symbol = false;
        /*
         * Check if studied symbol is a DMRS
         */
        for (int j = 0; j < num_symbols_dmrs; j ++){
            if (symb + S == dmrs_symbols[j]){
                dmrs_symbol = true;
                break;
            }
        }
        dmrs_symbol_array[symb] = dmrs_symbol;
        /*
         * Get DMRS sequence
         */
        free5GRAN::utils::sequence_generator::generate_pdsch_dmrs_sequence(free5GRAN::NUMBER_SYMBOLS_PER_SLOT_NORMAL_CP, pdcch_ss_mon_occ.n0 + pdcch_ss_mon_occ.monitoring_slot + k0, symb + S, 0, pci, temp_dmrs_sequence, 6 * pdcch_ss_mon_occ.n_rb_coreset);
        if (dmrs_symbol){
            for (int i = 0; i < 6 * lrb; i ++){
                dmrs_sequence[count_dmrs_symbol * 6 * lrb + i] = temp_dmrs_sequence[rb_start * 6 + i];
            }
            count_dmrs_symbol += 1;
        }

        for (int i = 0; i < 12 * lrb; i ++){
            pdsch_samples[symb][i] = pdsch_ofdm_symbols[symb][12 * rb_start + i];
        }
    }
    free5GRAN::phy::physical_channel::compute_pdsch_indexes(ref, dmrs_symbol_array, L, lrb);
    /*
     * Channel de-mapping
     */
    complex<float>* output_channels[] = {pdsch_samples_only,dmrs_samples_only};
    free5GRAN::phy::signal_processing::channel_demapper(pdsch_samples, ref, output_channels, channel_indexes, 2, L, 12 * lrb);
    bool validated;
    float f0 = 0;
    float phase_offset;
    /*
    * Phase decompensator. As phase compensation is not known a priori, we have to loop over different possibles phase compensation for decoding
    */
    for (int phase_decomp_index = 0; phase_decomp_index < 50; phase_decomp_index++){
        /*
         * Compute phase decomp value
         */
        f0 += (phase_decomp_index % 2) * pow(2,mu) * 1e3;
        phase_offset = (phase_decomp_index % 2) ? f0 : -f0;
        BOOST_LOG_TRIVIAL(trace) << "PHASE DECOMP " << phase_offset ;
        complex<float> phase_decomp[num_symbols_per_subframe_pdsch];
        /*
         * Compute phase decompensation value for each symbol in  a subframe
         */
        free5GRAN::phy::signal_processing::compute_phase_decomp(cp_lengths_pdsch, cum_sum_pdsch, rf_device->getSampleRate(),phase_offset,num_symbols_per_subframe_pdsch,phase_decomp);
        /*
         * Phase de-compensation
         */
        for (int samp = 0; samp < 12 * lrb * (L - num_symbols_dmrs); samp ++){
            pdsch_samples_only[samp] = pdsch_samples_only[samp] * phase_decomp[((pdcch_ss_mon_occ.n0 + pdcch_ss_mon_occ.monitoring_slot + k0) % (num_slots_per_frame / 10)) * free5GRAN::NUMBER_SYMBOLS_PER_SLOT_NORMAL_CP + S + channel_indexes[0][0][samp]];
        }
        for (int samp = 0; samp < 6 * lrb * num_symbols_dmrs; samp ++){
            dmrs_samples_only[samp] = dmrs_samples_only[samp] * phase_decomp[((pdcch_ss_mon_occ.n0 + pdcch_ss_mon_occ.monitoring_slot + k0) % (num_slots_per_frame / 10)) * free5GRAN::NUMBER_SYMBOLS_PER_SLOT_NORMAL_CP + S + channel_indexes[1][0][samp]];
        }
        /*
         * Channel estimation
         */
        free5GRAN::phy::signal_processing::channelEstimation(dmrs_samples_only, dmrs_sequence, channel_indexes[1],coefficients, snr, 12 * lrb, L, 6 * lrb * num_symbols_dmrs);
        /*
         * Channel equalization
         */
        vector<complex<float>> pdsch_samples_vector((size_t) 12 * lrb * (L - num_symbols_dmrs));
        for (int sc = 0; sc <  12 * lrb * (L - num_symbols_dmrs); sc ++){
            pdsch_samples_vector[sc] = (pdsch_samples_only[sc]) * conj(coefficients[channel_indexes[0][0][sc]][channel_indexes[0][1][sc]]) / (float) pow(abs(coefficients[channel_indexes[0][0][sc]][channel_indexes[0][1][sc]]),2);
        }

        ofstream data_pdsch;
        data_pdsch.open("output_files/pdsch_constellation.txt");
        for (int i = 0; i < 12 * lrb * (L - num_symbols_dmrs); i ++){
            data_pdsch << pdsch_samples_vector[i];
            data_pdsch << "\n";
        }
        data_pdsch.close();

        /*
         * PDSCH and DL-SCH decoding
         */
        double dl_sch_bits[2 * pdsch_samples_vector.size()];
        free5GRAN::phy::physical_channel::decode_pdsch(pdsch_samples_vector, dl_sch_bits, pci);
        int n_re = free5GRAN::phy::signal_processing::compute_nre(L, num_symbols_dmrs);

        vector<int> desegmented = free5GRAN::phy::transport_channel::decode_dl_sch(dl_sch_bits, n_re, (float) code_rate / (float) 1024, lrb,2 * pdsch_samples_vector.size(), validated, dci_1_0_si_rnti);
        /*
         * If DL-SCH CRC is validated, Phase decompensation is validated
         */
        if (validated){
            int bytes_size = (int) ceil(desegmented.size()/8.0);
            uint8_t dl_sch_bytes[bytes_size];
            for (int i = 0; i < desegmented.size(); i ++){
                if (i % 8 == 0){
                    dl_sch_bytes[i/8] = 0;
                }
                dl_sch_bytes[i/8] += desegmented[i] * pow(2, 8 - (i%8) - 1);
            }
            asn_decode(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_BCCH_DL_SCH_Message,(void **) &sib1, dl_sch_bytes, bytes_size);
            break;
        }
    }

}

BCCH_DL_SCH_Message_t *phy::getSib() {
    return this->sib1;
}

void phy::print_sib1() {
    asn_fprint(stdout, &asn_DEF_BCCH_DL_SCH_Message, sib1);
}

int phy::getSIB1RV() {
    return dci_1_0_si_rnti.rv;
}
