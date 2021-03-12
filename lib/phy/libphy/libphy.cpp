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

#include "libphy.h"
#include "../../variables/common_variables/common_variables.h"
#include "../../utils/sequence_generator/sequence_generator.h"
#include "../../utils/common_utils/common_utils.h"
#include <complex>
#include <vector>
#include <algorithm>
#include <fftw3.h>
#include <iostream>
#include <fstream>

using namespace std;

void free5GRAN::phy::signal_processing::channelEstimation(complex<float> * pilots, complex<float> * reference_pilot, vector<vector<int>> &pilot_indexes, vector<vector<complex<float>>> &coefficients, float &snr, int num_sc, int num_symbols, int pilot_size){
    /**
     * \fn channelEstimation
     * \brief Estimate channel coefficients for equalization
     * \details
     * - Compute channel coefficients for pilots
     * - Linear interpolation in frequency domain
     * - Linear interpolation in time domain
     *
     * \param[in] pilots: Signal received pilots
     * \param[in] reference_pilot: Generated pilots
     * \param[in] pilot_indexes: Pilot indexes in the signal
     * \param[out] coefficients: Channel coefficient returned by the function
     * \param[out] snr: Computed SNR
     * \param[in] num_sc: Number of subcarriers in the signal
     * \param[in] num_symbols: Number of symbols in the signal
     * \param[in] pilot_size: Pilot size
    */

    // Initializing variables
    complex<float> noise_coef;
    vector<int> found_indexes, symbols_with_pilot,symbols_with_no_pilot;
    vector<complex<float>> coef_same_sc;
    int upper_index, lower_index, step, step_width;
    float re_int, im_int;
    float mean_power = 0;
    float mean_noise = 0;

    // Computing pilots transport_channel coefficients
    for (int i = 0; i < pilot_size; i ++){
        coefficients[pilot_indexes[0][i]][pilot_indexes[1][i]] = pilots[i] * conj(reference_pilot[i]) / (float) pow(abs(reference_pilot[i]),2);
    }

    // Interpolating in frequency domain to complete transport_channel coefficients grid
    for (int symbol = 0; symbol <num_symbols; symbol ++ ){
        found_indexes.clear();
        /*
         * Adding in found_indexes all the pilot index that are in the current symbol
         */
        for (int k = 0; k < pilot_size; k ++){
            if (pilot_indexes[0][k] == symbol){
                found_indexes.push_back(pilot_indexes[1][k]);
            }
        }


        if (found_indexes.size() != 0 ){
            symbols_with_pilot.push_back(symbol);
            /*
             * Linear interpolation. Looping over each subcarrier of the current symbol
             */
            for (int sc = 0; sc < num_sc; sc ++){
                // If the current resource element is not a pilot
                if(find(found_indexes.begin(), found_indexes.end(), sc) == found_indexes.end()) {
                    // Get lower and upper pilot indexes for interpolating. If no lower, take the two closest upper pilots and respectively if no upper pilot
                    if (sc < found_indexes[0]){
                        lower_index = found_indexes[0];
                        upper_index = found_indexes[1];
                    }else if(sc > found_indexes[found_indexes.size() - 1]){
                        lower_index = found_indexes[found_indexes.size() - 2];
                        upper_index = found_indexes[found_indexes.size() - 1];
                    }else{
                        for (int k = 0; k < found_indexes.size() - 1; k ++){
                            if (found_indexes[k] < sc && found_indexes[k + 1] > sc){
                                lower_index = found_indexes[k];
                                upper_index = found_indexes[k + 1];
                                break;
                            }
                        }
                    }
                    step = sc - lower_index;
                    step_width = upper_index - lower_index;
                    // Interpolate real and imaginary part of the resource element transport_channel coefficient
                    re_int = step * (real(coefficients[symbol][upper_index]) - real(coefficients[symbol][lower_index])) / step_width + real(coefficients[symbol][lower_index]);
                    im_int = step * (imag(coefficients[symbol][upper_index]) - imag(coefficients[symbol][lower_index])) / step_width + imag(coefficients[symbol][lower_index]);
                    coefficients[symbol][sc] = complex<float>(re_int,im_int);
                    // Compute mean power over all the resource elements
                    mean_power += pow(abs(coefficients[symbol][sc]),2);
                }
            }
        }else {
            symbols_with_no_pilot.push_back(symbol);
        }
    }

    /*
     *  Time domain linear interpolation
     */

    for (int s = 0; s < symbols_with_no_pilot.size(); s ++){
        if (symbols_with_no_pilot[s] < symbols_with_pilot[0]){
            lower_index = symbols_with_pilot[0];
            upper_index = symbols_with_pilot[1];
        }else if(symbols_with_no_pilot[s] > symbols_with_pilot[symbols_with_pilot.size() - 1]){
            lower_index = symbols_with_pilot[symbols_with_pilot.size() - 2];
            upper_index = symbols_with_pilot[symbols_with_pilot.size() - 1];
        }else{
            for (int k = 0; k < symbols_with_pilot.size() - 1; k ++){
                if (symbols_with_pilot[k] < symbols_with_no_pilot[s] && symbols_with_pilot[k + 1] > symbols_with_no_pilot[s]){
                    lower_index = symbols_with_pilot[k];
                    upper_index = symbols_with_pilot[k + 1];
                    break;
                }
            }
        }
        step = symbols_with_no_pilot[s] - lower_index;
        step_width = upper_index - lower_index;


        for (int sc = 0; sc < num_sc; sc ++){
            re_int = step * (real(coefficients[upper_index][sc]) - real(coefficients[lower_index][sc])) / step_width + real(coefficients[lower_index][sc]);
            im_int = step * (imag(coefficients[upper_index][sc]) - imag(coefficients[lower_index][sc])) / step_width + imag(coefficients[lower_index][sc]);
            coefficients[symbols_with_no_pilot[s]][sc] = complex<float>(re_int,im_int);
            // Compute mean power over all the resource elements
            mean_power += pow(abs(coefficients[symbols_with_no_pilot[s]][sc]),2);
        }

    }
    // Computing signal power.
    mean_power /= num_sc * num_symbols;
    mean_power = 10 * log10(mean_power);


    // Computing signal noise by time averaging the mean standard deviation of the coefficients on all the subcarriers
    for (int symbol = 0; symbol <num_symbols; symbol ++ ){
        noise_coef = 0;
        for (int sc = 0; sc < num_sc; sc ++){
            noise_coef += coefficients[symbol][sc];
        }
        noise_coef /= num_sc;
        for (int sc = 0; sc < num_sc; sc ++){
            mean_noise += pow(abs(coefficients[symbol][sc] - noise_coef),2) /num_sc ;
        }
    }
    mean_noise /= num_symbols;
    mean_noise = 10 * log10(mean_noise);

    snr = mean_power - mean_noise;
}

void free5GRAN::phy::signal_processing::hard_demodulation(vector<complex<float>> signal, int *bits, int signal_length, int modulation_scheme) {
    /**
     * \brief Samples hard demodulation
     * \param[in] signal: Input IQ data to be demodulated
     * \param[in] signal_length: Number of symbols to be demodulated
     * \param[in] modulation_scheme: Modulation used (0: BPSK, 1: QPSK)
     * \param[out] bits: Output hard bits
    */
    for (int i = 0; i < signal_length; i ++){
        /*
         * BPSK demodulation
         */
        if (modulation_scheme == 0){
            if (real(signal[i]) > 0){
                bits[i] = 0;
            }else {
                bits[i] = 1;
            }
        }
        /*
         * QPSK demodulation
         */
        else if(modulation_scheme == 1){
            if (real(signal[i]) > 0){
                bits[2 * i] = 0;
            }else {
                bits[2 * i] = 1;
            }
            if (imag(signal[i]) > 0){
                bits[2 * i + 1] = 0;
            }else {
                bits[2 * i + 1] = 1;
            }
        }
    }
}

void free5GRAN::phy::signal_processing::soft_demodulation(vector<complex<float>> signal, double *soft_bits, int signal_length, int modulation_scheme) {
    /**
     * \fn soft_demodulation
     * \brief Samples soft demodulation
     * \details
     * - Compute minimum distance to 1 and 0 candidate sample
     * - Take the difference between shortest distance to 1 and shortest distance to 0
     *
     * \param[in] signal: Input IQ data to be demodulated
     * \param[out] bits: Output LLR bits
     * \param[in] signal_length: Number of symbols to be demodulated
     * \param[in] modulation_scheme: Modulation used (0: BPSK, 1: QPSK)
     */
    /*
     * QPSK soft-demodulation
     */
    if(modulation_scheme == 1){
        double l_01,l_02, l_11, l_12, min0, min1;
        complex<float> s_00, s_01, s_10, s_11;
        double const_power = 1 / sqrt(2);
        s_00 = complex<double>(const_power,const_power);
        s_01 = complex<double>(const_power,-const_power);
        s_10 = complex<double>(-const_power,const_power);
        s_11 = complex<double>(-const_power,-const_power);
        for (int i = 0; i < signal_length; i ++){
            // Computing distance from current signal point to each of the four theoretical QPSK points
            l_01 = abs(signal[i] - s_00);
            l_02 = abs(signal[i] - s_01);
            l_11 = abs(signal[i] - s_10);
            l_12 = abs(signal[i] - s_11);
            // Determine LLR soft bit of the first QPSK bit
            soft_bits[2 * i] = pow(min(l_11,l_12),2) - pow(min(l_01,l_02),2);
            // Computing distance from current signal point to each of the four theoretical QPSK points
            l_01 = abs(signal[i] - s_00);
            l_02 = abs(signal[i] - s_10);
            l_11 = abs(signal[i] - s_01);
            l_12 = abs(signal[i] - s_11);
            // Determine LLR soft bit of the second QPSK bit
            soft_bits[2 * i + 1] = pow(min(l_11,l_12),2) - pow(min(l_01,l_02),2);
        }
    }

}

void free5GRAN::phy::signal_processing::compute_fine_frequency_offset(vector<complex<float>> input_signal, int symbol_duration, int fft_size, int cp_length, int scs, float &output, int num_symbols) {
    /**
     * \fn compute_fine_frequency_offset
     * \brief  Compute fine frequency offset of a received signal by computing phase offset between cyclic prefix and corresponding symbol data.
     * \param[in] input_signal: Received signal
     * \param[in] symbol_duration: Number of samples per symbol
     * \param[in] fft_size: Number of samples in a symbol, after removing cyclic prefix
     * \param[in] cp_length: Cyclic prefix size
     * \param[in] scs: Subcarrier spacing
     * \param[out] output: Frequency offset in Hertz
     * \param[in] num_symbols: Number of symbols in the signal
     */

    complex<float> out;
    float phase_offset = 0;

    /*
     * Averaging phase offset between cyclic prefix and corresponding symbol data
     */
    for (int symbol = 0; symbol < num_symbols; symbol ++){
        out = 0;
        for (int i = 0; i < cp_length; i ++){
            out += conj(input_signal[i + symbol * symbol_duration]) * input_signal[i + symbol * symbol_duration + fft_size];
        }
        phase_offset += arg(out);
    }

    phase_offset/= num_symbols;
    /*
     * Computing frequency offset corresponding to the computed phase offset
     */
    output = scs * phase_offset/(2 * M_PI);

}

void free5GRAN::phy::signal_processing::transpose_signal(vector<complex<float>> *input_signal, float freq_offset, int sample_rate, int input_length) {
    /**
     * \fn transpose_signal
     * \brief Shift signal in frequency domain by a frequency offset
     * \param[in] input_signal: Input signal to be shifted
     * \param[in] freq_offset: Frequency offset
     * \param[in] sample_rate: Signal sample rate
     * \param[in] input_length: Input signal length
     */
    complex<float> j(0, 1);
    for (int i = 0; i < input_length; i ++){
        (*input_signal)[i] = (*input_signal)[i] * exp(complex<float>(-2,0) * j * complex<float>(M_PI,0) * freq_offset * complex<float>((float) i,0) / complex<float>(sample_rate,0));
    }
}


void free5GRAN::phy::signal_processing::channel_demapper(vector<vector<complex<float>>> &input_signal, vector<vector<vector<int>>> &ref, complex<float> **output_channels, vector<vector<vector<int>>> &output_indexes, int num_channels, int num_symbols, int num_sc){
    /**
     * \fn channel_demapper
     * \brief Consumes an input signal and returns different channels containing corresponding data, based on ref indexes
     * \param[in] input_signal: Input signal to be de-mapped
     * \param[in] ref: Array containing the indexes of the different channels inside the OFDM grid
     * \param[in] channel_sizes: Array containing the outpu transport_channel sizes
     * \param[out] output_channels: Channels symbols
     * \param[out] output_indexes: Indexes of the output_channels elements
     * \param[in] num_channels: Number of channels
     * \param[in] num_symbols: Number of symbols in the OFDM grid
     * \param[in] num_sc: Number of subcarriers in the OFDM grid
     */
    int channel_counter[num_channels];
    for (int i = 0; i < num_channels; i ++){
        channel_counter[i] = 0;
    }
    /*
     * Loop over each resource element in the grid
     */
    for (int symbol = 0; symbol < num_symbols; symbol ++){
        for (int sc = 0; sc < num_sc; sc ++){
            /*
             * Loop over each transport_channel
             */
            for (int channel = 0; channel < num_channels; channel++){
                /*
                 * If the current resource element belongs to the transport_channel, store the element and corresponding grid index
                 */
                if (ref[channel][symbol][sc] == 1){
                    output_channels[channel][channel_counter[channel]] = input_signal[symbol][sc];
                    output_indexes[channel][0][channel_counter[channel]] = symbol;
                    output_indexes[channel][1][channel_counter[channel]] = sc;
                    channel_counter[channel] ++;
                }
            }
        }
    }
}

double free5GRAN::phy::signal_processing::compute_freq_from_gscn(int gscn){
    /**
     * \fn compute_freq_from_gscn
     * \brief Computing frequency from received GSCN
     * \standard TS38.104 V15.2.0 Table 5.4.3.1-1
     *
     * \param[in] gscn: Input GSCN
     */
    int M,N;
    double freq;
    if (gscn < 7499){
        for (int i = 0; i < 3; i ++){
            M = 2 * i + 1;
            if ((gscn - (M - 3)/2) % 3 == 0){
                break;
            }
        }
        N = (gscn - (M - 3)/2) / 3;
        freq = N * 1.2e6 + M * 50e3;
    }else if (gscn < 22256){
        N = gscn - 7499;
        freq = 3e9 + N * 1.44e6;
    }else {
        N = gscn - 22256;
        freq = 24250.08e6 + N * 17.28e6;
    }
    return freq;
}


free5GRAN::pdcch_t0ss_monitoring_occasions  free5GRAN::phy::signal_processing::compute_pdcch_t0_ss_monitoring_occasions(int pdcch_config, int pbch_scs, int common_scs, int i){
    /**
     * \fn compute_pdcch_t0_ss_monitoring_occasions
     * \brief Compute PDCCH monitoring occasion informations
     * \standard TS38.213 V15.2.0 Section 13
     *
     * \param[in] pdcch_config: PDCCH config received from MIB
     * \param[in] pbch_scs: PBCH subcarrier spacing
     * \param[in] common_scs: BWP subcarrier spacing
     * \param[in] i: SSB index
    */
    free5GRAN::pdcch_t0ss_monitoring_occasions obj = {};
    int index1,index2;
    index1 = pdcch_config / 16;
    index2 = pdcch_config % 16;
    int *table1;
    float *table2;
    if (pbch_scs==15e3 && common_scs == 15e3){
        table1 = free5GRAN::TS_38_213_TABLE_13_1[index1];
    }else if (pbch_scs==15e3 && common_scs == 30e3){
        table1 = free5GRAN::TS_38_213_TABLE_13_2[index1];
    }else if (pbch_scs==30e3 && common_scs == 15e3){
        table1 = free5GRAN::TS_38_213_TABLE_13_3[index1];
    }else if (pbch_scs==30e3 && common_scs == 30e3){
        table1 = free5GRAN::TS_38_213_TABLE_13_4[index1];
    }
    table2 = free5GRAN::TS_38_213_TABLE_13_11[index2];

    obj.multiplexing_pattern = table1[0];
    obj.n_rb_coreset = table1[1];
    obj.n_symb_coreset = table1[2];
    obj.offset = table1[3];
    obj.O = table2[0];
    obj.num_ss_slots = table2[1];
    obj.M = table2[2];

    if (table2[3] == -1){
        if (i % 2 == 0){
            obj.first_symb_index = 0;
        }else {
            obj.first_symb_index = obj.n_symb_coreset;
        }
    }else {
        obj.first_symb_index = table2[3];
    }

    return obj;
}


void free5GRAN::phy::signal_processing::compute_rb_start_lrb_dci(int RIV, int n_size_bwp, int &lrb, int &rb_start){
    /**
     * \fn compute_rb_start_lrb_dci
     * \brief Compute RB start and LRB from DCI
     * \standard TS38.214 V15.2.0 Section 5.1.2.2.2
     *
     * \param[in] RIV: Frequency domain resource allocation
     * \param[in] n_size_bwp: Active BWP size (in RB)
     * \param[out] lrb: Number of PDSCH RB
     * \param[out] rb_start: First PDSCH RB index in BWP
    */
    lrb = floor(RIV/n_size_bwp) + 1;
    rb_start = RIV - ((lrb - 1) * n_size_bwp);
    if (lrb > n_size_bwp - rb_start){
        lrb = n_size_bwp - lrb + 2;
        rb_start = n_size_bwp - 1 - rb_start;
    }
}

void free5GRAN::phy::signal_processing::get_pdsch_dmrs_symbols(string type, int duration, int additionnal_position, int l0, int **output, int &size){
    /**
     * \fn get_pdsch_dmrs_symbols
     * \brief Retrieve DMRS symbols
     * \standard TS38.211 V15.2.0 Table 7.4.1.1.2-3
     *
     * \param[in] additionnal_position: Number of additional positions
     * \param[in] l0: First DMRS symbol
     * \param[out] output: DMRS symbols array
     * \param[out] size: Number of DMRS symbols
    */
    if (duration < 8 || additionnal_position==0){
        size = 1;
        (*output) = new int[size]{l0};
    }else if (duration < 10){
        size = 2;
        (*output) = new int[size]{l0,7};
    }else if (additionnal_position==1){
        size = 2;
        if (duration < 13){
            (*output) = new int[size]{l0,9};
        }else {
            (*output) = new int[size]{l0,11};
        }
    }
    else if (additionnal_position==2){
        size = 3;
        if (duration < 13){
            (*output) = new int[size]{l0,6,9};
        }else {
            (*output) = new int[size]{l0,7,11};
        }
    }else if (additionnal_position==3){
        if (duration < 12){
            size = 3;
            (*output) = new int[size]{l0,6,9};
        }else {
            size = 4;
            (*output) = new int[size]{l0,5,8,11};
        }
    }
}



void free5GRAN::phy::signal_processing::compute_cp_lengths(int scs, int nfft, int is_extended_cp, int num_symb_per_subframes, int *cp_lengths, int *cum_sum_cp_lengths){
    /**
     * \fn compute_cp_lengths
     * \brief Compute cyclic prefix size
     * \standard TS38.211 V15.2.0 Section 5.3
     *
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
}

void free5GRAN::phy::signal_processing::compute_phase_decomp(int* cp_lengths, int* cum_sum_symb, float sampling_rate, float f0,  int num_symb_per_subframes, complex<float>* phase_decomp_factor){
    /**
     * \fn compute_phase_decomp
     * \brief Compute phase decompensation vector from frequency
     * \standard TS38.211 V15.2.0 Section 5.3
     *
     * \param[in] cp_lengths: CP lengths
     * \param[in] cum_sum_symb: CP lengths cumulative sum
     * \param[in] sampling_rate: RF device sample rate
     * \param[in] f0: Frequency in Hz
     * \param[in] num_symb_per_subframes: Number of symbols per subframes
     * \param[out] phase_decomp_factor: Phase de-compensator vector
    */
    float t_start, t_cp;
    for (int i = 0; i < num_symb_per_subframes; i ++){
        t_start = cum_sum_symb[i]/ sampling_rate;
        t_cp = cp_lengths[i] / sampling_rate;
        phase_decomp_factor[i] = exp(complex<float>(0,- 2 * f0 * M_PI) * (- t_start - t_cp));
    }

}

int free5GRAN::phy::signal_processing::compute_nre(int num_symb_pdsch, int num_dmrs_symb){
    /**
     * \fn compute_nre
     * \brief Compute number of PDSCH RE per RB
     * \param[in] num_symb_pdsch: Number of PDSCH symbols
     * \param[in] num_dmrs_symb: Number of DMRS symbols
     * \return number of PDSCH RE per RB
    */
    return 12 * (num_symb_pdsch - num_dmrs_symb);
}

void free5GRAN::phy::signal_processing::fft(vector<complex<float>> time_domain_signal, vector<vector<complex<float>>> &output_signal, int fft_size, int *cp_lengths, int *cum_sum_symb, int num_symbols, int num_sc_output, int first_symb_index, int offset){
    /**
 * \fn fft
 * \brief Perform FFT on time domain signal to recover frequency domain signal (= RE grid)
 * \param[in] time_domain_signal: Input time domain signal
 * \param[out] output_signal: Output RE grid
 * \param[in] fft_size: FFT size (Number of samples per symbol, excluding CP)
 * \param[in] cp_lengths: Array of CP lengths for 1 subframe (= 1ms)
 * \param[in] cum_sum_symb: Cumulative sum of symbols length in one subframe
 * \param[in] num_symbols: Number of symbols in output RE grid (= Number of rows of output_signal)
 * \param[in] num_sc_output: Number of subcarriers in output RE grid (= Number of columns of output_signal).
 * \param[in] first_symb_index: Index of first symbol to be extracted in frame.
 * \param[in] offset: Number of samples to be left before extracting. Can be used while extracting specific slots in a radio frame.
*/
    // Initializing fft parameters
    fftw_complex *fft_in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * fft_size);
    fftw_complex *fft_out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * fft_size);

    fftw_plan fft_plan = fftw_plan_dft_1d(fft_size, fft_in, fft_out, FFTW_FORWARD, FFTW_MEASURE);

    int symb_index;

    for (int symbol = 0; symbol < num_symbols; symbol ++){
        symb_index = (first_symb_index + symbol) % free5GRAN::NUMBER_SYMBOLS_PER_SLOT_NORMAL_CP;
        // Filling fft input signal with current symbol IQ
        for (int i = 0; i < fft_size; i++){
            fft_in[i][0] = real(time_domain_signal[i + offset + cum_sum_symb[symb_index] + cp_lengths[symb_index]]);
            fft_in[i][1] = imag(time_domain_signal[i + offset + cum_sum_symb[symb_index] + cp_lengths[symb_index]]);
        }
        // Execute the fft
        fftw_execute(fft_plan);
        // Building the RE grid
        for (int i = 0; i < num_sc_output / 2; i++){
            output_signal[symbol][num_sc_output / 2 + i ] = complex<float>(fft_out[i][0],fft_out[i][1]);
            output_signal[symbol][num_sc_output / 2 - i - 1] = complex<float>(fft_out[fft_size - i - 1][0],fft_out[fft_size - i - 1][1]);
        }
    }
}

void free5GRAN::phy::signal_processing::get_candidates_frames_indexes(vector<vector<int>> &frame_indexes, int *frame_numbers, int sfn, int index_first_pss, int num_samples_before_pss, int frame_size){
    if (num_samples_before_pss < index_first_pss) {
        if (num_samples_before_pss + 2 * frame_size < index_first_pss){
            frame_indexes[0][0] = index_first_pss - num_samples_before_pss - 2 * frame_size;
            frame_indexes[0][1] = index_first_pss - num_samples_before_pss - frame_size - 1;
            frame_indexes[1][0] = index_first_pss - num_samples_before_pss - frame_size;
            frame_indexes[1][1] = index_first_pss - num_samples_before_pss - 1;
            frame_numbers[0] = sfn - 2;
            frame_numbers[1] = sfn - 1;
        }
        else if (num_samples_before_pss + frame_size < index_first_pss){
            frame_indexes[0][0] = index_first_pss - num_samples_before_pss - frame_size;
            frame_indexes[0][1] = index_first_pss - num_samples_before_pss - 1;
            frame_indexes[1][0] = index_first_pss - num_samples_before_pss;
            frame_indexes[1][1] = index_first_pss - num_samples_before_pss + frame_size - 1;
            frame_numbers[0] = sfn - 1;
            frame_numbers[1] = sfn;
        }
        else {
            frame_indexes[0][0] = index_first_pss - num_samples_before_pss;
            frame_indexes[0][1] = index_first_pss - num_samples_before_pss + frame_size - 1;
            frame_indexes[1][0] = index_first_pss - num_samples_before_pss + frame_size;
            frame_indexes[1][1] = index_first_pss - num_samples_before_pss + 2 * frame_size - 1;
            frame_numbers[0] = sfn;
            frame_numbers[1] = sfn + 1;
        }
    }else {
        frame_indexes[0][0] = index_first_pss - num_samples_before_pss + frame_size;
        frame_indexes[0][1] = index_first_pss - num_samples_before_pss + 2 * frame_size - 1;
        frame_indexes[1][0] = index_first_pss - num_samples_before_pss + 2 * frame_size;
        frame_indexes[1][1] = index_first_pss - num_samples_before_pss + 3 * frame_size - 1;
        frame_numbers[0] = sfn + 1;
        frame_numbers[1] = sfn + 2;
    }
}















/** FROM HERE, IT'S ADDITION FROM BENOIT. BE CAREFUL WHEN MERGING */

void free5GRAN::phy::signal_processing::modulation(vector<int> bits, int bit_sequence_length, int modulation_scheme,
                                                   vector<complex<float>> &pbch_symbols_vector) {
    /**
    * \fn modulation(vector<int> bits, int bit_sequence_length, int modulation_scheme, vector<complex<float>> &pbch_symbols_vector)
    * \brief converts a bits sequence into IQ symbols, using BPSK (Binary Phase Shit Keying) or QPSK (Quadrature Phase Shift Keying)
    * \details
        * For the BPSK (if modulation_scheme == 0): for each bit, the corresponding symbol will be 1 or -1
        * For the QPSK (if modulation_scheme == 1): for each pair of bits, the corresponding symbols will be +/- 1/sqrt(2) +/- i 1/sqrt(2)
    * \standard TS38.211 V15.2.0 Section 5.1
    * \param[in] bits The input sequence of bits.
    * \param[in] bit_sequence_length number of bits.
    * \param[in] modulation_scheme. 0 if BPSK, 1 if QPSK.
    * \param[out] pbch_symbols_vector the output sequence of IQ symbols.
    */


    /**  BPSK modulation (modulation_scheme == 0)
    * For BPSK modulation pattern, see the TS38.211 V15.2.0 Section 5.1.2 */
    if (modulation_scheme == 0) {
        for (int i = 0; i < bit_sequence_length ; i++) {
            pbch_symbols_vector[i] = {(1/(float) sqrt(2)) * (1 - 2*bits[2*i]), (1/(float) sqrt(2) * (1 - 2*bits[2*i]))};
        }
    }

    /**  QPSK modulation (modulation_scheme == 1)
    * For QPSK modulation pattern, see the TS38.211 V15.2.0 Section 5.1.3 */
    if(modulation_scheme == 1){
        for (int i=0; i < (bit_sequence_length/2); i++) {
            pbch_symbols_vector[i] = {(1/(float) sqrt(2)) * (1 - 2*bits[2*i]), (1/(float) sqrt(2)) * (1 - 2*bits[2*i + 1]) };
        }
    }
}


void free5GRAN::phy::signal_processing::build_reference_grid(int num_channels, int num_sc_ssb, int num_symbols_ssb, int pci,
                                                             vector<vector<vector<int>>> &ref) {
    /**
    * \fn build_reference_grid (int num_channels, int num_sc_ssb, int num_symbols_ssb, int pci, vector<vector<vector<int>>> &ref)
    * \brief Constructs a reference grid (called ref) to build the SSB signal. Same principle as "Mask" in network
    * \details
     * In our case, there are 4 channels to build on the SSB signal:  PSS (127 symbols), SSS (127 symbols), PBCH (432 symbols) and DMRS (144 symbols).
     * In our case, the SSB contains 4 symbols of each 240 subcarriers (sc).
     * Goal here is to construct a reference grid that will allow, at next step, to fill SSB with the 4 channels.
     * If the value of ref[channel][symbol][sc] is 1, it means that a symbol of this channel has to be placed at SSB[symbol][sc].
     *
    * \standard TS38.211 V15.2.0 Section 7.4.3
    * \param[in] num_channels. Number of channels as input of this function. In our case, this value is 4.
    * \param[in] num_sc_ssb. Number of sub-carrier (sc) in the SSB. In our case, this value is 240.
    * \param[in] num_symbols_ssb. Number of symbols in the SSB. In our case, this value is 4.
     *\param[in] pci. Physical Cell Id. Used to calculate DMRS index.
    * \param[out] ref. A 3 dimensions vector of int (0 or 1). ref[channel][symbol][subcarrier]. Also called 'Mask'
     *             In our case,     ref[0] -> index of PSS channel
     *                              ref[1] -> index of SSS channel
     *                              ref[2] -> index of PBCH channel
     *                              ref[3] -> index of DMRS channel
     *
     */


    /**Loop over the symbols */
    for (int symbol = 0; symbol < num_symbols_ssb; symbol++) {
        /** Loop over the subcarriers */
        for (int sc = 0; sc < num_sc_ssb; sc++) {
            /** Filling the ref table for each symbol and for each channel, according to TS38.211 V15.2.0 Section 7.4.3 */
            /** Symbol 0 only contains PSS */
            if (symbol == 0) {
                if (sc >= free5GRAN::INTERVAL_SSB_PSS[0] && sc <= free5GRAN::INTERVAL_SSB_PSS[1]) {
                    ref[0][symbol][sc] = 1;
                } else {
                    ref[0][symbol][sc] = 0;
                }
                ref[1][symbol][sc] = 0;
                ref[2][symbol][sc] = 0;
                ref[3][symbol][sc] = 0;
            }

            /** Symbol 1 only contains part of PBCH/DMRS */
            if (symbol == 1) {
                ref[0][symbol][sc] = 0;
                ref[1][symbol][sc] = 0;
                if (sc % 4 != pci % 4) {
                    ref[2][symbol][sc] = 1;
                    ref[3][symbol][sc] = 0;
                } else {
                    /** A DMRS symbol is placed every 4 symbols, beginning at pci%4 */
                    ref[2][symbol][sc] = 0;
                    ref[3][symbol][sc] = 1;
                }
            }

            /** Symbol 2 contains 2 parts of PBCH/DMRS and SSS */
            if (symbol == 2) {
                ref[0][symbol][sc] = 0;
                if (sc >= free5GRAN::INTERVAL_SSB_SSS[0] && sc <= free5GRAN::INTERVAL_SSB_SSS[1]) {
                    ref[1][symbol][sc] = 1;
                    ref[2][symbol][sc] = 0;
                    ref[3][symbol][sc] = 0;
                } else {
                    ref[1][symbol][sc] = 0;
                }
                if (sc < free5GRAN::INTERVAL_SSB_NO_PBCH_DMRS[0] || sc > free5GRAN::INTERVAL_SSB_NO_PBCH_DMRS[1]) {
                    if (sc % 4 != pci % 4) {
                        ref[2][symbol][sc] = 1;
                        ref[3][symbol][sc] = 0;
                    } else {
                        /** A DMRS symbol is placed every 4 symbols, beginning at pci%4 */
                        ref[2][symbol][sc] = 0;
                        ref[3][symbol][sc] = 1;
                    }
                } else {
                    ref[2][symbol][sc] = 0;
                    ref[3][symbol][sc] = 0;
                }
            }

            /** Symbol 3 only contains last part of PBCH/DMRS */
            if (symbol == 3) {
                ref[0][symbol][sc] = 0;
                ref[1][symbol][sc] = 0;
                if (sc % 4 != pci % 4) {
                    ref[2][symbol][sc] = 1;
                    ref[3][symbol][sc] = 0;
                } else {
                    /** A DMRS symbol is placed every 4 symbols, beginning at pci%4 */
                    ref[2][symbol][sc] = 0;
                    ref[3][symbol][sc] = 1;
                }
            }
        }
    }
}


void free5GRAN::phy::signal_processing::map_ssb(vector<vector<complex<float>>> input_channels, vector<vector<vector<int>>> ref,
                                                vector<vector<complex<float>>> &output_channels, int num_channels,
                                                int num_symbols, int num_sc) {

    /**
   * \fn map_ssb (vector<vector<complex<float>>> input_channels, vector<vector<vector<int>>> ref, vector<vector<complex<float>>> &output_channels, int num_channels, int num_symbols, int num_sc)
   * \brief Build a SSB signal with the 4 channels (PSS, SSS, PBCH and DMRS) using the ref vector constructed before.
   * \standard TS38.211 V15.2.0 Section 7.4.3
   *
   * \param[in] input_channels. 2 dimensions vector of complexes containing our input channels (in our case: PSS, SSS, PBCH and DMRS).
   * \param[in] ref. 3 dimensions vector of int (1 or 0) that indicates the indexes of our channels.
   * \param[in] num_channels. Number of input channels. In our case, it is 4 (PSS, SSS, PBCH and DMRS).
   * \param[in] num_symbols. Number of symbols on the output_channel (SSB). In our case, it is 4.
   * \param[in] num_sc. Number of subcarrier (sc) on the output_channel (SSB). In our case, it is 240.
   * \param[out] &output_channels. 2 dimensions vector of complexes. In our case, it is the SSB signal, composed of 4 symbols of each 240 subcarriers.
   */


    /** Initialize channel_counter for each channel. channel_counter[channel] will vary from 0 to channel size */
    vector<int> channel_counter(num_channels, 0);

    /** Loop over all channels (4 in our case) */
    for (int channel = 0; channel < num_channels; channel++) {
        /** Loop over all symbols (4 in our case) */
        for (int symbol = 0; symbol < num_symbols; symbol++) {
            /** Loop over all subcarrier (240 in our case) */
            for (int sc = 0; sc < num_sc; sc++) {
                /** If ref[channel][symbol][sc] = 1, it means that a sample of this channel has to be placed at SSB[symbol][sc] */
                if (ref[channel][symbol][sc] == 1) {
                    output_channels[symbol][sc] = input_channels[channel][channel_counter[channel]];
                    channel_counter[channel]++;
                }
            }
        }
    }
}



void free5GRAN::phy::signal_processing::channel_mapper(vector<vector<complex<float>>> input_channel,
                                                       vector<vector<complex<float>>> &output_channel, int num_symbols_ssb, int index_symbol_ssb, int num_SSB_in_this_frame,
                                                       int num_sc_input, int ifft_size) {
    /**
    * \fn channel_mapper(vector<vector<complex<float>>> input_channel, vector<vector<complex<float>>> &output_channel, int num_symbols_ssb, int index_symbol_ssb, int num_SSB_in_this_frame, int num_sc_input, int ifft_size)
    * \brief Build a 10 ms time/frequency resource grid, of size num_symbols_frame (eg 140) * IFFT_SIZE (eg 512) and place the SSB in this resource grid at the right place.
    * \details
         * -step 1: From input_channel to SSB_signal_extended :increase size of the 4 SSB symbols from 240 to ifft_size, padding with '0' values.
         * -step 2: From SSB_signal_extended to output_channel: place SSB at the right place in the 10 ms resource grid.
    * \standard !! TS TO BE ADDED !!
    * \param[in] input_channel. 2 dimensions vector of complexes. In our case, it is the SSB signal (4*240 symbols)
    * \param[out] &output_channel. 2 dimensions vector of complex. It's the resource grid generated including the SSB
    * \param[in] num_symbols_ssb. Number of symbols in the SSB. In our case, it is 4.
    * \param[in] index_symbol_ssb. Position of the SSB in the 10 ms resource grid. Is calculated in function of i_b_ssb
    * \param[in] num_SSB_in_this_frame. Number of SSB that the resource grid will contain. Is calculated in function of ssb_period and sfn. Should be equal to 0, 1 or 2.
    * \param[in] num_sc_input. Number of subcarrier in SSB. In our case, it's 240
    * \param[in] ifft_size. Frequency size of the output resource grid
    */


    /** Step 1: INCREASE SIZE SSB SYMBOLS */
    /** Loop over all symbols */
    for (int symbol = 0; symbol < num_symbols_ssb; symbol++){
        int sc_in_counter = 0;
        /** Loop over all subcarriers of output signal */
        for (int sc_out = 0; sc_out < ifft_size; sc_out++){
            if (sc_out < ((ifft_size - num_sc_input) / 2) || sc_out > ifft_size - ((ifft_size - num_sc_input) / 2)){
                free5GRAN::SSB_signal_extended[symbol][sc_out] = {0,0};
            }else{
                free5GRAN::SSB_signal_extended[symbol][sc_out] = input_channel[symbol][sc_in_counter];
                sc_in_counter++;
            }
        }
    }

    /** Step 2: PLACE SSB IN A FRAME */
    /** Place the 4 symbols of SSB at the right place in output_channel */
    int i = 0;
    for (int symbol_ssb = index_symbol_ssb; symbol_ssb < index_symbol_ssb + NUM_SYMBOLS_SSB; symbol_ssb++) {
        output_channel[symbol_ssb] = free5GRAN::SSB_signal_extended[i];
        i++;
    }

    /** If num_SSB_in_this_frame = 2 (i.e. if ssb_period == 0.005), place a second SSB in output_channel, 5 ms after the first one */
    if (num_SSB_in_this_frame == 2){
        int j = 0;
        int index_symbol_second_ssb = index_symbol_ssb + (num_symbols_frame/2);
        for (int symbol_ssb = index_symbol_second_ssb; symbol_ssb < index_symbol_second_ssb + NUM_SYMBOLS_SSB; symbol_ssb++) {
            output_channel[symbol_ssb] = free5GRAN::SSB_signal_extended[j];
            j++;
        }
    }
}



void free5GRAN::phy::signal_processing::generate_freq_domain_frame(vector<complex<float>> pbch_symbols_vector, int pci, int index_symbol_ssb, int num_SSB_in_this_frame,
                                                                   int i_b_ssb, vector<vector<complex<float>>> &freq_domain_frame) {

    /**
    * \fn generate_freq_domain_frame(vector<complex<float>> pbch_symbols_vector, int pci, int index_symbol_ssb, int num_SSB_in_this_frame, int i_b_ssb, vector<vector<complex<float>>> &freq_domain_frame)
    * \brief Generates from a pbch sequence a frequency domain frame (also called: resource grid) that includes the SSB (Synchronisation Signal Block).
    * \details
            * step 1: generate DMRS symbols, PSS symbols and SSS symbols
            * step 2: map_SSB: assemble pbch, dmrs, pss and sss to obtain a SSB
            * step 3: channel_mapper: construct time/domain reference grid and place the SSB in it.
    * \standard !! Standard TO BE ADDED
    * \param[in] pbch_symbols_vector. In our case, it is a 432 symbols sequence.
    * \param[in] pci. Physical Cell ID.
    * \param[in] index_symbol_ssb : Position of the SSB in the resource grid. Is calculated in function of i_b_ssb
    * \param[in] num_SSB_in_this_frame. Number of SSB that the resource grid will contain. Is calculated in function of ssb_period and sfn. Should be equal to 0, 1 or 2.
    * \param[in] i_b_ssb. SSB index. Should be between 0 and 7.
    * \param[out] &freq_domain_frame. 2 dimensions vector, containing a time/frequency resource grid with SSB placed in it.
    */

    /** Step 1: generate DMRS symbols, PSS symbols and SSS symbols */
    /** DMRS -> Generate dmrs_symbols (144 symbols long in our case) from pci and i_b_ssb. TS38.211 V15.2.0 Section 7.4.1.4.1 */
    std::complex<float> dmrs_symbols[free5GRAN::SIZE_SSB_DMRS_SYMBOLS];
    free5GRAN::utils::sequence_generator::generate_pbch_dmrs_sequence(pci, i_b_ssb, dmrs_symbols);

    /** Convert PCI into n_id_1 and n_id_2 */
    int n_id_1, n_id_2;
    n_id_2 = pci % 3;
    n_id_1 = (pci - n_id_2) / 3;

    /** Compute pss_sequence_symbols (127 symbols long in our case) from n_id_2. TS38.211 V15.2.0 Section 7.4.2.2.1 */
    int pss_sequence_symbols[free5GRAN::SIZE_PSS_SSS_SIGNAL];
    free5GRAN::utils::sequence_generator::generate_pss_sequence(n_id_2, pss_sequence_symbols);

    /** Convert PSS sequence element from table<int> to vector complex<float> (Imaginary part = 0) */
    vector<complex<float>> pss_vector(free5GRAN::SIZE_PSS_SSS_SIGNAL, {0.0, 0.0});
    for (int i = 0; i < free5GRAN::SIZE_PSS_SSS_SIGNAL; i++) {
        pss_vector[i] = {static_cast<float>(pss_sequence_symbols[i]), 0};
    }


    /** Compute sss_sequence_symbols (127 symbols long in our case) from n_id_1 and n_id_2. TS38.211 V15.2.0 Section 7.4.2.3.1 */
    int sss_sequence_symbols[free5GRAN::SIZE_PSS_SSS_SIGNAL];
    free5GRAN::utils::sequence_generator::generate_sss_sequence(n_id_1, n_id_2, sss_sequence_symbols);

    /** Convert SSS sequence element from table<int> to vector complex<float> (Imaginary part = 0) */
    vector<complex<float>> sss_vector(free5GRAN::SIZE_PSS_SSS_SIGNAL, {0.0, 0.0});
    for (int i = 0; i < free5GRAN::SIZE_PSS_SSS_SIGNAL; i++) {
        sss_vector[i] = {static_cast<float>(sss_sequence_symbols[i]), 0};
    }


    /** Step 2: map_SSB: assemble pbch, dmrs, pss and sss to obtain a SSB */
    /** Build reference grid ref to then fill the SSB correctly, according to TS38.211 V15.2.0 Section 7.4.3 */
    vector<vector<vector<int>>> ref(4, vector<vector<int>>(free5GRAN::NUM_SYMBOLS_SSB, vector<int>(free5GRAN::NUM_SC_SSB)));

    free5GRAN::phy::signal_processing::build_reference_grid(4, free5GRAN::NUM_SC_SSB, free5GRAN::NUM_SYMBOLS_SSB, pci, ref);

    /** Convert DMRS from table to vector */
    vector<complex<float>> dmrs_vector(free5GRAN::SIZE_SSB_DMRS_SYMBOLS, {0.0, 0.0});
    for (int sample = 0; sample < free5GRAN::SIZE_SSB_DMRS_SYMBOLS; sample ++) {
        dmrs_vector[sample] = dmrs_symbols[sample];
    }

    /** Regroup the 4 channels (PSS, SSS, PBCH and DMRS) into a vector */
    vector<vector<complex<float>>> input_channels{pss_vector, sss_vector, pbch_symbols_vector, dmrs_vector};


    /** Initialize ssb_signal_freq_domain (In our case, 4 * 240 symbols) */
    vector<vector<complex<float>>> ssb_signal_freq_domain(free5GRAN::NUM_SYMBOLS_SSB,
                                                          vector<complex<float>>(NUM_SC_SSB));

    free5GRAN::phy::signal_processing::map_ssb(input_channels, ref,
            ssb_signal_freq_domain, 4, free5GRAN::NUM_SYMBOLS_SSB, free5GRAN::NUM_SC_SSB);


    /** step 3: channel_mapper: construct time/domain reference grid and place the SSB in it */
    free5GRAN::phy::signal_processing::channel_mapper(ssb_signal_freq_domain, freq_domain_frame,
                                                      free5GRAN::NUM_SYMBOLS_SSB, index_symbol_ssb,
                                                      num_SSB_in_this_frame, free5GRAN::NUM_SC_SSB,
                                                      free5GRAN::SIZE_IFFT_SSB);
}



void free5GRAN::phy::signal_processing::ifft(vector<vector<complex<float>>> freq_domain_frame, int *cp_lengths_one_frame, vector<int> data_symbols, int num_symbols_frame, float scaling_factor, vector<complex<float>> &one_frame_vector) {
    /**
      * \fn ifft(vector<vector<complex<float>>> freq_domain_frame, int *cp_lengths_one_frame, vector<int> data_symbols, int num_symbols_frame, float scaling_factor, vector<complex<float>> &one_frame_vector)
      * \brief performs ifft (Inverse Fast Fourier Transform) for each symbols of the input resource grid.
      * \details
            * -step 1: Reverse symbols: Only for not null symbols, inverting first half with second half
            * -step 2: Scaling factor: Only for not null symbols, multiply each value by scaling_factor
            * -step 3: ifft: For each not null symbols, perform ifft
            * -step 4: Adding CP: place each results on a single vector and adding Cyclic Prefix before each symbols.
      * \standard !! TS to be added !!
      *
      * \param[in] freq_domain_frame. Input 2 dimensions vector, containing a time/frequency resource grid with SSB placed in it.
      * \param[in] *cp_lengths_one_frame. Cyclic Prefix length for each symbol of a frame
      * \param[in] data_symbols. If a symbol is null, data_symbols[symbol] = 0. Otherwise, data_symbols[symbol] = 1
      * \param[in] num_symbols_frame. Number of symbols in a 10 ms resource grid (eg 140 or 280).
      * \param[in] scaling_factor. multiplication factor applied to each values before performing ifft
      * \param[out] &one_frame_vector. One dimension vector containing, in time domain, 10 ms of signal with SSB included in it.
      */



    /** Step 1 & 2 : REVERSE SYMBOLS AND SCALING FACTOR */
    /** Loop over all symbols of resource grid */
    for (int symbol = 0; symbol < num_symbols_frame; symbol++) {
        /** Following scop is not done for the null symbols (when data_symbols[symbol] = 0) */
        if (data_symbols[symbol] == 1) {
            int sc_counter1 = free5GRAN::SIZE_IFFT_SSB / 2, sc_counter2 = 0;
            for (int sc = 0; sc < free5GRAN::SIZE_IFFT_SSB; sc++) {
                /** Fill first half of freq_domain_reversed_frame */
                if (sc < free5GRAN::SIZE_IFFT_SSB / 2) {
                    free5GRAN::freq_domain_reversed_frame[symbol][sc] =
                            freq_domain_frame[symbol][sc_counter1] * scaling_factor;
                    sc_counter1++;
                }
                /** Fill second half of freq_domain_reversed_frame */
                else {
                    free5GRAN::freq_domain_reversed_frame[symbol][sc] =
                            freq_domain_frame[symbol][sc_counter2] * scaling_factor;
                    sc_counter2++;
                }
            }
        }
    }

    /** Step 3: IFFT */
    /** Loop over all symbols of resource grid */
    for (int symbol = 0; symbol < num_symbols_frame; symbol++) {
        /** Following scop is not done for the null symbols (when data_symbols[symbol] = 0) */
        if (data_symbols[symbol] == 1) {
            /** Generate complex arrays to store ifft signals */
            fftw_complex *signal_in = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * free5GRAN::SIZE_IFFT_SSB);
            fftw_complex *signal_out = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * free5GRAN::SIZE_IFFT_SSB);

            /** Generate plans */
            fftw_plan ifft_plan = fftw_plan_dft_1d(free5GRAN::SIZE_IFFT_SSB, signal_in, signal_out, FFTW_BACKWARD,
                                                   FFTW_MEASURE);

            /** Initialize arrays */
            for (int i = 0; i < free5GRAN::SIZE_IFFT_SSB; i++) {
                signal_in[i][0] = 0;
                signal_out[i][1] = 0;
            }

            /** Fill signal_in with the freq_domain_reversed_frame signal */
            for (int sc = 0; sc < free5GRAN::SIZE_IFFT_SSB; sc++) {
                signal_in[sc][0] = free5GRAN::freq_domain_reversed_frame[symbol][sc].real();
                signal_in[sc][1] = free5GRAN::freq_domain_reversed_frame[symbol][sc].imag();
            }

            /** Execute the ifft */
            fftw_execute(ifft_plan);

            /** Fill time_domain_frame with signal_out */
            for (int sc = 0; sc < free5GRAN::SIZE_IFFT_SSB; sc++) {
                free5GRAN::time_domain_frame[symbol][sc] = {static_cast<float>(signal_out[sc][0]),
                                                            static_cast<float>(signal_out[sc][1])};
            }

            /** Clean up ifft_plan, signal_in and signal_out */
            fftw_destroy_plan(ifft_plan);
            fftw_free(signal_in); fftw_free(signal_out);
        }
    }

    /** Fill a txt file to check spectogram on Python or Matlab */
    /** for (int sc = 0; sc < free5GRAN::SIZE_IFFT_SSB; sc++) {
        file_gnodeb << free5GRAN::time_domain_frame[symbol][sc];
        file_gnodeb << "\n";
    }
     file_gnodeb.close();*/


    /** Step 4: ADDING CP & FILLING one_frame_vector */
    int sc_count_frame = 0;
    /** Loop over all symbols */
    for (int symbol = 0; symbol < num_symbols_frame; symbol++ ){
        int cp_count1 = 0;
        int cp_count2 = 0;
        /** Loop over the symbol size (ifft_size + cp_lengths[symbol]) */
        for (int sc = 0; sc < free5GRAN::SIZE_IFFT_SSB + cp_lengths_one_frame[symbol]; sc++) {
            /** Adding CP of ith symbol at one_frame_vector */
            if (sc < cp_lengths_one_frame[symbol]) {
                one_frame_vector[sc_count_frame] = free5GRAN::time_domain_frame[symbol][free5GRAN::SIZE_IFFT_SSB - cp_lengths_one_frame[symbol] + cp_count1];
                cp_count1++;
            }
            /** Adding ith symbol at one_frame_vector */
            else{
                one_frame_vector[sc_count_frame] = free5GRAN::time_domain_frame[symbol][cp_count2];
                cp_count2++;
            }
            sc_count_frame++;
        }
    }
}





void free5GRAN::phy::signal_processing::map_pdcch(vector<complex<float>> pdcch_symbols, int CORESET_size, int agg_level, int R, int pci, int slot_number, int symbol_number, vector<vector<complex<float>>> &interleaved_coreset_grid){

    vector<complex<float>> pdcch_dmrs (CORESET_size*3, {0,0});
    free5GRAN::utils::sequence_generator::generate_pdcch_dmrs_sequence(pci, slot_number, symbol_number, pdcch_dmrs, CORESET_size*3);
    free5GRAN::utils::common_utils::display_vector(pdcch_dmrs, CORESET_size*3, "pdcch_dmrs from libphy");
}

