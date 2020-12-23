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

#include "synchronization.h"
#include "../../utils/sequence_generator/sequence_generator.h"
#include "../../variables/common_variables/common_variables.h"
#include <fftw3.h>
#include <thread>
#include <fstream>
#include <complex>
#include <vector>
#include <iostream>
using namespace std;


/*
 * get_pss method: try to synchronize PSS for both short and extended cyclic prefix
 */
void free5GRAN::phy::synchronization::search_pss(int &n_id_2, int &synchronisation_index, float &peak_value, int cp_length, vector<complex<float>> &buff, int fft_size) {
    /**
     * \fn search_pss
     * \brief Search for PSS correlation peak inside a signal.
     * \details
     * Details:
     * - Generating the three possible PSS sequences (for N_ID_2 in [0,1,2])
     * - Performing iFFT to retreive Time domain PSS sequence
     * - Cross-correlation between received signal and three time domain PSS signals
     * - Return PSS sequence with highest correlation peak.
     *
     * \param[out] n_id_2: Returned N_ID_2
     * \param[out] synchronisation_index: Synchronization index corresponding to correlation peak
     * \param[out] peak_value: Peak height
     * \param[in] cp_length: Cyclic prefix length
     * \param[in] buff: Input IQ signal
     * \param[in] fft_size: FFT size and symbol size
    */


    /*
     * Generate complex arrays to store IFFT signals for the three different PSS sequence
     */
    fftw_complex *pss_in_0 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * fft_size);
    fftw_complex *pss_out_0 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * fft_size);
    fftw_complex *pss_in_1 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * fft_size);
    fftw_complex *pss_out_1 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * fft_size);
    fftw_complex *pss_in_2 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * fft_size);
    fftw_complex *pss_out_2 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * fft_size);

    /*
     * Generate IFFT plans for the three different PSS sequence
     */
    fftw_plan ifft_plan_0 = fftw_plan_dft_1d(fft_size, pss_in_0, pss_out_0, FFTW_BACKWARD, FFTW_MEASURE);
    fftw_plan ifft_plan_1 = fftw_plan_dft_1d(fft_size, pss_in_1, pss_out_1, FFTW_BACKWARD, FFTW_MEASURE);
    fftw_plan ifft_plan_2 = fftw_plan_dft_1d(fft_size, pss_in_2, pss_out_2, FFTW_BACKWARD, FFTW_MEASURE);

    /*
     * Initialize arrays
     */
    for (int i = 0; i < fft_size; i++){
        pss_in_0[i][0] = 0;
        pss_in_0[i][1] = 0;

        pss_in_1[i][0] = 0;
        pss_in_1[i][1] = 0;

        pss_in_2[i][0] = 0;
        pss_in_2[i][1] = 0;
    }


    /*
     * Generate and get PSS sequences
     */
    int *pss_0_seq, *pss_1_seq, *pss_2_seq;
    pss_0_seq = new int[free5GRAN::SIZE_PSS_SSS_SIGNAL];
    pss_1_seq = new int[free5GRAN::SIZE_PSS_SSS_SIGNAL];
    pss_2_seq = new int[free5GRAN::SIZE_PSS_SSS_SIGNAL];

    free5GRAN::utils::sequence_generator::generate_pss_sequence(0, pss_0_seq);
    free5GRAN::utils::sequence_generator::generate_pss_sequence(1, pss_1_seq);
    free5GRAN::utils::sequence_generator::generate_pss_sequence(2, pss_2_seq);


    /*
     * Generate frequency domain signal (PSS is BPSK modulated, real part is the pss sequence value and imaginary part is 0)
     */
    for (int i = 0; i < fft_size/2; i++){
        if (i < 63){
            pss_in_0[i][0] = pss_0_seq[i + 64];
            pss_in_0[i][1] = 0;

            pss_in_1[i][0] = pss_1_seq[i + 64];
            pss_in_1[i][1] = 0;

            pss_in_2[i][0] = pss_2_seq[i + 64];
            pss_in_2[i][1] = 0;
        }
        if (i < 64){
            pss_in_0[fft_size - i - 1][0] = pss_0_seq[64 - i - 1];
            pss_in_0[fft_size - i - 1][1] = 0;

            pss_in_1[fft_size - i - 1][0] = pss_1_seq[64 - i - 1];
            pss_in_1[fft_size - i - 1][1] = 0;

            pss_in_2[fft_size - i - 1][0] = pss_2_seq[64 - i - 1];
            pss_in_2[fft_size - i - 1][1] = 0;
        }
    }

    /*
     * Execute the IFFT
     */
    fftw_execute(ifft_plan_0);
    fftw_execute(ifft_plan_1);
    fftw_execute(ifft_plan_2);

    vector<complex<float>> time_signal_pss_0(cp_length + fft_size),time_signal_pss_1(cp_length + fft_size),time_signal_pss_2(cp_length + fft_size);

    /*
     * Transform fftw complex signals into vectors of complex values and add cyclic prefix
     */
    for (int i = 0; i < cp_length + fft_size; i++){
        if (i < cp_length){
            time_signal_pss_0[i] = complex<float>(pss_out_0[fft_size - cp_length + i][0], pss_out_0[fft_size - cp_length + i][1]);

            time_signal_pss_1[i] = complex<float>(pss_out_1[fft_size - cp_length + i][0], pss_out_1[fft_size - cp_length + i][1]);

            time_signal_pss_2[i] = complex<float>(pss_out_2[fft_size - cp_length + i][0], pss_out_2[fft_size - cp_length + i][1]);
        }
        else {
            time_signal_pss_0[i] = complex<float>(pss_out_0[i - cp_length][0], pss_out_0[i - cp_length][1]);

            time_signal_pss_1[i] = complex<float>(pss_out_1[i - cp_length][0], pss_out_1[i - cp_length][1]);

            time_signal_pss_2[i] = complex<float>(pss_out_2[i - cp_length][0], pss_out_2[i - cp_length][1]);
        }

    }


    size_t num_samples = buff.size();

    complex<float> corr_0[num_samples + fft_size + cp_length - 1], corr_1[num_samples + fft_size + cp_length - 1], corr_2[num_samples + fft_size + cp_length - 1];

    /*
     * Correlate different PSS signals with different CP length with signal obtained from PHY layer
     */

    thread correlation_thread_0(cross_correlation, buff,time_signal_pss_0,&corr_0[0],num_samples,fft_size + cp_length);
    thread correlation_thread_1(cross_correlation, buff,time_signal_pss_1,&corr_1[0],num_samples,fft_size + cp_length);
    thread correlation_thread_2(cross_correlation, buff,time_signal_pss_2,&corr_2[0],num_samples,fft_size + cp_length);
    correlation_thread_0.join();
    correlation_thread_1.join();
    correlation_thread_2.join();

    float max_value = -1;
    n_id_2 = -1;
    synchronisation_index = -1;

    /*
     * Search for the max value and index
     */
    for (int i = 0; i < num_samples + fft_size + cp_length - 1; i++){
        if (i >= fft_size + cp_length- 2 ){
            float abs_0_short = abs(corr_0[i]);
            float abs_1_short = abs(corr_1[i]);
            float abs_2_short = abs(corr_2[i]);

            if (abs_0_short > max_value){
                max_value = abs_0_short;
                n_id_2 = 0;
                synchronisation_index = i;
            } else if (abs_1_short  > max_value){
                max_value = abs_1_short ;
                n_id_2 = 1;
                synchronisation_index = i;
            } else if (abs_2_short > max_value){
                max_value = abs_2_short;
                n_id_2 = 2;
                synchronisation_index = i;
            }
        }
    }
    peak_value = max_value;

}

/*
 * cross_correlation method: cross-correlate two signals in1 and in2 which size are size1 and size2 and put the result in out
 */
void free5GRAN::phy::synchronization::cross_correlation(vector<complex<float>> in1, vector<complex<float>> in2, complex<float> *out, int size1, int size2) {
    /**
     * \fn cross_correlation
     * \brief Perform cross correlation (i.e. moving window correlation) between signal 1 and signal 2
     * \param[in] in1: Signal 1
     * \param[in] in2: Signal 2
     * \param[out] out: Correlation result
     * \param[in] size1: Signal 1 size
     * \param[in] size2: Signal 2 size
    */
    int common = 0;
    int base_id1, base_id2;
    for (int m = 0; m < size1 + size2 - 1; m++){
        if (m < size2){
            common++;

            base_id1 = 0;
            base_id2 = size2 - common;

        }else if (m > size1 - 1) {
            common--;

            base_id1 = size1 - common;
            base_id2 = 0;
        }else {
            base_id1 = m + 1 - size2;
            base_id2 = 0;
        }
        out[m] = 0;

        for (int n = 0; n < common; n++){
            out[m] += in1[base_id1 + n] * conj(in2[base_id2 + n]);
        }
    }

}

void free5GRAN::phy::synchronization::get_sss(int &n_id_1, float &peak_value, vector<complex<float>> &buff, int fft_size, int n_id_2) {
    /**
     * \fn get_sss
     * \brief Search for SSS sequence value.
     * \details
     * Details:
     * - Retrieve frequency domain input signal
     * - Generate 336 possible freq domain SSS signals for N_ID_1 in [0,335]
     * - Correlate every signals to input freq domain signal
     * - Return N_ID_1 with highest correlation
     *
     * \param[out] n_id_1: N_ID_1 with highest correlation peak
     * \param[out] peak_value: Peak value
     * \param[in] buff: Input IQ signal
     * \param[in] fft_size: FFT size
     * \param[in] n_id_2: N_ID_2 value computed by search_pss function
    */
    /*
     * Create fftw complex arrays
     */
    fftw_complex *fft_in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * fft_size);
    fftw_complex *fft_out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * fft_size);

    /*
     * Create a FFT plan
     */
    fftw_plan fft_plan = fftw_plan_dft_1d(fft_size, fft_in, fft_out, FFTW_FORWARD, FFTW_MEASURE);

    /*
     * Put the buff into the fft_in array
     */
    for (int i = 0; i < fft_size; i++){
        fft_in[i][0] = real(buff[i]);
        fft_in[i][1] = imag(buff[i]);
    }

    /*
     * Execute the FFT
     */
    fftw_execute(fft_plan);

    vector<complex<float>> frequency_sss(free5GRAN::SIZE_PSS_SSS_SIGNAL);

    /*
     * Build frequency domain signal (frequency_sss) from FFT out signal
     */
    for (int i = 0; i < 64; i++){
        if (i < 63){
            frequency_sss[i + 64] = complex<float>(fft_out[i][0], fft_out[i][1]);
        }
        frequency_sss[64 - i - 1] = complex<float>(fft_out[fft_size - i - 1][0], fft_out[fft_size - i - 1][1]);
    }
    float max_value = -1;
    n_id_1 = -1;
    complex<float> correlation_value;
    float abs_value;

    /*
     * Trying to correlate frequency domain signal to SSS sequences to find n_id_2
     * TODO: use mutlithreading instead of basic loop
     */
    int *sss_seq;
    for (int i = 0; i < free5GRAN::MAX_N_ID_1; i ++){
        sss_seq = new int[free5GRAN::SIZE_PSS_SSS_SIGNAL];
        free5GRAN::utils::sequence_generator::generate_sss_sequence(i, n_id_2, sss_seq);
        correlation_value = correlate(frequency_sss, sss_seq, free5GRAN::SIZE_PSS_SSS_SIGNAL);
        abs_value = abs(correlation_value);
        if (abs_value > max_value){
            max_value = abs_value;
            n_id_1 = i;
        }
    }
    peak_value = max_value;
}

complex<float> free5GRAN::phy::synchronization::correlate(vector<complex<float>> in1, int *in2, int size) {
    /**
     * \fn correlate
     * \brief Simple correlation between two signals
     * \param[in] in1: Signal 1
     * \param[in] in2: Signal 2
     * \param[in] size: Signals size
     *
     * \return Complex correlation value
    */
    complex<float> out = 0;
    for (int i = 0; i < size; i ++){
        out += in1[i] * conj(complex<float>(in2[i],0));
    }
    return out;
}
