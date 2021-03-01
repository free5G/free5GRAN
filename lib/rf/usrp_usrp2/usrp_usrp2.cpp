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

#include "usrp_usrp2.h"
#include <uhd.h>
#include <uhd/utils/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <complex>
#include <vector>
#include <chrono>


free5GRAN::usrp_usrp2::usrp_usrp2(double sample_rate, double center_frequency, double gain, double bandwidth, free5GRAN::rf_device chosen_device) : rf() {
    /**
     * \fn usrp_usrp2
     * \param[in] sample_rate: RF sampling rate
     * \param[in] center_frequency: Frequency center
     * \param[in] gain: Reception gain
     * \param[in] bandwidth: Received bandwidth
     * \param[in] chosen_device: RF device object
    */
    uhd::set_thread_priority_safe();

    string device_args("serial="+chosen_device.serial);
    string subdev;
    subdev = (chosen_device.subdev.empty()) ? "A:0" : chosen_device.subdev;
    string ant("TX/RX");
    string ref;
    ref = (chosen_device.ref.empty()) ? "internal" : chosen_device.ref;

    adjust_sampling_rate(sample_rate);

    this->sample_rate = sample_rate;
    this->center_frequency = center_frequency;
    this->gain = gain;
    this->bandwidth = bandwidth;

    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(device_args);
    this->usrp = usrp;
    this->usrp->set_clock_source(ref);
    this->usrp->set_rx_subdev_spec(subdev);
    this->usrp->set_rx_rate(this -> internal_sampling_rate);
    uhd::tune_request_t tune_request(this->center_frequency);
    this->usrp->set_rx_freq(tune_request);
    this->usrp->set_rx_gain(this->gain);
    this->usrp->set_rx_bandwidth(this -> internal_sampling_rate);
    this->usrp->set_rx_antenna(ant);

}

double free5GRAN::usrp_usrp2::getSampleRate() {
    /**
     * \fn getSampleRate
     * \return RF device sampling rate, in Hz
    */
    return this -> sample_rate;
}

/*
 * Get samples from USRP device
 */
void free5GRAN::usrp_usrp2::get_samples(vector<complex<float>> &buff, double &time_first_sample) {
    /**
     * \fn get_samples
     * \brief Get samples from RF device.
     * \details
     * - Configure and open receiver stream
     * - Read samples into a temp buffer
     * - Resample temp buffer to fill input buffer
     *
     * \param[out] buff: Buffer in which data will be put. Number of samples is given by buffer size.
     * \param[out] time_first_sample: Time at which first sample is received
    */


    vector<complex<float>> buff2(ceil((float) buff.size()/resampling_ratio));

    size_t num_samples = buff2.size();
    uhd::set_thread_priority_safe();
    uhd::stream_args_t stream_args("fc32", "sc16");
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps  = size_t(num_samples);
    stream_cmd.stream_now = true;
    rx_stream->issue_stream_cmd(stream_cmd);

    uhd::rx_metadata_t md;

    size_t total_rcvd_samples = 0;

    while (total_rcvd_samples<num_samples){
        size_t num_rx_samps = rx_stream->recv(&buff2.front(), buff2.size() - total_rcvd_samples, md, 10.0, false);
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
            cout << boost::format("Timeout while streaming") << endl;
            break;
        }
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
            cout << boost::format("Overflow\n") << endl;

            continue;
        }
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
            string error = str(boost::format("Receiver error: %s") % md.strerror());
        }
        total_rcvd_samples += num_rx_samps;
    }
    uhd::time_spec_t time_spec = md.time_spec;
    time_first_sample = time_spec.get_full_secs();
    time_first_sample += time_spec.get_frac_secs();
    
    complex<float> output_filter[buff.size()];
    unsigned int num_written;
    unsigned int num_samples_resampled = 0;
    for (int i =0; i < buff2.size(); i ++){
        resamp_crcf_execute(resampling_filter, buff2[i], &output_filter[num_samples_resampled], &num_written);
        num_samples_resampled += num_written;
    }

    for (int i = 0; i < buff.size(); i ++){
        buff[i] = output_filter[i];
    }

}


void free5GRAN::usrp_usrp2::setSampleRate(double rate) {
    /**
     * \fn setSampleRate
     * \brief Set RF device sampling rate
     * \param[in] rate: Sampling rate in Hz
    */
    adjust_sampling_rate(rate);
    this->usrp->set_rx_rate(this -> internal_sampling_rate);
    usrp->set_rx_bandwidth(this -> internal_sampling_rate);
}

double free5GRAN::usrp_usrp2::getCenterFrequency() {
    /**
     * \fn getCenterFrequency
     * \return RF device center frequency in Hz
    */
    return this->center_frequency;
}

void free5GRAN::usrp_usrp2::setCenterFrequency(double freq) {
    /**
     * \fn setCenterFrequency
     * \brief Change RF device center frequency
     * \param[in] freq: Center frequency in Hz
    */
    uhd::tune_request_t tune_request(freq);
    this->usrp->set_rx_freq(tune_request);
    this->center_frequency = freq;

}

void free5GRAN::usrp_usrp2::setGain(double gain) {
    /**
     * \fn setGain
     * \brief Change RF device reception gain
     * \param[in] gain: Gain in dB
    */
    this->gain = gain;
    this->usrp->set_rx_gain(this->gain);
}

double free5GRAN::usrp_usrp2::getGain() {
    /**
     * \fn getGain
     * \return RF device gain in dB
    */
    return this->gain;
}

void free5GRAN::usrp_usrp2::adjust_sampling_rate(double &rate) {
    /*
     * \fn adjust_sampling_rate
     * \brief Adjust free5GRAN desired sampling rate to USRP N210 achievable sampling rate. Setup LIQUID DSP resampler
     * \param[in] rate: Sampling rate in Hz.
     */
    if (rate == 30.72e6) {
        rate = 23.04e6;
    }
    this->sample_rate = rate;
    this -> internal_sampling_rate = 25e6;
    resampling_ratio = rate / this -> internal_sampling_rate;

    if (filter_init_done){
        resamp_crcf_destroy(resampling_filter);
    }
    resampling_filter = resamp_crcf_create(resampling_ratio,13,0.45f,60,32);
    filter_init_done = true;
}

