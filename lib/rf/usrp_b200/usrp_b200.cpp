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

#include "usrp_b200.h"
#include <uhd.h>
#include <uhd/utils/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <complex>
#include <vector>


free5GRAN::usrp_b200::usrp_b200(double sample_rate, double center_frequency, double gain, double bandwidth, free5GRAN::rf_device chosen_device) : rf() {
    /**
     * \fn usrp_b200
     * \param[in] sample_rate: RF sampling rate
     * \param[in] center_frequency: Frequency center
     * \param[in] gain: Reception gain
     * \param[in] bandwidth: Received bandwidth
     * \param[in] chosen_device: RF device object
    */
    uhd::set_thread_priority_safe();

    string device_args("serial="+chosen_device.serial);
    string subdev;
    subdev = (chosen_device.subdev.empty()) ? "A:A" : chosen_device.subdev;
    string ant("TX/RX");
    string ref;
    ref = (chosen_device.ref.empty()) ? "internal" : chosen_device.ref;

    this->sample_rate = sample_rate;
    this->center_frequency = center_frequency;
    this->gain = gain;
    this->bandwidth = bandwidth;

    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(device_args);
    this->usrp = usrp;
    this->usrp->set_clock_source(ref);
    this->usrp->set_rx_subdev_spec(subdev);
    this->usrp->set_rx_rate(this->sample_rate);
    uhd::tune_request_t tune_request(this->center_frequency);
    this->usrp->set_rx_freq(tune_request);
    this->usrp->set_rx_gain(this->gain);
    this->usrp->set_rx_bandwidth(this->bandwidth);
    this->usrp->set_rx_antenna(ant);

}

double free5GRAN::usrp_b200::getSampleRate() {
    /**
     * \fn getSampleRate
     * \return RF device sampling rate, in Hz
    */
    return usrp->get_rx_rate();
}

/*
 * Get samples from USRP device
 */
void free5GRAN::usrp_b200::get_samples(vector<complex<float>> &buff, double &time_first_sample) {
    /**
     * \fn get_samples
     * \brief Get samples from RF device.
     * \details
     * - Configure and open receiver stream
     * - Read samples until buffer is full
     *
     * \param[out] buff: Buffer in which data will be put. Number of samples is given by buffer size.
     * \param[out] time_first_sample: Time at which first sample is received
    */
    size_t num_samples = buff.size();
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
        size_t num_rx_samps = rx_stream->recv(&buff.front(), buff.size() - total_rcvd_samples, md, 10.0, false);
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
}


void free5GRAN::usrp_b200::setSampleRate(double rate) {
    /**
     * \fn setSampleRate
     * \brief Set RF device sampling rate
     * \param[in] rate: Sampling rate in Hz
    */
    this->usrp->set_rx_rate(rate);
    this->sample_rate = rate;
    usrp->set_rx_bandwidth(rate);
}

double free5GRAN::usrp_b200::getCenterFrequency() {
    /**
     * \fn getCenterFrequency
     * \return RF device center frequency in Hz
    */
    return this->center_frequency;
}

void free5GRAN::usrp_b200::setCenterFrequency(double freq) {
    /**
     * \fn setCenterFrequency
     * \brief Change RF device center frequency
     * \param[in] freq: Center frequency in Hz
    */
    uhd::tune_request_t tune_request(freq);
    this->usrp->set_rx_freq(tune_request);
    this->center_frequency = freq;

}

void free5GRAN::usrp_b200::setGain(double gain) {
    /**
     * \fn setGain
     * \brief Change RF device reception gain
     * \param[in] gain: Gain in dB
    */
    this->gain = gain;
    this->usrp->set_rx_gain(this->gain);
}

double free5GRAN::usrp_b200::getGain() {
    /**
     * \fn getGain
     * \return RF device gain in dB
    */
    return this->gain;
}
