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

#include "rf.h"
#include <uhd.h>
#include <uhd/utils/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <complex>
#include <vector>

/*
 * Initialize USRP device
 */
rf::rf(double sample_rate, double center_frequency, double gain, double bandwidth,
         std::string subdev, std::string antenna_mode, std::string ref, std::string device_args) {

    /**
     * \fn rf
     * \param[in] sample_rate: RF sampling rate
     * \param[in] center_frequency: Frequency center
     * \param[in] gain: Reception gain
     * \param[in] bandwidth: Received bandwidth
     * \param[in] subdev: Subdevice to be used on USRP
     * \param[in] antenna_mode: USRP antenna mode
     * \param[in] ref: USRP time reference
     * \param[in] device_args: USRP arguments such as serial address
    */

    uhd::set_thread_priority_safe();

    this->sample_rate = sample_rate;
    this->center_frequency = center_frequency;
    this->gain = gain;
    this->bandwidth = bandwidth;
    this->device_args = device_args;
    this->subdev = subdev;
    this->antenna_mode = antenna_mode;
    this->ref = ref;

    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(device_args);
    this->usrp = usrp;
    this->usrp->set_clock_source(this->ref);
    this->usrp->set_rx_subdev_spec(this->subdev );
    this->usrp->set_rx_rate(this->sample_rate);
    uhd::tune_request_t tune_request(this->center_frequency);
    this->usrp->set_rx_freq(tune_request);
    this->usrp->set_rx_gain(this->gain);
    this->usrp->set_rx_bandwidth(this->bandwidth);
    this->usrp->set_rx_antenna(this->antenna_mode);
}

double rf::getSampleRate() const {
    /**
     * \fn getSampleRate
     * \return RF device sampling rate, in Hz
    */
    return usrp->get_rx_rate();
}

/*
 * Get samples from USRP device
 */
void rf::get_samples(std::vector<std::complex<float>> *buff, double &time_first_sample) {
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
    size_t num_samples = buff->size();
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
        size_t num_rx_samps = rx_stream->recv(&buff->front(), buff->size() - total_rcvd_samples, md, 10.0, false);
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
            std::cout << boost::format("Timeout while streaming") << std::endl;
            break;
        }
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
            std::cout << boost::format("Overflow\n") << std::endl;

            continue;
        }
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
            std::string error = str(boost::format("Receiver error: %s") % md.strerror());
        }
        total_rcvd_samples += num_rx_samps;
    }
    uhd::time_spec_t time_spec = md.time_spec;
    time_first_sample = time_spec.get_full_secs();
    time_first_sample += time_spec.get_frac_secs();
}


void rf::setSampleRate(double rate) {
    /**
     * \fn setSampleRate
     * \brief Set RF device sampling rate
     * \param[in] rate: Sampling rate in Hz
    */
    this->usrp->set_rx_rate(rate);
    this->sample_rate = rate;
    usrp->set_rx_bandwidth(rate);
}

double rf::getCenterFrequency() {
    return this->center_frequency;
}

void rf::setCenterFrequency(double freq) {
    /**
     * \fn setCenterFrequency
     * \brief Change RF device center frequency
     * \param[in] freq: Center frequency in Hz
    */
    uhd::tune_request_t tune_request(freq);
    this->usrp->set_rx_freq(tune_request);
    this->center_frequency = freq;

}

void rf::setGain(double gain) {
    /**
     * \fn setGain
     * \brief Change RF device reception gain
     * \param[in] gain: Gain in dB
    */
    this->gain = gain;
    this->usrp->set_rx_gain(this->gain);
}

double rf::getGain() {
    return this->gain;
}
