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
 * \date January 2021
 */

#include "rf.h"
#include "../phy/phy.h"
#include <uhd.h>
#include <boost/format.hpp>
#include <iostream>
#include <complex>
#include <vector>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <mutex>
#include <unistd.h>


/*
 * Initialize USRP device
 */
rf::rf(double sample_rate, double center_frequency, double gain, double bandwidth,
               std::string subdev, std::string antenna_mode, std::string ref, std::string device_args) {

    //uhd::set_thread_priority_safe();

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

    this->usrp->set_tx_subdev_spec(this->subdev );
    this->usrp->set_tx_rate(this->sample_rate);
    this->usrp->set_tx_freq(tune_request);
    this->usrp->set_tx_gain(this->gain);
    this->usrp->set_tx_bandwidth(this->bandwidth);
    this->usrp->set_tx_antenna(this->antenna_mode);
}

double rf::getSampleRate() const {
    return usrp->get_rx_rate();
}

/*
 * Get samples from USRP device
 */
void rf::get_samples(std::vector<std::complex<float>> *buff, double &time_first_sample) {
    size_t num_samples = buff->size();
    //uhd::set_thread_priority_safe();
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
    this->usrp->set_rx_rate(rate);
    this->sample_rate = rate;
    usrp->set_rx_bandwidth(rate);
}

double rf::getCenterFrequency() {
    return this->center_frequency;
}

void rf::setCenterFrequency(double freq) {
    uhd::tune_request_t tune_request(freq);
    this->usrp->set_rx_freq(tune_request);
    this->center_frequency = freq;

}

void rf::setGain(double gain) {
    this->gain = gain;
    this->usrp->set_rx_gain(this->gain);
}

double rf::getGain() {
    return this->gain;
}




void rf::buffer_transmition(std::vector<std::complex<float>> &buff1, std::vector<std::complex<float>> &buff2) {
    BOOST_LOG_TRIVIAL(warning) << "Function buffer_transmition begins ";

    uhd::stream_args_t stream_args("fc32", "sc16");
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);

    uhd::tx_metadata_t md;
    md.start_of_burst = false;
    md.end_of_burst = false;

    std::cout << "Sending Frame indefinitely...."<<std::endl;

    /** Initialize variables to measure time to send */
    auto start_send1 = chrono::high_resolution_clock::now(), stop_send1 = chrono::high_resolution_clock::now();
    auto start_send2 = chrono::high_resolution_clock::now(), stop_send2 = chrono::high_resolution_clock::now();
    auto duration_send1 = chrono::duration_cast<chrono::microseconds>(stop_send1 - start_send1);
    auto duration_send2 = chrono::duration_cast<chrono::microseconds>(stop_send2 - start_send2);
    auto duration_between_send1and2 = chrono::duration_cast<chrono::microseconds>(start_send2 - stop_send1);
    int duration_send1_int, i = 0, duration_sum_send = 0, duration_send2_int = 0, duration_sum_send2 = 0, duration_between_send1and2_int = 0, duration_sum_between_send1and2 = 0;
    int number_calculate_mean = 400; /** indicates the number of iterations of 'while true' before display the mean durations */

    while (true) {





            /** Send buffer 1*/
            start_send1 = chrono::high_resolution_clock::now();
            tx_stream->send(&buff1.front(), buff1.size(), md);
            stop_send1 = chrono::high_resolution_clock::now();
            sem_post(&free5GRAN::semaphore_common1); // release semaphore_common1 to let the main thread generate the next buffer1
            BOOST_LOG_TRIVIAL(warning) << "Buffer 1 has been sent ";

            /** Send buffer 2*/
            start_send2 = chrono::high_resolution_clock::now();
            tx_stream->send(&buff2.front(), buff2.size(), md);
            stop_send2 = chrono::high_resolution_clock::now();
            sem_post(&free5GRAN::semaphore_common2); // release semaphore_common2 to let the main thread generate the next buffer2
            BOOST_LOG_TRIVIAL(warning) << "Buffer 2 has been sent ";



        /** Calculate the mean duration of the number_calculate_mean first call */
        if (i < number_calculate_mean) {
                duration_send1 = chrono::duration_cast<chrono::microseconds>(stop_send1- start_send1);
                duration_send2 = chrono::duration_cast<chrono::microseconds>(stop_send2- start_send2);
                duration_send1_int = duration_send1.count(), duration_send2_int = duration_send2.count();
                duration_sum_send = duration_sum_send + duration_send1_int + duration_send2_int;
                duration_between_send1and2 = chrono::duration_cast<chrono::microseconds>(start_send2- stop_send1);
                duration_between_send1and2_int = duration_between_send1and2.count();
                duration_sum_between_send1and2 += duration_between_send1and2_int;

        }

        /** Display the mean duration */
        if (i == number_calculate_mean + 1) {
            float mean_duration_send = (duration_sum_send / (2*number_calculate_mean));
            float mean_duration_between_send1and2 = (duration_sum_between_send1and2 / number_calculate_mean);
            cout <<"\nduration of send (mean of "<<number_calculate_mean<<" last) = " << mean_duration_send / 1000 << " ms" << endl;
            cout <<"duration between send 1 and 2 (mean of "<<number_calculate_mean<<" last) = " << mean_duration_between_send1and2 / 1000 << " ms" << endl;
            duration_sum_send = 0;
            duration_sum_between_send1and2 = 0;
        }
        i = (i + 1) % 3000;
    }
}