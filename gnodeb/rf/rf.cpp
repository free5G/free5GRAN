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
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
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


    //On envoie des infos à l'USRP
    // usrp est un objet de type usrp... qui est un attribu de rf (pk dfin dasn rf.h).
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


    //configure Tx
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




void rf::buffer_transmition(std::vector<std::complex<float>> &buff) {
    BOOST_LOG_TRIVIAL(warning) << "Function buffer_transmition begins ";

    uhd::stream_args_t stream_args("fc32", "sc16");
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);

    uhd::tx_metadata_t md;
    md.start_of_burst = false;
    md.end_of_burst = false;

    std::cout << "Sending Frame indefinitely...."<<std::endl;

    /** Initialize variables to mesurate time to send */
    auto start_send = chrono::high_resolution_clock::now(), stop_send = chrono::high_resolution_clock::now();
    auto duration_send = chrono::duration_cast<chrono::microseconds>(stop_send - start_send);
    auto duration_between_send = chrono::duration_cast<chrono::microseconds>(stop_send - start_send);
    int duration_send_int, i = 0, duration_sum_send = 0, duration_between_send_int = 0, duration_sum_between_send = 0;
    int number_calculate_mean = 400; /** indicates the number of iterations of 'while true' before display the mean durations */

    while (true) {
            free5GRAN::mtx_common.lock();
            start_send = chrono::high_resolution_clock::now();

            /** Calculate mean duration between 2 sends */
            if (i < number_calculate_mean) {
                duration_between_send = chrono::duration_cast<chrono::microseconds>(start_send- stop_send);
                duration_between_send_int = duration_between_send.count();
                duration_sum_between_send = duration_sum_between_send + duration_between_send_int;
            }

            /** Send frame */
            tx_stream->send(&buff.front(), buff.size(), md);
            stop_send = chrono::high_resolution_clock::now();
            free5GRAN::mtx_common.unlock();
            BOOST_LOG_TRIVIAL(warning) << "A frame has been sent ";
            usleep(1);

        /** Calculate the mean duration of the number_calculate_mean first call */
        if (i < number_calculate_mean) {
                duration_send = chrono::duration_cast<chrono::microseconds>(stop_send- start_send);
                duration_send_int = duration_send.count();
                duration_sum_send = duration_sum_send + duration_send_int;
        }
        /** Display the mean duration */
        if (i == number_calculate_mean + 1) {
            float mean_duration_send = (duration_sum_send / number_calculate_mean);
            float mean_duration_between_send = (duration_sum_between_send / number_calculate_mean);
            cout <<"\nduration of send (mean of "<<number_calculate_mean<<" last) = " << mean_duration_send / 1000 << " ms" << endl;
            cout <<"duration between 2 send (mean of "<<number_calculate_mean<<" last) = " << mean_duration_between_send / 1000 << " ms" << endl;
            duration_sum_send = 0;
            duration_sum_between_send = 0;
        }
        i = (i + 1) % 3000;
    }
}



void buffer_transm_test_mutex(std::vector<std::complex<float>> &buff) {
    std::cout<<"Function send_buffer_test_mutex begins "<<std::endl;
    unsigned int time_to_send = 10000;
    while (true) {
        free5GRAN::mtx_common.lock();
        BOOST_LOG_TRIVIAL(warning) << "One loop from buff_transm_test_mutex ";
        std::cout<<"SENDING TEST THREAD"<<std::endl;
        usleep(time_to_send);

        free5GRAN::mtx_common.unlock();
        usleep(1);

    }
}