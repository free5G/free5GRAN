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

#include "usrp_x300.h"

#include <uhd.h>

#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>
#include <complex>
#include <iostream>
#include <uhd/utils/thread.hpp>
#include <vector>

free5GRAN::usrp_x300::usrp_x300(double sample_rate,
                                double center_frequency,
                                double gain,
                                double bandwidth,
                                free5GRAN::rf_device chosen_device,
                                free5GRAN::rf_buffer* rf_buff)
    : rf() {
  /**
   * \fn usrp_x300
   * \param[in] sample_rate: RF sampling rate
   * \param[in] center_frequency: Frequency center
   * \param[in] gain: Reception gain
   * \param[in] bandwidth: Received bandwidth
   * \param[in] chosen_device: RF device object
   */
  // Set buffer size
  int res;
  res = system("sudo sysctl -w net.core.wmem_max=33554432");
  res = system("sudo sysctl -w net.core.rmem_max=33554432");
  res = system("sudo sysctl -w net.core.wmem_default=33554432");
  res = system("sudo sysctl -w net.core.rmem_default=33554432");

  string device_args("serial=" + chosen_device.serial);
  device_args +=
      ", master_clock_rate=184.32e6, recv_frame_size=7972, "
      "send_frame_size=7972, num_recv_frames=256, num_send_frames=256";
  string subdev;
  subdev = (chosen_device.subdev.empty()) ? "A:0" : chosen_device.subdev;
  string ant("TX/RX");
  string ref;
  ref = (chosen_device.ref.empty()) ? "internal" : chosen_device.ref;
  this->rf_buff = rf_buff;

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

auto free5GRAN::usrp_x300::getSampleRate() -> double {
  /**
   * \fn getSampleRate
   * \return RF device sampling rate, in Hz
   */
  return usrp->get_rx_rate();
}

/*
 * Get samples from USRP device
 */
void free5GRAN::usrp_x300::get_samples(vector<complex<float>>& buff,
                                       double& time_first_sample) {
  /**
   * \fn get_samples
   * \brief Get samples from RF device.
   * \details
   * - Configure and open receiver stream
   * - Read samples until buffer is full
   *
   * \param[out] buff: Buffer in which data will be put. Number of samples is
   * given by buffer size. \param[out] time_first_sample: Time at which first
   * sample is received
   */
  // Set thread priority
  uhd::set_thread_priority_safe(1.0, true);
  // Get buffer size
  size_t num_samples = buff.size();
  // Configure stream arguments
  uhd::stream_args_t stream_args("fc32", "sc16");
  // Get USRP stream
  uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);
  // Configure the stream to receive a given number of symbols
  uhd::stream_cmd_t stream_cmd(
      uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
  stream_cmd.num_samps = size_t(num_samples);
  // Start streaming as soon as possible
  stream_cmd.stream_now = true;
  // Start rx stream
  rx_stream->issue_stream_cmd(stream_cmd);
  // Create metadata object
  uhd::rx_metadata_t md;

  size_t total_rcvd_samples = 0;
  // While buffer is not full
  while (total_rcvd_samples < num_samples) {
    // Receive samples
    size_t num_rx_samps = rx_stream->recv(
        &buff.front(), buff.size() - total_rcvd_samples, md, 10.0, false);
    // If receive timeout
    if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
      cout << boost::format("Timeout while streaming") << endl;
      break;
    }
    // If receive Overflow
    if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
      cout << boost::format("Overflow\n") << endl;
      continue;
    }
    // Else
    if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
      string error = str(boost::format("Receiver error: %s") % md.strerror());
    }
    // Increment the samples counter
    total_rcvd_samples += num_rx_samps;
  }
  // Set receive time
  uhd::time_spec_t time_spec = md.time_spec;
  time_first_sample = time_spec.get_full_secs();
  time_first_sample += time_spec.get_frac_secs();
}

void free5GRAN::usrp_x300::setSampleRate(double rate) {
  /**
   * \fn setSampleRate
   * \brief Set RF device sampling rate
   * \param[in] rate: Sampling rate in Hz
   */
  this->usrp->set_rx_rate(rate);
  this->sample_rate = rate;
  usrp->set_rx_bandwidth(rate);
}

auto free5GRAN::usrp_x300::getCenterFrequency() -> double {
  /**
   * \fn getCenterFrequency
   * \return RF device center frequency in Hz
   */
  return this->center_frequency;
}

void free5GRAN::usrp_x300::setCenterFrequency(double freq) {
  /**
   * \fn setCenterFrequency
   * \brief Change RF device center frequency
   * \param[in] freq: Center frequency in Hz
   */
  uhd::tune_request_t tune_request(freq);
  this->usrp->set_rx_freq(tune_request);
  this->center_frequency = freq;
}

void free5GRAN::usrp_x300::setGain(double gain) {
  /**
   * \fn setGain
   * \brief Change RF device reception gain
   * \param[in] gain: Gain in dB
   */
  this->gain = gain;
  this->usrp->set_rx_gain(this->gain);
}

auto free5GRAN::usrp_x300::getGain() -> double {
  /**
   * \fn getGain
   * \return RF device gain in dB
   */
  return this->gain;
}

void free5GRAN::usrp_x300::start_loopback_recv(bool& stop_signal,
                                               size_t buff_size) {
  /**
   * \fn start_loopback_recv
   * \brief Continuous receive of frames
   * \details
   * Details:
   * - Initialize a rx stream
   * - Continuously receive frame
   * - Push them to the primary buffer
   *
   * \param[in] buff_size: Buffer (frame) size
   * \param[in] stop_signal: While loop switch
   */
  // Set thread priority
  uhd::set_thread_priority_safe(1.0, true);
  // Create a new element
  free5GRAN::buffer_element new_elem = {
      .frame_id = primary_frame_id,
      .buffer = vector<complex<float>>(buff_size)};
  cout << "Starting recv thread" << endl;

  // Configure stream arguments
  uhd::stream_args_t stream_args("fc32", "sc16");
  // Get USRP stream
  uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);
  // Ask for continuous receiving
  uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
  // Start reception now
  stream_cmd.stream_now = true;
  // Create metadata object
  uhd::rx_metadata_t md;

  bool notified_all = false;
  bool last_notify = false;

  // Start rx stream
  rx_stream->issue_stream_cmd(stream_cmd);

  // Endless loop
  while (!stop_signal) {
    /*
     * Reset primary_frame_id every minute
     */
    if (primary_frame_id == 6000) {
      primary_frame_id = 0;
    }
    new_elem.overflow = false;
    size_t total_rcvd_samples = 0;
    // While buffer is not full
    while (total_rcvd_samples < buff_size) {
      // Receive samples
      size_t num_rx_samps = rx_stream->recv(&new_elem.buffer.front(),
                                            buff_size - total_rcvd_samples, md);
      // If receive timeout
      if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
        cout << boost::format("Timeout while streaming") << endl;
        break;
      }
      // If receive Overflow
      if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
        cout << boost::format("Overflow\n") << endl;
        new_elem.overflow = true;
        continue;
      }
      // Else
      if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
        string error = str(boost::format("Receiver error: %s") % md.strerror());
      }
      // Increment the samples counter
      total_rcvd_samples += num_rx_samps;
    }
    // Set new frame id to primary frame id
    new_elem.frame_id = primary_frame_id;
    // Push the new frame inside the primary buffer
    rf_buff->primary_buffer->push_back(new_elem);
    // Notify clients when buffer size increases
    if (last_notify) {
      (*rf_buff->cond_var_vec_prim_buffer)[rf_buff->primary_buffer->size() - 1]
          .notify_all();
      last_notify = false;
    }
    if (!notified_all) {
      (*rf_buff->cond_var_vec_prim_buffer)[rf_buff->primary_buffer->size() - 1]
          .notify_all();
      if (rf_buff->primary_buffer->size() ==
          rf_buff->primary_buffer->capacity() - 1) {
        last_notify = true;
        notified_all = true;
      }
    }

    // If adjust thread is started
    if (rf_buff->frame_thread_started) {
      // Notify it that a new frame has been pushed to the primary buffer
      sem_post(rf_buff->semaphore);
    }
    // Increment primary frame id
    primary_frame_id++;
  }

  // Stop continuous reception
  stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
  rx_stream->issue_stream_cmd(stream_cmd);
  // Reset primary frame id
  primary_frame_id = 0;
  cout << "Finishing recv thread" << endl;
}
