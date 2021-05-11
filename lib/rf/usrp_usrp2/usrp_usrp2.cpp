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

#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>
#include <chrono>
#include <complex>
#include <iostream>
#include <uhd/utils/thread.hpp>
#include <vector>

free5GRAN::usrp_usrp2::usrp_usrp2(double sample_rate,
                                  double center_frequency,
                                  double gain,
                                  double bandwidth,
                                  free5GRAN::rf_device chosen_device,
                                  free5GRAN::rf_buffer* rf_buff)
    : rf() {
  /**
   * \fn usrp_usrp2
   * \param[in] sample_rate: RF sampling rate
   * \param[in] center_frequency: Frequency center
   * \param[in] gain: Reception gain
   * \param[in] bandwidth: Received bandwidth
   * \param[in] chosen_device: RF device object
   */
  // Set buffer size
  int res;
  res = system("sudo sysctl -w net.core.wmem_max=50000000");
  res = system("sudo sysctl -w net.core.rmem_max=50000000");
  res = system("sudo sysctl -w net.core.wmem_default=50000000");
  res = system("sudo sysctl -w net.core.rmem_default=50000000");

  string device_args("serial=" + chosen_device.serial);
  device_args +=
      ", recv_frame_size=1472, send_frame_size=1472, num_recv_frames=64, "
      "num_send_frames=64";
  string subdev;
  subdev = (chosen_device.subdev.empty()) ? "A:0" : chosen_device.subdev;
  string ant("TX/RX");
  string ref;
  ref = (chosen_device.ref.empty()) ? "internal" : chosen_device.ref;
  this->rf_buff = rf_buff;

  adjust_sampling_rate(sample_rate);

  this->sample_rate = sample_rate;
  this->center_frequency = center_frequency;
  this->gain = gain;
  this->bandwidth = bandwidth;

  uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(device_args);
  this->usrp = usrp;
  this->usrp->set_clock_source(ref);
  this->usrp->set_rx_subdev_spec(subdev);
  this->usrp->set_rx_rate(this->internal_sampling_rate);
  uhd::tune_request_t tune_request(this->center_frequency);
  this->usrp->set_rx_freq(tune_request);
  this->usrp->set_rx_gain(this->gain);
  this->usrp->set_rx_bandwidth(this->internal_sampling_rate);
  this->usrp->set_rx_antenna(ant);
}

auto free5GRAN::usrp_usrp2::getSampleRate() -> double {
  /**
   * \fn getSampleRate
   * \return RF device sampling rate, in Hz
   */
  return this->sample_rate;
}

/*
 * Get samples from USRP device
 */
void free5GRAN::usrp_usrp2::get_samples(vector<complex<float>>& buff,
                                        double& time_first_sample) {
  /**
   * \fn get_samples
   * \brief Get samples from RF device.
   * \details
   * - Configure and open receiver stream
   * - Read samples into a temp buffer
   * - Resample temp buffer to fill input buffer
   *
   * \param[out] buff: Buffer in which data will be put. Number of samples is
   * given by buffer size. \param[out] time_first_sample: Time at which first
   * sample is received
   */

  // Set thread priority
  uhd::set_thread_priority_safe(1.0, true);

  // Create a secondary buffer that will contain the signal with the N210
  // sampling rate
  vector<complex<float>> buff2(ceil((float)buff.size() / resampling_ratio));

  size_t num_samples = buff2.size();
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
        &buff2.front(), buff2.size() - total_rcvd_samples, md, 10.0, false);
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

  // Resample received signal using Liquid DSP
  complex<float> output_filter[buff.size()];
  unsigned int num_written;
  unsigned int num_samples_resampled = 0;
  // Apply resampling filter
  for (auto& i : buff2) {
    resamp_crcf_execute(resampling_filter, i,
                        &output_filter[num_samples_resampled], &num_written);
    num_samples_resampled += num_written;
  }
  // Copy resampling filter ouput to output buffer
  for (int i = 0; i < buff.size(); i++) {
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
  this->usrp->set_rx_rate(this->internal_sampling_rate);
  usrp->set_rx_bandwidth(this->internal_sampling_rate);
}

auto free5GRAN::usrp_usrp2::getCenterFrequency() -> double {
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

auto free5GRAN::usrp_usrp2::getGain() -> double {
  /**
   * \fn getGain
   * \return RF device gain in dB
   */
  return this->gain;
}

void free5GRAN::usrp_usrp2::adjust_sampling_rate(double& rate) {
  /*
   * \fn adjust_sampling_rate
   * \brief Adjust free5GRAN desired sampling rate to USRP N210 achievable
   * sampling rate. Setup LIQUID DSP resampler \param[in] rate: Sampling rate in
   * Hz.
   */
  if (rate == 30.72e6) {
    rate = 23.04e6;
  }
  this->sample_rate = rate;
  this->internal_sampling_rate = 25e6;
  resampling_ratio = rate / this->internal_sampling_rate;

  if (filter_init_done) {
    resamp_crcf_destroy(resampling_filter);
  }
  resampling_filter = resamp_crcf_create(resampling_ratio, 13, 0.45f, 60, 32);
  filter_init_done = true;
}

void free5GRAN::usrp_usrp2::start_loopback_recv(bool& stop_signal,
                                                size_t buff_size) {
  /**
   * \fn start_loopback_recv
   * \brief Continuous receive of frames
   * \details
   * Details:
   * - Initialize a rx stream
   * - Continuously receive frame
   * - Push them to the temporary buffer
   * - Notify resampling filter that a new frame can be resampled
   *
   * \param[in] buff_size: Buffer (frame) size
   * \param[in] stop_signal: While loop switch
   */
  // Set thread priority
  uhd::set_thread_priority_safe(1.0, true);
  cout << "Starting recv thread" << endl;
  // Create a temporary buffer that will contain the signal with the N210
  // sampling rate
  size_t usrp2_signal_size = ceil(buff_size / resampling_ratio);
  vector<complex<float>> tmp_buffer(usrp2_signal_size),
      tmp_buffer_shared(usrp2_signal_size);
  bool overflow = false;
  bool shared_overflow;

  // Initialize semaphore for signalisation with resampling thread
  sem_t filter_sem;
  sem_init(&filter_sem, 1, 0);

  // Start resampling filter thread to adjust sampling rate from N210 to
  // free5GRAN
  boost::thread filtering_thread(
      [this, &tmp_buffer_shared, &filter_sem, &stop_signal, &shared_overflow] {
        loopback_filter(tmp_buffer_shared, filter_sem, stop_signal,
                        shared_overflow);
      });
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
  // Start rx stream
  rx_stream->issue_stream_cmd(stream_cmd);

  // Endless loop
  while (!stop_signal) {
    size_t total_rcvd_samples = 0;
    overflow = false;
    // While buffer is not full
    while (total_rcvd_samples < buff_size) {
      // Receive samples
      size_t num_rx_samps = rx_stream->recv(
          &tmp_buffer.front(), usrp2_signal_size - total_rcvd_samples, md);
      // If receive timeout
      if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
        cout << boost::format("Timeout while streaming") << endl;
        break;
      }
      // If receive Overflow
      if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
        cout << boost::format("Overflow\n") << endl;
        overflow = true;
        continue;
      }
      // Else
      if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
        string error = str(boost::format("Receiver error: %s") % md.strerror());
      }
      // Increment the samples counter
      total_rcvd_samples += num_rx_samps;
    }
    // Copy tmp_buffer to shared temp buffer
    tmp_buffer_shared = tmp_buffer;
    shared_overflow = overflow;
    // Notify resampling thread that a new frame has been created
    sem_post(&filter_sem);
  }
  // Stop continuous reception
  stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
  rx_stream->issue_stream_cmd(stream_cmd);

  // Wait resampling thread to finish
  filtering_thread.join();
  // Reset primary frame id
  primary_frame_id = 0;
  cout << "Finishing recv thread" << endl;
}

void free5GRAN::usrp_usrp2::loopback_filter(
    vector<complex<float>>& tmp_buffer_shared,
    sem_t& filtering_sem,
    bool& stop_signal,
    bool& overflow) {
  /**
   * \fn loopback_filter
   * \brief Continuously resample frames from USRP N210 sampling rate to
   * free5GRAN sampling rate \details Details:
   * - Get a received frame
   * - Apply resampling filter
   * - Push it to the primary buffer
   *
   * \param[in] tmp_buffer_shared: Buffer containing the new received frame
   * \param[in] filtering_sem: Semaphore signal to notify the reception of a new
   * frame \param[in] stop_signal: While loop switch \param[in] overflow:
   * Overflow switch
   */

  // Set thread priority
  uhd::set_thread_priority_safe(0.75, false);
  cout << "Starting filtering thread" << endl;
  // Compute final buffer size (which should be equal to start_loopback_recv
  // input buffer size)
  size_t buff_size = floor(tmp_buffer_shared.size() * resampling_ratio);
  // Create a temp buffer
  vector<complex<float>> tmp_buffer(tmp_buffer_shared.size());

  // Create a new element for primary buffer
  free5GRAN::buffer_element new_elem = {
      .frame_id = primary_frame_id,
      .buffer = vector<complex<float>>(buff_size)};

  bool notified_all = false;
  bool last_notify = false;

  // Endless loop
  while (!stop_signal) {
    /*
     * Reset primary_frame_id every minute
     */
    if (primary_frame_id == 6000) {
      primary_frame_id = 0;
    }
    // Wait the reception of a new frame
    sem_wait(&filtering_sem);
    // Copy the new frame into the temp buffer
    tmp_buffer = tmp_buffer_shared;

    // Resample new frame to free5GRAN sampling rate
    complex<float> output_filter[buff_size];
    unsigned int num_written;
    unsigned int num_samples_resampled = 0;
    for (auto& i : tmp_buffer) {
      resamp_crcf_execute(resampling_filter, i,
                          &output_filter[num_samples_resampled], &num_written);
      num_samples_resampled += num_written;
    }
    // Copy resampling filter output to new element buffer
    for (int i = 0; i < buff_size; i++) {
      new_elem.buffer[i] = output_filter[i];
    }
    // Set new element frame ID
    new_elem.frame_id = primary_frame_id;
    new_elem.overflow = overflow;
    if (overflow) {
      cout << "OVERFLOW DETECTED" << endl;
    }
    // Push the new element into the primary buffer
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

  cout << "Finishing filtering thread" << endl;
}
