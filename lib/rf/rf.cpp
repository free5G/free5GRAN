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

#include "rf.h"
#include <chrono>
#include <cmath>
#include <uhd/utils/thread.hpp>
#include "../phy/libphy/libphy.h"
#include "../phy/synchronization/synchronization.h"
#include "../variables/common_variables/common_variables.h"

using namespace std;

void free5GRAN::rf::adjust_frames(
    free5GRAN::synchronization_object& sync_object,
    size_t buff_size,
    bool& stop_signal,
    bool& local_stop_signal) {
  /**
   * \fn adjust_frames
   * \brief Adjust primary buffer frames into gNodeB-synchronized frames
   * \details
   * Details:
   * - Wait for a new primary frame
   * - Adjust last and penultimate primary frames to get one sychronized radio
   * frame based on synchronization index
   *
   * \param[in] sync_object: Structure containing synchronization variables
   * \param[in] buff_size: Buffer (frame) size
   * \param[in] stop_signal: While loop switch
   * \param[in] local_stop_signal: Secondary while loop switch
   */

  // Set thread priority using UHD library
  uhd::set_thread_priority_safe(0.5, false);

  // Create new buffer element
  free5GRAN::buffer_element new_elem = {
      .frame_id = 0, .buffer = vector<complex<float>>(buff_size)};
  // Wait primary buffer to be full before beginning
  mutex m;
  // If there is less than 2 elements in the buffer
  if (rf_buff->primary_buffer->size() < 2) {
    unique_lock<mutex> lk(m);
    // Wait buffer to contain 2 frames with a timeout of 2 sec
    (*rf_buff->cond_var_vec_prim_buffer)[1].wait_for(lk,
                                                     std::chrono::seconds(2));
    lk.unlock();
  }
  // If timeout occured
  if (rf_buff->primary_buffer->size() < 2) {
    return;
  }

  free5GRAN::buffer_element n_1_element, n_2_element;
  bool notified_all = false;
  bool last_notify = false;
  bool first_iteration = true;

  // Notify reception thread that adjust frame is started
  rf_buff->frame_thread_started = true;

  // Endless loop
  while ((!stop_signal) && (!local_stop_signal)) {
    // Wait for a new frame in the buffer
    sem_wait(rf_buff->semaphore);
    // Get last primary buffer element
    n_1_element =
        (*rf_buff->primary_buffer)[rf_buff->primary_buffer->size() - 1];
    if (first_iteration) {
      first_iteration = false;
      // Get the penultimate primary frame
      n_2_element =
          (*rf_buff->primary_buffer)[rf_buff->primary_buffer->size() - 2];
      // Compute new element SFN based on synchronization SFN and frame id
      new_elem.frame_id =
          (sync_object.sfn +
           (n_2_element.frame_id - sync_object.frame_id) % 1024) %
          1024;
    } else {
      // Increment SFN
      new_elem.frame_id++;
      if (new_elem.frame_id == 1024) {
        new_elem.frame_id = 0;
      }
    }
    // Create a synchronized frame base on two primary buffer frames using
    // synchronization index
    for (int j = 0; j < buff_size - sync_object.sync_index; j++) {
      new_elem.buffer[j] = n_2_element.buffer[j + sync_object.sync_index];
    }
    for (int j = 0; j < sync_object.sync_index; j++) {
      new_elem.buffer[j + buff_size - sync_object.sync_index] =
          n_1_element.buffer[j];
    }
    // Copy last element to penultimate element
    n_2_element = n_1_element;
    // Apply fine frequency correction (sync_object.freq_offset has been
    // computed using cyclic prefixes)
    free5GRAN::phy::signal_processing::transpose_signal(
        &new_elem.buffer, sync_object.freq_offset, this->getSampleRate(),
        new_elem.buffer.size());
    // Push the newly generated frame into the buffer
    rf_buff->frame_buffer->push_back(new_elem);
    // Notify resynchronization thread that a new frame has been added to the
    // buffer
    sem_post(sync_object.cont_sync_sem);
    // Notify clients when buffer size increases
    if (last_notify) {
      (*rf_buff->cond_var_vec_frame_buffer)[rf_buff->frame_buffer->size() - 1]
          .notify_all();
      last_notify = false;
    }
    if (!notified_all) {
      (*rf_buff->cond_var_vec_frame_buffer)[rf_buff->frame_buffer->size() - 1]
          .notify_all();
      ;
      if (rf_buff->frame_buffer->size() ==
          rf_buff->frame_buffer->capacity() - 1) {
        last_notify = true;
        notified_all = true;
      }
    }
  }
  cout << "Finishing adjust thread" << endl;
}

auto free5GRAN::rf::get_frame_with_parity(int parity)
    -> free5GRAN::buffer_element {
  // If frame buffer is empty
  if (rf_buff->frame_buffer->size() == 0) {
    mutex m;
    unique_lock<mutex> lk(m);
    // Wait buffer to have one elment with 5 sec of timeout
    (*rf_buff->cond_var_vec_frame_buffer)[0].wait_for(lk,
                                                      std::chrono::seconds(5));
    lk.unlock();
  }
  // If frame buffer is empty after waiting, then timeout occured
  if (rf_buff->frame_buffer->size() == 0) {
    free5GRAN::buffer_element new_elem;
    new_elem.overflow = true;
    return new_elem;
  }
  // If the first element has the expected parity
  if ((*rf_buff->frame_buffer)[0].frame_id % 2 == parity) {
    // Return the first element
    return (*rf_buff->frame_buffer)[0];
  } else {
    // Otherwise, wait second element and return it
    if (rf_buff->frame_buffer->size() < 2) {
      mutex m2;
      unique_lock<mutex> lk2(m2);
      (*rf_buff->cond_var_vec_frame_buffer)[1].wait_for(
          lk2, std::chrono::seconds(5));
      lk2.unlock();
    }
    // If second frame did not come after waiting, then timeout occured
    if (rf_buff->frame_buffer->size() < 2) {
      free5GRAN::buffer_element new_elem;
      new_elem.overflow = true;
      return new_elem;
    }
    return (*rf_buff->frame_buffer)[1];
  }
}

void free5GRAN::rf::get_n_frame(vector<free5GRAN::buffer_element>& frames,
                                int n,
                                bool& timeout) {
  /**
   * \fn get_n_frame
   * \brief Get N frames from the frame buffer
   * \param[out] frames: Vector where frames are pushed
   * \param[in] n: Number of frames to be returned
   * \param[in] timeout: Wait timeout switch
   */

  timeout = false;
  // This function currently only support the case where the number of requested
  // frames is less than the max size of the buffer
  if (n <= rf_buff->frame_buffer->capacity()) {
    // Wait until there is n elements in the frame buffer with a timeout of 2
    // sec
    if (rf_buff->frame_buffer->size() < n + 1) {
      mutex m;
      unique_lock<mutex> lk(m);
      (*rf_buff->cond_var_vec_frame_buffer)[n].wait_for(
          lk, std::chrono::seconds(2));
      lk.unlock();
    }
    // Timeout occured
    if (rf_buff->frame_buffer->size() < n + 1) {
      timeout = true;
      cout << "Timeout detected" << endl;
      return;
    }
    // Return the n first elements
    for (int i = 0; i < n; i++) {
      frames.push_back((*rf_buff->frame_buffer)[i]);
    }
  }
}

void free5GRAN::rf::resynchronization(
    free5GRAN::synchronization_object& sync_object,
    bool& stop_signal,
    bool& local_stop_signal) {
  /**
   * \fn resynchronization
   * \brief Continuous resynchronization with cell
   * \details
   * Details:
   * - Wait for a new frame
   * - If this frame contains SSB, then
   * - Locate PSS and extract signal round it
   * - Search PSS inside this extracted signal and compute N_ID_2
   * - Locate SSS signal based on PSS synchronization and compute N_ID_1
   * - Recompute PCI and adjust cell synchronization parameter if PCI is correct
   *
   * \param[in] sync_object: Synchronization structure containing sync
   * information \param[in] stop_signal: While loop switch \param[in]
   * local_stop_signal: Secondary while loop switch
   */

  // Set thread priority using UHD library
  uhd::set_thread_priority_safe(0, false);

  // Computing frame size and symbol duration
  int frame_size = 0.01 * sample_rate;
  int symbol_duration = sync_object.common_cp_length + sync_object.fft_size_ssb;
  // searching in +/- sync_space samples around expected PSS position.
  // Initilized to a half symbol
  int sync_space = symbol_duration / 2;
  int last_resync_frame_id = 0;
  // Resynchronization period in frames
  int resync_period = 10;
  bool resync_now = true;
  // Endless loop
  while ((!stop_signal) && (!local_stop_signal)) {
    // Waiting for a new frame to be added to the buffer
    sem_wait(sync_object.cont_sync_sem);

    // Getting last buffer frame
    free5GRAN::buffer_element last_elem =
        (*rf_buff->frame_buffer)[rf_buff->frame_buffer->size() - 1];

    // If (last frame contains SSB) AND ((resynchronization period is over) OR
    // (specific demand for immediate resynchronization))
    if (abs((int)last_elem.frame_id - sync_object.sfn) %
                (int)(sync_object.ssb_period / 0.01) ==
            0 &&
        (abs((int)last_elem.frame_id - last_resync_frame_id) > resync_period ||
         resync_now)) {
      // Extract signal around expected PSS position with +/- sync_space samples
      vector<complex<float>> pss_symbols(symbol_duration + 2 * sync_space);
      for (int i = 0; i < pss_symbols.size(); i++) {
        pss_symbols[i] =
            last_elem
                .buffer[(frame_size + (sync_object.pss_index - sync_space + i) %
                                          frame_size) %
                        frame_size];
      }
      // Search PSS in extracted signal
      int n_id_2, n_id_1, synchronization_index;
      float peak_value, peak_value_sss;
      free5GRAN::phy::synchronization::search_pss(
          n_id_2, synchronization_index, peak_value,
          sync_object.common_cp_length, pss_symbols, sync_object.fft_size_ssb);
      // Compute PSS buffer position
      int pss_start_index = synchronization_index - symbol_duration + 1;
      int relative_index = pss_start_index - sync_space;
      int sss_init_index = sync_object.pss_index + relative_index +
                           2 * symbol_duration + sync_object.common_cp_length;
      vector<complex<float>> sss_signal(sync_object.fft_size_ssb);
      // Extract SSS based on PSS buffer position
      for (int i = 0; i < sync_object.fft_size_ssb; i++) {
        sss_signal[i] =
            last_elem.buffer[(frame_size + (i + sss_init_index) % frame_size) %
                             frame_size];
      }
      // Correlate SSS to extract N ID 1
      free5GRAN::phy::synchronization::get_sss(
          n_id_1, peak_value_sss, sss_signal, sync_object.fft_size_ssb, n_id_2);

      last_resync_frame_id = (int)last_elem.frame_id;
      // Deactivate resync_now
      if (resync_now) {
        resync_now = false;
      }
      // Check whether or not resynchronization has been possible.
      if (sync_object.pci == n_id_2 + 3 * n_id_1) {
        // resync has been possible
        // Modify the frame synchronization index based on new PSS index and
        // bind it to a frame
        sync_object.sync_index += relative_index;
        if (sync_object.sync_index < 0) {
          sync_object.sync_index += frame_size;
        } else if (sync_object.sync_index > frame_size - 1) {
          sync_object.sync_index -= frame_size;
        }
        // Reduce sync_space;
        sync_space = symbol_duration / 4;
      } else {
        cout << "#####################################" << endl;
        cout << "####          LOST SYNC          ####" << endl;
        cout << "#####################################" << endl;
        // Resynchronize over a full frame ( + / - (frame / 2)
        sync_space = frame_size / 2;
        resync_now = true;
      }
    }
  }
}
