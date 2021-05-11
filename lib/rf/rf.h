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

#ifndef FREE5GRAN_RF_H
#define FREE5GRAN_RF_H

#include <uhd.h>

#include <boost/circular_buffer.hpp>
#include <complex>
#include <condition_variable>
#include <string>
#include <uhd/usrp/multi_usrp.hpp>
#include <vector>

#include "../variables/common_structures/common_structures.h"

using namespace std;
namespace free5GRAN {
class rf {
  /*
   * rf class implements the exchanges between NRPhy and USRP based on UHD lib
   * (https://files.ettus.com/manual/index.html).
   */

 protected:
  double sample_rate;                //< RF device sampling rate
  double center_frequency;           //< Cell center frequency
  double gain;                       //< RF device gain
  double bandwidth;                  //< RF device input/output filter bandwidth
  uhd::usrp::multi_usrp::sptr usrp;  //< RF device pointer
  size_t primary_frame_id = 0, frame_id = 0;  //< frame ID
  free5GRAN::rf_buffer* rf_buff;              //< Shared buffer

 public:
  rf() = default;
  ;

  /*
   * Get samples from RF device
   * buff is the output buffer
   * time_first_sample is the time of the first received sample
   */
  virtual void get_samples(vector<complex<float>>& buff,
                           double& time_first_sample) = 0;

  virtual auto getSampleRate() -> double = 0;

  virtual void setSampleRate(double rate) = 0;

  virtual auto getCenterFrequency() -> double = 0;

  virtual void setCenterFrequency(double freq) = 0;

  virtual void setGain(double gain) = 0;

  virtual auto getGain() -> double = 0;

  virtual void start_loopback_recv(bool& stop_signal, size_t buff_size) = 0;

  void adjust_frames(free5GRAN::synchronization_object& sync_object,
                     size_t buff_size,
                     bool& stop_signal,
                     bool& local_stop_signal);

  void resynchronization(free5GRAN::synchronization_object& sync_object,
                         bool& stop_signal,
                         bool& local_stop_signal);

  auto get_frame_with_parity(int parity) -> free5GRAN::buffer_element;
  void get_n_frame(vector<free5GRAN::buffer_element>& frames,
                   int n,
                   bool& timeout);

  auto getPrimaryFrameId() -> size_t { return this->primary_frame_id; }
};
}  // namespace free5GRAN

#endif  // FREE5GRAN_RF_H
