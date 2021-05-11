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

#ifndef FREE5GRAN_USRP_USRP2_H
#define FREE5GRAN_USRP_USRP2_H

#include <semaphore.h>

#include "../../variables/common_structures/common_structures.h"
#include "../rf.h"

#include <liquid/liquid.h>

namespace free5GRAN {
class usrp_usrp2 : public rf {
 private:
  double internal_sampling_rate;
  double resampling_ratio;
  resamp_crcf resampling_filter;
  bool filter_init_done = false;

 public:
  usrp_usrp2(double sample_rate,
             double center_frequency,
             double gain,
             double bandwidth,
             free5GRAN::rf_device chosen_device,
             free5GRAN::rf_buffer* rf_buff);

  void get_samples(vector<complex<float>>& buff,
                   double& time_first_sample) override;

  auto getSampleRate() -> double override;

  void setSampleRate(double rate) override;

  auto getCenterFrequency() -> double override;

  void setCenterFrequency(double freq) override;

  void setGain(double gain) override;

  auto getGain() -> double override;

  void adjust_sampling_rate(double& rate);

  void start_loopback_recv(bool& stop_signal, size_t buff_size) override;

  void loopback_filter(vector<complex<float>>& tmp_buff,
                       sem_t& filtering_sem,
                       bool& stop_signal,
                       bool& overflow);
};
}  // namespace free5GRAN

#endif  // FREE5GRAN_USRP_USRP2_H
