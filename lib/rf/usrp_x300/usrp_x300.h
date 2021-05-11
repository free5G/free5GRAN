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

#ifndef FREE5GRAN_USRP_X300_H
#define FREE5GRAN_USRP_X300_H

#include "../../variables/common_structures/common_structures.h"
#include "../rf.h"

namespace free5GRAN {
class usrp_x300 : public rf {
 public:
  usrp_x300(double sample_rate,
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

  void start_loopback_recv(bool& stop_signal, size_t buff_size) override;
};
}  // namespace free5GRAN

#endif  // FREE5GRAN_USRP_X300_H
