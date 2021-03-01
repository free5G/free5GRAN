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

#ifndef FREE5GRAN_USRP_B200_H
#define FREE5GRAN_USRP_B200_H

#include "../rf.h"
#include "../../variables/common_structures/common_structures.h"

namespace free5GRAN {
    class usrp_b200: public rf {
        public:
            usrp_b200(
                    double sample_rate,
                    double center_frequency,
                    double gain,
                    double bandwidth,
                    free5GRAN::rf_device chosen_device
                    );

            void get_samples(vector<complex<float>> &buff, double &time_first_sample);

            double getSampleRate();

            void setSampleRate(double rate);

            double getCenterFrequency();

            void setCenterFrequency(double freq);

            void setGain(double gain);

            double getGain();
    };
}

#endif //FREE5GRAN_USRP_B200_H
