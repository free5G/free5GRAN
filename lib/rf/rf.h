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

#include <string>
#include <uhd.h>
#include <uhd/usrp/multi_usrp.hpp>
#include <complex>

using namespace std;
namespace free5GRAN {
    class rf {
        /*
     * rf class implements the exchanges between NRPhy and USRP based on UHD lib (https://files.ettus.com/manual/index.html).
     */

    protected:
        double sample_rate;
        double center_frequency;
        double gain;
        double bandwidth;
        uhd::usrp::multi_usrp::sptr usrp;

    public:
        rf(){};

        /*
         * Get samples from RF device
         * buff is the output buffer
         * time_first_sample is the time of the first received sample
         */
        virtual void get_samples(vector<complex<float>> &buff, double &time_first_sample) = 0;

        virtual double getSampleRate() = 0;

        virtual void setSampleRate(double rate) = 0;

        virtual double getCenterFrequency() = 0;

        virtual void setCenterFrequency(double freq) = 0;

        virtual void setGain(double gain) = 0;

        virtual double getGain() = 0;

    };
}



#endif //FREE5GRAN_RF_H
