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

class rf {
    /*
     * rf class implements the exchanges between NRPhy and USRP based on UHD lib (https://files.ettus.com/manual/index.html).
     */

private:
    double sample_rate;
    double center_frequency;
    double gain;
    double bandwidth;
    string device_args;
    string subdev;
    string antenna_mode;
    string ref;
    uhd::usrp::multi_usrp::sptr usrp;

public:
    rf(
            double sample_rate,
            double center_frequency,
            double gain,
            double bandwidth,
            const string &subdev,
            const string &antenna_mode,
            const string &ref,
            const string &device_args);

    /*
     * Get samples from RF device
     * buff is the output buffer
     * time_first_sample is the time of the first received sample
     */
    void get_samples(vector<complex<float>> *buff, double &time_first_sample);

    double getSampleRate() const;

    void setSampleRate(double rate);

    double getCenterFrequency();

    void setCenterFrequency(double freq);

    void setGain(double gain);

    double getGain();


};


#endif
