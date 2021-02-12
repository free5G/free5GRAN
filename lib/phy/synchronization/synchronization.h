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

#include <complex>
#include <vector>
using namespace std;

namespace free5GRAN {
    namespace phy {
        namespace synchronization {
            void search_pss(int &n_id_2, int &synchronisation_index, float &peak_value, int cp_length, vector<complex<float>> &buff, int fft_size);

            void cross_correlation(vector<complex<float>> in1, vector<complex<float>> in2, complex<float>* out, int size1,  int size2);

            void get_sss(int &n_id_1, float &peak_value, vector<complex<float>> &buff, int fft_size, int n_id_2);

            complex<float> correlate(vector<complex<float>> in1, int* in2, int size);

        };
    }

}

