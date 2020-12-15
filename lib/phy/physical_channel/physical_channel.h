/*
 * Copyright 2020 Telecom Paris

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
        namespace physical_channel {
            void decode_pdcch(vector<complex<float>> pdcch_symbols,int *dci_bits, int agg_level, int* reg_index, int* reg_index_sorted, int pci);

            void decode_pdsch(vector<complex<float>> pdsch_samples, double *unscrambled_soft_bits, int pci);

            void decode_pbch(vector<complex<float>> pbch_symbols, int i_ssb, int pci, int *bch_bits);
        }
    }
}
