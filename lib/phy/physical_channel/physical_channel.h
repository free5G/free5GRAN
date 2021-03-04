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
#include "../../variables/common_structures/common_structures.h"
using namespace std;

namespace free5GRAN {
    namespace phy {
        namespace physical_channel {
            void decode_pdcch(vector<complex<float>> pdcch_symbols,int *dci_bits, int agg_level, int* reg_index, int* reg_index_sorted, int pci);

            void decode_pdsch(vector<complex<float>> pdsch_samples, double *unscrambled_soft_bits, int pci);

            void decode_pbch(vector<complex<float>> pbch_symbols, int i_ssb, int pci, int *bch_bits);

            void compute_pbch_indexes(vector<vector<vector<int>>> &ref, int pci);

            void compute_pdcch_indexes(vector<vector<vector<int>>> &ref, free5GRAN::pdcch_t0ss_monitoring_occasions pdcch_ss_mon_occ, int agg_level, int *reg_bundles, int height_reg_rb);

            void compute_pdsch_indexes(vector<vector<vector<int>>> &ref, bool dmrs_symbol_array[], int L, int lrb);


            /** FROM HERE, IT'S ADDITION FROM BENOIT. BE CAREFUL WHEN MERGING */
            void pbch_encoding(vector<int> rate_matched_bch_vector, int pci, int i_b_ssb, vector<complex<float>> &pbch_symbols_vector);
            void pdcch_encoding(free5GRAN::dci_1_0_si_rnti dci_object, int freq_domain_ra_size, int n_rb_coreset, int length_crc, int *rnti, int agg_level, int n, vector<complex<float>> &pdcch_symbols);
            }
    }
}
