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

namespace free5GRAN::utils::sequence_generator {

void generate_pss_sequence(int n_id_2, int* output_sequence);

void generate_sss_sequence(int n_id_1, int n_id_2, int* output_sequence);

void generate_pbch_dmrs_sequence(int pci,
                                 int i_bar_ssb,
                                 complex<float>* output_sequence);

void generate_c_sequence(long c_init,
                         int length,
                         int* output_sequence,
                         int demod_type);

void generate_pdcch_dmrs_sequence(int nid,
                                  int slot_number,
                                  int symbol_number,
                                  complex<float>* output_sequence,
                                  int size);

void generate_pdcch_dmrs_sequence(int nid,
                                  int slot_number,
                                  int symbol_number,
                                  vector<complex<float>>& output_sequence,
                                  int size);

void generate_pdsch_dmrs_sequence(int n_symb_slot,
                                  int slot_number,
                                  int symbol_number,
                                  int n_scid,
                                  int n_id_scid,
                                  complex<float>* output_sequence,
                                  int size);

}  // namespace free5GRAN::utils::sequence_generator
