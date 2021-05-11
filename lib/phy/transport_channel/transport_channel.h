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

#include "../../asn1c/nr_rrc/BCCH-DL-SCH-Message.h"
#include "../../variables/common_structures/common_structures.h"
using namespace std;

namespace free5GRAN::phy::transport_channel {

auto compute_N_polar_code(int E, int K, int nmax) -> int;

void rate_recover(int* input_bits,
                  int* output_bits,
                  int i_bil,
                  int E,
                  int N,
                  int K);

void polar_decode(int* input_bits,
                  int* output_bits,
                  int N,
                  int K,
                  int nmax,
                  int i_il,
                  int n_pc,
                  int n_wm_pc,
                  int E);

void crc_validate(int* input_bits,
                  int* crc_polynom,
                  int* remainder,
                  int length_input,
                  int length_crc);

void compute_crc(int* input_bits,
                 int* crc_polynom,
                 int* remainder,
                 int length_input,
                 int length_crc);

void compute_ldpc_base_graph(int A, float R, int& graph);

void compute_transport_block_size(int n_re,
                                  float R,
                                  int mod_order,
                                  int num_layers,
                                  int nrb,
                                  int& tbs);

void compute_Zc_dl_sch(int kb, float k_p, int& Zc, int& i_ls);

void compute_code_block_segmentation_info_ldpc(int graph,
                                               int B,
                                               int& Zc,
                                               int& K,
                                               int& i_ls,
                                               int& L,
                                               int& C,
                                               int& N,
                                               int& K_p);

void rate_recover_ldpc(int* input_bits,
                       int N,
                       int i_lbrm,
                       int E,
                       int id_rv,
                       int mod_order,
                       int C,
                       int Zc,
                       int graph,
                       int K,
                       int K_p,
                       int* output_sequence);

void rate_recover_ldpc(double* input_bits,
                       int N,
                       int i_lbrm,
                       int E,
                       int id_rv,
                       int mod_order,
                       int C,
                       int Zc,
                       int graph,
                       int K,
                       int K_p,
                       double* output_sequence);

void compute_circular_permutation_matrix(int size, int offset, int** matrix);

void ldpc_decode_one_bit(vector<vector<int>> R,
                         double* soft_bits,
                         int i,
                         double& new_bit);

void compute_H_matrix_ldpc(int Zc,
                           int graph,
                           int i_ls,
                           vector<vector<int>>& matrix,
                           int& size_i,
                           int& size_j);

void ldpc_decode(double* input_bits,
                 int N,
                 int Zc,
                 int graph,
                 int K,
                 int i_ls,
                 int* output_sequence);

void decode_bch(int* bch_bits, bool& crc_validated, int* mib_bits, int pci);

void decode_dci(int* dci_bits,
                int E,
                int K,
                int* rnti,
                bool& validated,
                vector<int>& decoded_dci_bits);

auto decode_dl_sch(double* dl_sch_bits,
                   int n_re,
                   float R,
                   int nrb,
                   int E,
                   bool& validated,
                   free5GRAN::dci_1_0_si_rnti dci_1_0_si_rnti) -> vector<int>;

}  // namespace free5GRAN::phy::transport_channel