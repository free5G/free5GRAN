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

#include "../../variables/common_structures/common_structures.h"

/** BELOW ARE INCLUDE FROM BENOIT. BE CAREFUL WHEN MERGING */

#include <complex>
#include <vector>

namespace free5GRAN {
    namespace utils {
        namespace common_utils {
            void parse_mib(int *mib_bits, free5GRAN::mib &mib_object);

            void scramble(int * input_bits, int * c_seq, int * output_bits, int length, int offset);

            void scramble(double * input_bits, int * c_seq, double * output_bits, int length, int offset);





            /** FROM HERE, IT'S ADDITION FROM BENOIT. BE CAREFUL WHEN MERGING */

            void scramble(std::vector<int> input_bits, int * c_seq, std::vector<int> &output_bits, int length, int offset);
            void encode_mib(free5GRAN::mib mib_object, int *mib_bits);
            void dci_generation(free5GRAN::dci_1_0_si_rnti dci_object, int *dci_bits, int freq_domain_ra_size);
            void convert_decimal_to_binary(int size, int decimal, int* table_output);
            void display_signal_float(std::complex<float> ** signal_to_display, int num_symbols, int num_sc, char* signal_name);
            void display_vector_2D(std::vector<std::vector<std::complex<float>>> vector_to_display, int vector_size1, int vector_size2, int line_break, char *vector_name);
            void display_vector_2D_int(std::vector<std::vector<int>> vector_to_display, int vector_size1, int vector_size2, int line_break, char *vector_name);
            void display_vector_per_symbols(std::vector<std::complex<float>> vector_to_display, int size1, char* vector_name);
            void display_vector(std::vector<std::complex<float>> vector_to_display, int size, char *vector_name);
            void display_vector(std::vector<int> vector_to_display, int size1, char *vector_name);
            void display_complex_double(std::complex<double> *table_to_display, int vector_size, char* vector_name);
            void display_complex_float(std::complex<float> *table_to_display, int vector_size, char* vector_name);
            void display_table(int* table_to_display, int size, char* table_name);
            void read_config_gNodeB(const char config_file[]);
            void init_logging(std::string level);
            void compute_num_sample_per_frame(free5GRAN::mib mib_object, int &Num_samples_in_frame);
        }
    }
}
