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

#include "../../variables/common_structures/common_structures.h"

namespace free5GRAN {
    namespace utils {
        namespace common_utils {
            void parse_mib(int *mib_bits, free5GRAN::mib &mib_object);

            void scramble(int * input_bits, int * c_seq, int * output_bits, int length, int offset);

            void scramble(double * input_bits, int * c_seq, double * output_bits, int length, int offset);

            /** FROM HERE, IT'S ADDITION FROM BENOIT. BE CAREFUL WHEN MERGING */

            void encode_mib(free5GRAN::mib mib_object, int *mib_bits);

            void convert_decimal_to_binary(int size, int decimal, int* table_output);
        }
    }
}
