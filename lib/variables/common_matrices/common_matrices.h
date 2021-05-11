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

#ifndef FREE5GRAN_COMMON_MATRICES_H
#define FREE5GRAN_COMMON_MATRICES_H

namespace free5GRAN {

/*
 * Gn matrix and inverse matrix (TS38.212 5.3.1.2)
 */

extern int G5[32][32];

extern int G5_INV[32][32];

extern int G6[64][64];

extern int G6_INV[64][64];

extern int G7[128][128];

extern int G7_INV[128][128];

extern int G8[256][256];

extern int G8_INV[256][256];

extern int G9[512][512];

extern int G9_INV[512][512];

extern int G10[1024][1024];

extern int G10_INV[1024][1024];

}  // namespace free5GRAN

#endif
