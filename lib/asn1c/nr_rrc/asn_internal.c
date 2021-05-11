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
#include "asn_internal.h"

ssize_t
asn__format_to_callback(int (*cb)(const void *, size_t, void *key), void *key,
                        const char *fmt, ...) {
    char scratch[64];
    char *buf = scratch;
    size_t buf_size = sizeof(scratch);
    int wrote;
    int cb_ret;

    do {
        va_list args;
        va_start(args, fmt);

        wrote = vsnprintf(buf, buf_size, fmt, args);
        va_end(args);
        if(wrote < (ssize_t)buf_size) {
            if(wrote < 0) {
                if(buf != scratch) FREEMEM(buf);
                return -1;
            }
            break;
        }

        buf_size <<= 1;
        if(buf == scratch) {
            buf = MALLOC(buf_size);
            if(!buf) return -1;
        } else {
            void *p = REALLOC(buf, buf_size);
            if(!p) {
                FREEMEM(buf);
                return -1;
            }
            buf = p;
        }
    } while(1);

    cb_ret = cb(buf, wrote, key);
    if(buf != scratch) FREEMEM(buf);
    if(cb_ret < 0) {
        return -1;
    }

    return wrote;
}

