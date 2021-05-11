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
/*
 * Copyright (c) 2017 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * Redistribution and modifications are permitted subject to BSD license.
 */
#ifndef	OER_SUPPORT_H
#define	OER_SUPPORT_H

#include "asn_system.h"		/* Platform-specific types */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Pre-computed OER constraints.
 */
typedef struct asn_oer_constraint_number_s {
    unsigned width;    /* ±8,4,2,1 fixed bytes */
    unsigned positive; /* 1 for unsigned number, 0 for signed */
} asn_oer_constraint_number_t;
typedef struct asn_oer_constraints_s {
    asn_oer_constraint_number_t value;
    ssize_t size;    /* -1 (no constraint) or >= 0 */
} asn_oer_constraints_t;


/*
 * Fetch the length determinant (X.696 (08/2015), #8.6) into *len_r.
 * RETURN VALUES:
 *       0:     More data expected than bufptr contains.
 *      -1:     Fatal error deciphering length.
 *      >0:     Number of bytes used from bufptr.
 */
ssize_t oer_fetch_length(const void *bufptr, size_t size, size_t *len_r);

/*
 * Serialize OER length. Returns the number of bytes serialized
 * or -1 if a given callback returned with negative result.
 */
ssize_t oer_serialize_length(size_t length, asn_app_consume_bytes_f *cb, void *app_key);


#ifdef __cplusplus
}
#endif

#endif	/* OER_SUPPORT_H */
