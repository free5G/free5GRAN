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
#ifndef	OER_DECODER_H
#define	OER_DECODER_H

#include "asn_application.h"
#include "oer_support.h"

#ifdef __cplusplus
extern "C" {
#endif

struct asn_TYPE_descriptor_s;	/* Forward declaration */
struct asn_codec_ctx_s;		/* Forward declaration */

/*
 * The Octet Encoding Rules (OER, X.696 08/2015) decoder for any given type.
 * This function may be invoked directly by the application.
 * Parses CANONICAL-OER and BASIC-OER.
 */
asn_dec_rval_t oer_decode(const struct asn_codec_ctx_s *opt_codec_ctx,
	const struct asn_TYPE_descriptor_s *type_descriptor,
	void **struct_ptr,	/* Pointer to a target structure's pointer */
	const void *buffer,	/* Data to be decoded */
	size_t size		/* Size of that buffer */
	);

/*
 * Type of generic function which decodes the byte stream into the structure.
 */
typedef asn_dec_rval_t(oer_type_decoder_f)(
    const struct asn_codec_ctx_s *opt_codec_ctx,
    const struct asn_TYPE_descriptor_s *type_descriptor,
    const asn_oer_constraints_t *constraints,
    void **struct_ptr,
    const void *buf_ptr,
    size_t size);

/*
 * Swallow the Open Type (X.696 (08/2015), #30) into /dev/null.
 * RETURN VALUES:
 *      -1:     Fatal error deciphering length.
 *       0:     More data expected than bufptr contains.
 *      >0:     Number of bytes used from bufptr.
 */
ssize_t oer_open_type_skip(const void *bufptr, size_t size);

/*
 * Read the Open Type (X.696 (08/2015), #30).
 * RETURN VALUES:
 *       0:     More data expected than bufptr contains.
 *      -1:     Fatal error deciphering length.
 *      >0:     Number of bytes used from bufptr.
 */
ssize_t oer_open_type_get(const asn_codec_ctx_t *opt_codec_ctx,
                          const struct asn_TYPE_descriptor_s *td,
                          const asn_oer_constraints_t *constraints,
                          void **struct_ptr, const void *bufptr, size_t size);

/*
 * Length-prefixed buffer decoding for primitive types.
 */
oer_type_decoder_f oer_decode_primitive;


#ifdef __cplusplus
}
#endif

#endif	/* OER_DECODER_H */
