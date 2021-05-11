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
/*-
 * Copyright (c) 2003-2017 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * Redistribution and modifications are permitted subject to BSD license.
 */
#ifndef	ASN_TYPE_NULL_H
#define	ASN_TYPE_NULL_H

#include "asn_application.h"
#include "BOOLEAN.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * The value of the NULL type is meaningless: see BOOLEAN if you want to
 * carry true/false semantics.
 */
typedef int NULL_t;

extern asn_TYPE_descriptor_t asn_DEF_NULL;
extern asn_TYPE_operation_t asn_OP_NULL;

asn_struct_print_f NULL_print;
asn_struct_compare_f NULL_compare;
der_type_encoder_f NULL_encode_der;
xer_type_decoder_f NULL_decode_xer;
xer_type_encoder_f NULL_encode_xer;
oer_type_decoder_f NULL_decode_oer;
oer_type_encoder_f NULL_encode_oer;
per_type_decoder_f NULL_decode_uper;
per_type_encoder_f NULL_encode_uper;
per_type_decoder_f NULL_decode_aper;
per_type_encoder_f NULL_encode_aper;
asn_random_fill_f  NULL_random_fill;

#define NULL_free	BOOLEAN_free
#define NULL_decode_ber	BOOLEAN_decode_ber
#define NULL_constraint	asn_generic_no_constraint

#ifdef __cplusplus
}
#endif

#endif	/* NULL_H */
