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
 * Copyright (c) 2004, 2007 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * Redistribution and modifications are permitted subject to BSD license.
 */
/*
 * This type differs from the standard INTEGER in that it is modelled using
 * the fixed machine type (long, int, short), so it can hold only values of
 * limited length. There is no type (i.e., NativeInteger_t, any integer type
 * will do).
 * This type may be used when integer range is limited by subtype constraints.
 */
#ifndef	_NativeInteger_H_
#define	_NativeInteger_H_

#include "asn_application.h"
#include "INTEGER.h"

#ifdef __cplusplus
extern "C" {
#endif

extern asn_TYPE_descriptor_t asn_DEF_NativeInteger;
extern asn_TYPE_operation_t asn_OP_NativeInteger;

asn_struct_free_f  NativeInteger_free;
asn_struct_print_f NativeInteger_print;
asn_struct_compare_f NativeInteger_compare;
ber_type_decoder_f NativeInteger_decode_ber;
der_type_encoder_f NativeInteger_encode_der;
xer_type_decoder_f NativeInteger_decode_xer;
xer_type_encoder_f NativeInteger_encode_xer;
oer_type_decoder_f NativeInteger_decode_oer;
oer_type_encoder_f NativeInteger_encode_oer;
per_type_decoder_f NativeInteger_decode_uper;
per_type_encoder_f NativeInteger_encode_uper;
per_type_decoder_f NativeInteger_decode_aper;
per_type_encoder_f NativeInteger_encode_aper;
asn_random_fill_f  NativeInteger_random_fill;

#define NativeInteger_constraint  asn_generic_no_constraint

#ifdef __cplusplus
}
#endif

#endif	/* _NativeInteger_H_ */
