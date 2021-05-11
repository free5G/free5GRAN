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
#ifndef	_BOOLEAN_H_
#define	_BOOLEAN_H_

#include "asn_application.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * The underlying integer may contain various values, but everything
 * non-zero is capped to 0xff by the DER encoder. The BER decoder may
 * yield non-zero values different from 1, beware.
 */
typedef int BOOLEAN_t;

extern asn_TYPE_descriptor_t asn_DEF_BOOLEAN;
extern asn_TYPE_operation_t asn_OP_BOOLEAN;

asn_struct_free_f BOOLEAN_free;
asn_struct_print_f BOOLEAN_print;
asn_struct_compare_f BOOLEAN_compare;
ber_type_decoder_f BOOLEAN_decode_ber;
der_type_encoder_f BOOLEAN_encode_der;
oer_type_decoder_f BOOLEAN_decode_oer;
oer_type_encoder_f BOOLEAN_encode_oer;
per_type_decoder_f BOOLEAN_decode_uper;
per_type_encoder_f BOOLEAN_encode_uper;
per_type_decoder_f BOOLEAN_decode_aper;
per_type_encoder_f BOOLEAN_encode_aper;
xer_type_decoder_f BOOLEAN_decode_xer;
xer_type_encoder_f BOOLEAN_encode_xer;
asn_random_fill_f  BOOLEAN_random_fill;

#define BOOLEAN_constraint     asn_generic_no_constraint

#ifdef __cplusplus
}
#endif

#endif	/* _BOOLEAN_H_ */
