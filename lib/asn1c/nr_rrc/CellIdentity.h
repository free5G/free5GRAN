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
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	`asn1c -gen-PER -fcompound-names -findirect-choice -no-gen-example`
 */

#ifndef	_CellIdentity_H_
#define	_CellIdentity_H_


#include "asn_application.h"

/* Including external dependencies */
#include "BIT_STRING.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CellIdentity */
typedef BIT_STRING_t	 CellIdentity_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_CellIdentity_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_CellIdentity;
asn_struct_free_f CellIdentity_free;
asn_struct_print_f CellIdentity_print;
asn_constr_check_f CellIdentity_constraint;
ber_type_decoder_f CellIdentity_decode_ber;
der_type_encoder_f CellIdentity_encode_der;
xer_type_decoder_f CellIdentity_decode_xer;
xer_type_encoder_f CellIdentity_encode_xer;
oer_type_decoder_f CellIdentity_decode_oer;
oer_type_encoder_f CellIdentity_encode_oer;
per_type_decoder_f CellIdentity_decode_uper;
per_type_encoder_f CellIdentity_encode_uper;
per_type_decoder_f CellIdentity_decode_aper;
per_type_encoder_f CellIdentity_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _CellIdentity_H_ */
#include "asn_internal.h"
