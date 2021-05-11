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
#ifndef	_CONSTR_SEQUENCE_OF_H_
#define	_CONSTR_SEQUENCE_OF_H_

#include "asn_application.h"
#include "constr_SET_OF.h"		/* Implemented using SET OF */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * A set specialized functions dealing with the SEQUENCE OF type.
 * Generally implemented using SET OF.
 */
asn_struct_compare_f SEQUENCE_OF_compare;
der_type_encoder_f SEQUENCE_OF_encode_der;
xer_type_encoder_f SEQUENCE_OF_encode_xer;
per_type_encoder_f SEQUENCE_OF_encode_uper;
per_type_encoder_f SEQUENCE_OF_encode_aper;
extern asn_TYPE_operation_t asn_OP_SEQUENCE_OF;

#define	SEQUENCE_OF_free	SET_OF_free
#define	SEQUENCE_OF_print	SET_OF_print
#define	SEQUENCE_OF_constraint	SET_OF_constraint
#define	SEQUENCE_OF_decode_ber	SET_OF_decode_ber
#define	SEQUENCE_OF_decode_xer	SET_OF_decode_xer
#define	SEQUENCE_OF_decode_oer  SET_OF_decode_oer
#define	SEQUENCE_OF_encode_oer  SET_OF_encode_oer
#define	SEQUENCE_OF_decode_uper	SET_OF_decode_uper
#define	SEQUENCE_OF_decode_aper	SET_OF_decode_aper
#define	SEQUENCE_OF_random_fill SET_OF_random_fill

#ifdef __cplusplus
}
#endif

#endif	/* _CONSTR_SET_OF_H_ */
