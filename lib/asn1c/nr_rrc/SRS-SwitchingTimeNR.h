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

#ifndef	_SRS_SwitchingTimeNR_H_
#define	_SRS_SwitchingTimeNR_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeEnumerated.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SRS_SwitchingTimeNR__switchingTimeDL {
	SRS_SwitchingTimeNR__switchingTimeDL_n0us	= 0,
	SRS_SwitchingTimeNR__switchingTimeDL_n30us	= 1,
	SRS_SwitchingTimeNR__switchingTimeDL_n100us	= 2,
	SRS_SwitchingTimeNR__switchingTimeDL_n140us	= 3,
	SRS_SwitchingTimeNR__switchingTimeDL_n200us	= 4,
	SRS_SwitchingTimeNR__switchingTimeDL_n300us	= 5,
	SRS_SwitchingTimeNR__switchingTimeDL_n500us	= 6,
	SRS_SwitchingTimeNR__switchingTimeDL_n900us	= 7
} e_SRS_SwitchingTimeNR__switchingTimeDL;
typedef enum SRS_SwitchingTimeNR__switchingTimeUL {
	SRS_SwitchingTimeNR__switchingTimeUL_n0us	= 0,
	SRS_SwitchingTimeNR__switchingTimeUL_n30us	= 1,
	SRS_SwitchingTimeNR__switchingTimeUL_n100us	= 2,
	SRS_SwitchingTimeNR__switchingTimeUL_n140us	= 3,
	SRS_SwitchingTimeNR__switchingTimeUL_n200us	= 4,
	SRS_SwitchingTimeNR__switchingTimeUL_n300us	= 5,
	SRS_SwitchingTimeNR__switchingTimeUL_n500us	= 6,
	SRS_SwitchingTimeNR__switchingTimeUL_n900us	= 7
} e_SRS_SwitchingTimeNR__switchingTimeUL;

/* SRS-SwitchingTimeNR */
typedef struct SRS_SwitchingTimeNR {
	long	*switchingTimeDL;	/* OPTIONAL */
	long	*switchingTimeUL;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SRS_SwitchingTimeNR_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_switchingTimeDL_2;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_switchingTimeUL_11;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_SRS_SwitchingTimeNR;
extern asn_SEQUENCE_specifics_t asn_SPC_SRS_SwitchingTimeNR_specs_1;
extern asn_TYPE_member_t asn_MBR_SRS_SwitchingTimeNR_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _SRS_SwitchingTimeNR_H_ */
#include "asn_internal.h"
