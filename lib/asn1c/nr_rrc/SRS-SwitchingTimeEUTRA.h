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

#ifndef	_SRS_SwitchingTimeEUTRA_H_
#define	_SRS_SwitchingTimeEUTRA_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeEnumerated.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SRS_SwitchingTimeEUTRA__switchingTimeDL {
	SRS_SwitchingTimeEUTRA__switchingTimeDL_n0	= 0,
	SRS_SwitchingTimeEUTRA__switchingTimeDL_n0dot5	= 1,
	SRS_SwitchingTimeEUTRA__switchingTimeDL_n1	= 2,
	SRS_SwitchingTimeEUTRA__switchingTimeDL_n1dot5	= 3,
	SRS_SwitchingTimeEUTRA__switchingTimeDL_n2	= 4,
	SRS_SwitchingTimeEUTRA__switchingTimeDL_n2dot5	= 5,
	SRS_SwitchingTimeEUTRA__switchingTimeDL_n3	= 6,
	SRS_SwitchingTimeEUTRA__switchingTimeDL_n3dot5	= 7,
	SRS_SwitchingTimeEUTRA__switchingTimeDL_n4	= 8,
	SRS_SwitchingTimeEUTRA__switchingTimeDL_n4dot5	= 9,
	SRS_SwitchingTimeEUTRA__switchingTimeDL_n5	= 10,
	SRS_SwitchingTimeEUTRA__switchingTimeDL_n5dot5	= 11,
	SRS_SwitchingTimeEUTRA__switchingTimeDL_n6	= 12,
	SRS_SwitchingTimeEUTRA__switchingTimeDL_n6dot5	= 13,
	SRS_SwitchingTimeEUTRA__switchingTimeDL_n7	= 14
} e_SRS_SwitchingTimeEUTRA__switchingTimeDL;
typedef enum SRS_SwitchingTimeEUTRA__switchingTimeUL {
	SRS_SwitchingTimeEUTRA__switchingTimeUL_n0	= 0,
	SRS_SwitchingTimeEUTRA__switchingTimeUL_n0dot5	= 1,
	SRS_SwitchingTimeEUTRA__switchingTimeUL_n1	= 2,
	SRS_SwitchingTimeEUTRA__switchingTimeUL_n1dot5	= 3,
	SRS_SwitchingTimeEUTRA__switchingTimeUL_n2	= 4,
	SRS_SwitchingTimeEUTRA__switchingTimeUL_n2dot5	= 5,
	SRS_SwitchingTimeEUTRA__switchingTimeUL_n3	= 6,
	SRS_SwitchingTimeEUTRA__switchingTimeUL_n3dot5	= 7,
	SRS_SwitchingTimeEUTRA__switchingTimeUL_n4	= 8,
	SRS_SwitchingTimeEUTRA__switchingTimeUL_n4dot5	= 9,
	SRS_SwitchingTimeEUTRA__switchingTimeUL_n5	= 10,
	SRS_SwitchingTimeEUTRA__switchingTimeUL_n5dot5	= 11,
	SRS_SwitchingTimeEUTRA__switchingTimeUL_n6	= 12,
	SRS_SwitchingTimeEUTRA__switchingTimeUL_n6dot5	= 13,
	SRS_SwitchingTimeEUTRA__switchingTimeUL_n7	= 14
} e_SRS_SwitchingTimeEUTRA__switchingTimeUL;

/* SRS-SwitchingTimeEUTRA */
typedef struct SRS_SwitchingTimeEUTRA {
	long	*switchingTimeDL;	/* OPTIONAL */
	long	*switchingTimeUL;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SRS_SwitchingTimeEUTRA_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_switchingTimeDL_2;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_switchingTimeUL_18;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_SRS_SwitchingTimeEUTRA;
extern asn_SEQUENCE_specifics_t asn_SPC_SRS_SwitchingTimeEUTRA_specs_1;
extern asn_TYPE_member_t asn_MBR_SRS_SwitchingTimeEUTRA_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _SRS_SwitchingTimeEUTRA_H_ */
#include "asn_internal.h"
