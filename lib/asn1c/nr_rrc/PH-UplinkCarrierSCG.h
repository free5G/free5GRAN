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

#ifndef	_PH_UplinkCarrierSCG_H_
#define	_PH_UplinkCarrierSCG_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeEnumerated.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum PH_UplinkCarrierSCG__ph_Type1or3 {
	PH_UplinkCarrierSCG__ph_Type1or3_type1	= 0,
	PH_UplinkCarrierSCG__ph_Type1or3_type3	= 1
} e_PH_UplinkCarrierSCG__ph_Type1or3;

/* PH-UplinkCarrierSCG */
typedef struct PH_UplinkCarrierSCG {
	long	 ph_Type1or3;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} PH_UplinkCarrierSCG_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_ph_Type1or3_2;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_PH_UplinkCarrierSCG;
extern asn_SEQUENCE_specifics_t asn_SPC_PH_UplinkCarrierSCG_specs_1;
extern asn_TYPE_member_t asn_MBR_PH_UplinkCarrierSCG_1[1];

#ifdef __cplusplus
}
#endif

#endif	/* _PH_UplinkCarrierSCG_H_ */
#include "asn_internal.h"
