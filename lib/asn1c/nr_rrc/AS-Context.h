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

#ifndef	_AS_Context_H_
#define	_AS_Context_H_


#include "asn_application.h"

/* Including external dependencies */
#include "constr_SEQUENCE.h"
#include "OCTET_STRING.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ReestablishmentInfo;
struct ConfigRestrictInfoSCG;
struct RAN_NotificationAreaInfo;
struct BandCombinationInfoSN;

/* AS-Context */
typedef struct AS_Context {
	struct ReestablishmentInfo	*reestablishmentInfo;	/* OPTIONAL */
	struct ConfigRestrictInfoSCG	*configRestrictInfo;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct AS_Context__ext1 {
		struct RAN_NotificationAreaInfo	*ran_NotificationAreaInfo;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	struct AS_Context__ext2 {
		OCTET_STRING_t	*ueAssistanceInformation;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext2;
	struct AS_Context__ext3 {
		struct BandCombinationInfoSN	*selectedBandCombinationSN;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext3;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} AS_Context_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_AS_Context;
extern asn_SEQUENCE_specifics_t asn_SPC_AS_Context_specs_1;
extern asn_TYPE_member_t asn_MBR_AS_Context_1[5];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "ReestablishmentInfo.h"
#include "ConfigRestrictInfoSCG.h"
#include "RAN-NotificationAreaInfo.h"
#include "BandCombinationInfoSN.h"

#endif	/* _AS_Context_H_ */
#include "asn_internal.h"
