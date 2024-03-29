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

#ifndef	_UECapabilityInformation_IEs_H_
#define	_UECapabilityInformation_IEs_H_


#include "asn_application.h"

/* Including external dependencies */
#include "OCTET_STRING.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct UE_CapabilityRAT_ContainerList;

/* UECapabilityInformation-IEs */
typedef struct UECapabilityInformation_IEs {
	struct UE_CapabilityRAT_ContainerList	*ue_CapabilityRAT_ContainerList;	/* OPTIONAL */
	OCTET_STRING_t	*lateNonCriticalExtension;	/* OPTIONAL */
	struct UECapabilityInformation_IEs__nonCriticalExtension {
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *nonCriticalExtension;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} UECapabilityInformation_IEs_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_UECapabilityInformation_IEs;
extern asn_SEQUENCE_specifics_t asn_SPC_UECapabilityInformation_IEs_specs_1;
extern asn_TYPE_member_t asn_MBR_UECapabilityInformation_IEs_1[3];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "UE-CapabilityRAT-ContainerList.h"

#endif	/* _UECapabilityInformation_IEs_H_ */
#include "asn_internal.h"
