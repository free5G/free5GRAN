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

#ifndef	_CG_ConfigInfo_IEs_H_
#define	_CG_ConfigInfo_IEs_H_


#include "asn_application.h"

/* Including external dependencies */
#include "OCTET_STRING.h"
#include "NativeEnumerated.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum CG_ConfigInfo_IEs__scgFailureInfo__failureType {
	CG_ConfigInfo_IEs__scgFailureInfo__failureType_t310_Expiry	= 0,
	CG_ConfigInfo_IEs__scgFailureInfo__failureType_randomAccessProblem	= 1,
	CG_ConfigInfo_IEs__scgFailureInfo__failureType_rlc_MaxNumRetx	= 2,
	CG_ConfigInfo_IEs__scgFailureInfo__failureType_synchReconfigFailure_SCG	= 3,
	CG_ConfigInfo_IEs__scgFailureInfo__failureType_scg_reconfigFailure	= 4,
	CG_ConfigInfo_IEs__scgFailureInfo__failureType_srb3_IntegrityFailure	= 5
} e_CG_ConfigInfo_IEs__scgFailureInfo__failureType;

/* Forward declarations */
struct MeasResultList2NR;
struct MeasResultCellListSFTD_NR;
struct ConfigRestrictInfoSCG;
struct DRX_Info;
struct MeasConfigMN;
struct MRDC_AssistanceInfo;
struct CG_ConfigInfo_v1540_IEs;

/* CG-ConfigInfo-IEs */
typedef struct CG_ConfigInfo_IEs {
	OCTET_STRING_t	*ue_CapabilityInfo;	/* OPTIONAL */
	struct MeasResultList2NR	*candidateCellInfoListMN;	/* OPTIONAL */
	OCTET_STRING_t	*candidateCellInfoListSN;	/* OPTIONAL */
	struct MeasResultCellListSFTD_NR	*measResultCellListSFTD_NR;	/* OPTIONAL */
	struct CG_ConfigInfo_IEs__scgFailureInfo {
		long	 failureType;
		OCTET_STRING_t	 measResultSCG;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *scgFailureInfo;
	struct ConfigRestrictInfoSCG	*configRestrictInfo;	/* OPTIONAL */
	struct DRX_Info	*drx_InfoMCG;	/* OPTIONAL */
	struct MeasConfigMN	*measConfigMN;	/* OPTIONAL */
	OCTET_STRING_t	*sourceConfigSCG;	/* OPTIONAL */
	OCTET_STRING_t	*scg_RB_Config;	/* OPTIONAL */
	OCTET_STRING_t	*mcg_RB_Config;	/* OPTIONAL */
	struct MRDC_AssistanceInfo	*mrdc_AssistanceInfo;	/* OPTIONAL */
	struct CG_ConfigInfo_v1540_IEs	*nonCriticalExtension;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CG_ConfigInfo_IEs_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_failureType_7;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_CG_ConfigInfo_IEs;
extern asn_SEQUENCE_specifics_t asn_SPC_CG_ConfigInfo_IEs_specs_1;
extern asn_TYPE_member_t asn_MBR_CG_ConfigInfo_IEs_1[13];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "MeasResultList2NR.h"
#include "MeasResultCellListSFTD-NR.h"
#include "ConfigRestrictInfoSCG.h"
#include "DRX-Info.h"
#include "MeasConfigMN.h"
#include "MRDC-AssistanceInfo.h"
#include "CG-ConfigInfo-v1540-IEs.h"

#endif	/* _CG_ConfigInfo_IEs_H_ */
#include "asn_internal.h"
