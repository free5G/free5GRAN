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

#ifndef	_CG_ConfigInfo_v1540_IEs_H_
#define	_CG_ConfigInfo_v1540_IEs_H_


#include "asn_application.h"

/* Including external dependencies */
#include "ARFCN-ValueNR.h"
#include "PhysCellId.h"
#include "CGI-InfoNR.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct PH_TypeListMCG;
struct CG_ConfigInfo_v1560_IEs;

/* CG-ConfigInfo-v1540-IEs */
typedef struct CG_ConfigInfo_v1540_IEs {
	struct PH_TypeListMCG	*ph_InfoMCG;	/* OPTIONAL */
	struct CG_ConfigInfo_v1540_IEs__measResultReportCGI {
		ARFCN_ValueNR_t	 ssbFrequency;
		PhysCellId_t	 cellForWhichToReportCGI;
		CGI_InfoNR_t	 cgi_Info;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *measResultReportCGI;
	struct CG_ConfigInfo_v1560_IEs	*nonCriticalExtension;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CG_ConfigInfo_v1540_IEs_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_CG_ConfigInfo_v1540_IEs;
extern asn_SEQUENCE_specifics_t asn_SPC_CG_ConfigInfo_v1540_IEs_specs_1;
extern asn_TYPE_member_t asn_MBR_CG_ConfigInfo_v1540_IEs_1[3];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "PH-TypeListMCG.h"
#include "CG-ConfigInfo-v1560-IEs.h"

#endif	/* _CG_ConfigInfo_v1540_IEs_H_ */
#include "asn_internal.h"
