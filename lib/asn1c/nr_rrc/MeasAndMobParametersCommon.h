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

#ifndef	_MeasAndMobParametersCommon_H_
#define	_MeasAndMobParametersCommon_H_


#include "asn_application.h"

/* Including external dependencies */
#include "BIT_STRING.h"
#include "NativeEnumerated.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum MeasAndMobParametersCommon__ssb_RLM {
	MeasAndMobParametersCommon__ssb_RLM_supported	= 0
} e_MeasAndMobParametersCommon__ssb_RLM;
typedef enum MeasAndMobParametersCommon__ssb_AndCSI_RS_RLM {
	MeasAndMobParametersCommon__ssb_AndCSI_RS_RLM_supported	= 0
} e_MeasAndMobParametersCommon__ssb_AndCSI_RS_RLM;
typedef enum MeasAndMobParametersCommon__ext1__eventB_MeasAndReport {
	MeasAndMobParametersCommon__ext1__eventB_MeasAndReport_supported	= 0
} e_MeasAndMobParametersCommon__ext1__eventB_MeasAndReport;
typedef enum MeasAndMobParametersCommon__ext1__handoverFDD_TDD {
	MeasAndMobParametersCommon__ext1__handoverFDD_TDD_supported	= 0
} e_MeasAndMobParametersCommon__ext1__handoverFDD_TDD;
typedef enum MeasAndMobParametersCommon__ext1__eutra_CGI_Reporting {
	MeasAndMobParametersCommon__ext1__eutra_CGI_Reporting_supported	= 0
} e_MeasAndMobParametersCommon__ext1__eutra_CGI_Reporting;
typedef enum MeasAndMobParametersCommon__ext1__nr_CGI_Reporting {
	MeasAndMobParametersCommon__ext1__nr_CGI_Reporting_supported	= 0
} e_MeasAndMobParametersCommon__ext1__nr_CGI_Reporting;
typedef enum MeasAndMobParametersCommon__ext2__independentGapConfig {
	MeasAndMobParametersCommon__ext2__independentGapConfig_supported	= 0
} e_MeasAndMobParametersCommon__ext2__independentGapConfig;
typedef enum MeasAndMobParametersCommon__ext2__periodicEUTRA_MeasAndReport {
	MeasAndMobParametersCommon__ext2__periodicEUTRA_MeasAndReport_supported	= 0
} e_MeasAndMobParametersCommon__ext2__periodicEUTRA_MeasAndReport;
typedef enum MeasAndMobParametersCommon__ext2__handoverFR1_FR2 {
	MeasAndMobParametersCommon__ext2__handoverFR1_FR2_supported	= 0
} e_MeasAndMobParametersCommon__ext2__handoverFR1_FR2;
typedef enum MeasAndMobParametersCommon__ext2__maxNumberCSI_RS_RRM_RS_SINR {
	MeasAndMobParametersCommon__ext2__maxNumberCSI_RS_RRM_RS_SINR_n4	= 0,
	MeasAndMobParametersCommon__ext2__maxNumberCSI_RS_RRM_RS_SINR_n8	= 1,
	MeasAndMobParametersCommon__ext2__maxNumberCSI_RS_RRM_RS_SINR_n16	= 2,
	MeasAndMobParametersCommon__ext2__maxNumberCSI_RS_RRM_RS_SINR_n32	= 3,
	MeasAndMobParametersCommon__ext2__maxNumberCSI_RS_RRM_RS_SINR_n64	= 4,
	MeasAndMobParametersCommon__ext2__maxNumberCSI_RS_RRM_RS_SINR_n96	= 5
} e_MeasAndMobParametersCommon__ext2__maxNumberCSI_RS_RRM_RS_SINR;
typedef enum MeasAndMobParametersCommon__ext3__nr_CGI_Reporting_ENDC {
	MeasAndMobParametersCommon__ext3__nr_CGI_Reporting_ENDC_supported	= 0
} e_MeasAndMobParametersCommon__ext3__nr_CGI_Reporting_ENDC;

/* MeasAndMobParametersCommon */
typedef struct MeasAndMobParametersCommon {
	BIT_STRING_t	*supportedGapPattern;	/* OPTIONAL */
	long	*ssb_RLM;	/* OPTIONAL */
	long	*ssb_AndCSI_RS_RLM;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct MeasAndMobParametersCommon__ext1 {
		long	*eventB_MeasAndReport;	/* OPTIONAL */
		long	*handoverFDD_TDD;	/* OPTIONAL */
		long	*eutra_CGI_Reporting;	/* OPTIONAL */
		long	*nr_CGI_Reporting;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	struct MeasAndMobParametersCommon__ext2 {
		long	*independentGapConfig;	/* OPTIONAL */
		long	*periodicEUTRA_MeasAndReport;	/* OPTIONAL */
		long	*handoverFR1_FR2;	/* OPTIONAL */
		long	*maxNumberCSI_RS_RRM_RS_SINR;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext2;
	struct MeasAndMobParametersCommon__ext3 {
		long	*nr_CGI_Reporting_ENDC;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext3;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MeasAndMobParametersCommon_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_ssb_RLM_3;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_ssb_AndCSI_RS_RLM_5;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_eventB_MeasAndReport_9;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_handoverFDD_TDD_11;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_eutra_CGI_Reporting_13;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_nr_CGI_Reporting_15;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_independentGapConfig_18;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_periodicEUTRA_MeasAndReport_20;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_handoverFR1_FR2_22;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_maxNumberCSI_RS_RRM_RS_SINR_24;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_nr_CGI_Reporting_ENDC_32;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_MeasAndMobParametersCommon;
extern asn_SEQUENCE_specifics_t asn_SPC_MeasAndMobParametersCommon_specs_1;
extern asn_TYPE_member_t asn_MBR_MeasAndMobParametersCommon_1[6];

#ifdef __cplusplus
}
#endif

#endif	/* _MeasAndMobParametersCommon_H_ */
#include "asn_internal.h"
