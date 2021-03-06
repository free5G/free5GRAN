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

#ifndef	_AffectedCarrierFreqCombInfoMRDC_H_
#define	_AffectedCarrierFreqCombInfoMRDC_H_


#include "asn_application.h"

/* Including external dependencies */
#include "VictimSystemType.h"
#include "NativeEnumerated.h"
#include "AffectedCarrierFreqCombNR.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum AffectedCarrierFreqCombInfoMRDC__interferenceDirectionMRDC {
	AffectedCarrierFreqCombInfoMRDC__interferenceDirectionMRDC_eutra_nr	= 0,
	AffectedCarrierFreqCombInfoMRDC__interferenceDirectionMRDC_nr	= 1,
	AffectedCarrierFreqCombInfoMRDC__interferenceDirectionMRDC_other	= 2,
	AffectedCarrierFreqCombInfoMRDC__interferenceDirectionMRDC_utra_nr_other	= 3,
	AffectedCarrierFreqCombInfoMRDC__interferenceDirectionMRDC_nr_other	= 4,
	AffectedCarrierFreqCombInfoMRDC__interferenceDirectionMRDC_spare3	= 5,
	AffectedCarrierFreqCombInfoMRDC__interferenceDirectionMRDC_spare2	= 6,
	AffectedCarrierFreqCombInfoMRDC__interferenceDirectionMRDC_spare1	= 7
} e_AffectedCarrierFreqCombInfoMRDC__interferenceDirectionMRDC;

/* Forward declarations */
struct AffectedCarrierFreqCombEUTRA;

/* AffectedCarrierFreqCombInfoMRDC */
typedef struct AffectedCarrierFreqCombInfoMRDC {
	VictimSystemType_t	 victimSystemType;
	long	 interferenceDirectionMRDC;
	struct AffectedCarrierFreqCombInfoMRDC__affectedCarrierFreqCombMRDC {
		struct AffectedCarrierFreqCombEUTRA	*affectedCarrierFreqCombEUTRA;	/* OPTIONAL */
		AffectedCarrierFreqCombNR_t	 affectedCarrierFreqCombNR;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *affectedCarrierFreqCombMRDC;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} AffectedCarrierFreqCombInfoMRDC_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_interferenceDirectionMRDC_3;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_AffectedCarrierFreqCombInfoMRDC;
extern asn_SEQUENCE_specifics_t asn_SPC_AffectedCarrierFreqCombInfoMRDC_specs_1;
extern asn_TYPE_member_t asn_MBR_AffectedCarrierFreqCombInfoMRDC_1[3];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "AffectedCarrierFreqCombEUTRA.h"

#endif	/* _AffectedCarrierFreqCombInfoMRDC_H_ */
#include "asn_internal.h"
