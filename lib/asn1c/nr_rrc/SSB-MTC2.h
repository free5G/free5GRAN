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

#ifndef	_SSB_MTC2_H_
#define	_SSB_MTC2_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeEnumerated.h"
#include "PhysCellId.h"
#include "asn_SEQUENCE_OF.h"
#include "constr_SEQUENCE_OF.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SSB_MTC2__periodicity {
	SSB_MTC2__periodicity_sf5	= 0,
	SSB_MTC2__periodicity_sf10	= 1,
	SSB_MTC2__periodicity_sf20	= 2,
	SSB_MTC2__periodicity_sf40	= 3,
	SSB_MTC2__periodicity_sf80	= 4,
	SSB_MTC2__periodicity_spare3	= 5,
	SSB_MTC2__periodicity_spare2	= 6,
	SSB_MTC2__periodicity_spare1	= 7
} e_SSB_MTC2__periodicity;

/* SSB-MTC2 */
typedef struct SSB_MTC2 {
	struct SSB_MTC2__pci_List {
		A_SEQUENCE_OF(PhysCellId_t) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *pci_List;
	long	 periodicity;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SSB_MTC2_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_periodicity_4;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_SSB_MTC2;
extern asn_SEQUENCE_specifics_t asn_SPC_SSB_MTC2_specs_1;
extern asn_TYPE_member_t asn_MBR_SSB_MTC2_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _SSB_MTC2_H_ */
#include "asn_internal.h"
