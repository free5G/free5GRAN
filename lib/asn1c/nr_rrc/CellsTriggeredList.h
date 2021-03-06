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
 * From ASN.1 module "NR-UE-Variables"
 * 	`asn1c -gen-PER -fcompound-names -findirect-choice -no-gen-example`
 */

#ifndef	_CellsTriggeredList_H_
#define	_CellsTriggeredList_H_


#include "asn_application.h"

/* Including external dependencies */
#include "asn_SEQUENCE_OF.h"
#include "PhysCellId.h"
#include "EUTRA-PhysCellId.h"
#include "constr_CHOICE.h"
#include "constr_SEQUENCE_OF.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum CellsTriggeredList__Member_PR {
	CellsTriggeredList__Member_PR_NOTHING,	/* No components present */
	CellsTriggeredList__Member_PR_physCellId,
	CellsTriggeredList__Member_PR_physCellIdEUTRA
} CellsTriggeredList__Member_PR;

/* Forward definitions */
typedef struct CellsTriggeredList__Member {
	CellsTriggeredList__Member_PR present;
	union CellsTriggeredList__Member_u {
		PhysCellId_t	 physCellId;
		EUTRA_PhysCellId_t	 physCellIdEUTRA;
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CellsTriggeredList__Member;

/* CellsTriggeredList */
typedef struct CellsTriggeredList {
	A_SEQUENCE_OF(CellsTriggeredList__Member) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CellsTriggeredList_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_CellsTriggeredList;
extern asn_SET_OF_specifics_t asn_SPC_CellsTriggeredList_specs_1;
extern asn_TYPE_member_t asn_MBR_CellsTriggeredList_1[1];
extern asn_per_constraints_t asn_PER_type_CellsTriggeredList_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _CellsTriggeredList_H_ */
#include "asn_internal.h"
