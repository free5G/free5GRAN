/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-InterNodeDefinitions"
 * 	found in "fixed_grammar.asn"
 * 	`asn1c -gen-PER -fcompound-names -findirect-choice -no-gen-example`
 */

#ifndef	_PH_TypeListSCG_H_
#define	_PH_TypeListSCG_H_


#include "asn_application.h"

/* Including external dependencies */
#include "asn_SEQUENCE_OF.h"
#include "constr_SEQUENCE_OF.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct PH_InfoSCG;

/* PH-TypeListSCG */
typedef struct PH_TypeListSCG {
	A_SEQUENCE_OF(struct PH_InfoSCG) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} PH_TypeListSCG_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_PH_TypeListSCG;
extern asn_SET_OF_specifics_t asn_SPC_PH_TypeListSCG_specs_1;
extern asn_TYPE_member_t asn_MBR_PH_TypeListSCG_1[1];
extern asn_per_constraints_t asn_PER_type_PH_TypeListSCG_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "PH-InfoSCG.h"

#endif	/* _PH_TypeListSCG_H_ */
#include "asn_internal.h"