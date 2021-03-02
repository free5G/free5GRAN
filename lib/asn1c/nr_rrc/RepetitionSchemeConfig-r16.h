/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "NR-RRC-Definitions.asn"
 * 	`asn1c -fcompound-names -no-gen-example -pdu=all`
 */

#ifndef	_RepetitionSchemeConfig_r16_H_
#define	_RepetitionSchemeConfig_r16_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NULL.h"
#include "FDM-TDM-r16.h"
#include "constr_CHOICE.h"
#include "SlotBased-r16.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum RepetitionSchemeConfig_r16_PR {
	RepetitionSchemeConfig_r16_PR_NOTHING,	/* No components present */
	RepetitionSchemeConfig_r16_PR_fdm_TDM_r16,
	RepetitionSchemeConfig_r16_PR_slotBased_r16
} RepetitionSchemeConfig_r16_PR;
typedef enum RepetitionSchemeConfig_r16__fdm_TDM_r16_PR {
	RepetitionSchemeConfig_r16__fdm_TDM_r16_PR_NOTHING,	/* No components present */
	RepetitionSchemeConfig_r16__fdm_TDM_r16_PR_release,
	RepetitionSchemeConfig_r16__fdm_TDM_r16_PR_setup
} RepetitionSchemeConfig_r16__fdm_TDM_r16_PR;
typedef enum RepetitionSchemeConfig_r16__slotBased_r16_PR {
	RepetitionSchemeConfig_r16__slotBased_r16_PR_NOTHING,	/* No components present */
	RepetitionSchemeConfig_r16__slotBased_r16_PR_release,
	RepetitionSchemeConfig_r16__slotBased_r16_PR_setup
} RepetitionSchemeConfig_r16__slotBased_r16_PR;

/* RepetitionSchemeConfig-r16 */
typedef struct RepetitionSchemeConfig_r16 {
	RepetitionSchemeConfig_r16_PR present;
	union RepetitionSchemeConfig_r16_u {
		struct RepetitionSchemeConfig_r16__fdm_TDM_r16 {
			RepetitionSchemeConfig_r16__fdm_TDM_r16_PR present;
			union RepetitionSchemeConfig_r16__fdm_TDM_r16_u {
				NULL_t	 release;
				FDM_TDM_r16_t	 setup;
			} choice;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} fdm_TDM_r16;
		struct RepetitionSchemeConfig_r16__slotBased_r16 {
			RepetitionSchemeConfig_r16__slotBased_r16_PR present;
			union RepetitionSchemeConfig_r16__slotBased_r16_u {
				NULL_t	 release;
				SlotBased_r16_t	 setup;
			} choice;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} slotBased_r16;
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RepetitionSchemeConfig_r16_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RepetitionSchemeConfig_r16;
extern asn_CHOICE_specifics_t asn_SPC_RepetitionSchemeConfig_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_RepetitionSchemeConfig_r16_1[2];
extern asn_per_constraints_t asn_PER_type_RepetitionSchemeConfig_r16_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _RepetitionSchemeConfig_r16_H_ */
#include "asn_internal.h"