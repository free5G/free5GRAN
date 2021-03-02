/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "NR-RRC-Definitions.asn"
 * 	`asn1c -fcompound-names -no-gen-example -pdu=all`
 */

#ifndef	_AffectedCarrierFreq_r16_H_
#define	_AffectedCarrierFreq_r16_H_


#include "asn_application.h"

/* Including external dependencies */
#include "ARFCN-ValueNR.h"
#include "NativeEnumerated.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum AffectedCarrierFreq_r16__interferenceDirection_r16 {
	AffectedCarrierFreq_r16__interferenceDirection_r16_nr	= 0,
	AffectedCarrierFreq_r16__interferenceDirection_r16_other	= 1,
	AffectedCarrierFreq_r16__interferenceDirection_r16_both	= 2,
	AffectedCarrierFreq_r16__interferenceDirection_r16_spare	= 3
} e_AffectedCarrierFreq_r16__interferenceDirection_r16;

/* AffectedCarrierFreq-r16 */
typedef struct AffectedCarrierFreq_r16 {
	ARFCN_ValueNR_t	 carrierFreq_r16;
	long	 interferenceDirection_r16;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} AffectedCarrierFreq_r16_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_interferenceDirection_r16_3;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_AffectedCarrierFreq_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_AffectedCarrierFreq_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_AffectedCarrierFreq_r16_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _AffectedCarrierFreq_r16_H_ */
#include "asn_internal.h"