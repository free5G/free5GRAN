/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "NR-RRC-Definitions.asn"
 * 	`asn1c -fcompound-names -no-gen-example -pdu=all`
 */

#ifndef	_MeasResultCLI_RSSI_r16_H_
#define	_MeasResultCLI_RSSI_r16_H_


#include "asn_application.h"

/* Including external dependencies */
#include "RSSI-ResourceId-r16.h"
#include "CLI-RSSI-Range-r16.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* MeasResultCLI-RSSI-r16 */
typedef struct MeasResultCLI_RSSI_r16 {
	RSSI_ResourceId_r16_t	 rssi_ResourceId_r16;
	CLI_RSSI_Range_r16_t	 cli_RSSI_Result_r16;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MeasResultCLI_RSSI_r16_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_MeasResultCLI_RSSI_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_MeasResultCLI_RSSI_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_MeasResultCLI_RSSI_r16_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _MeasResultCLI_RSSI_r16_H_ */
#include "asn_internal.h"