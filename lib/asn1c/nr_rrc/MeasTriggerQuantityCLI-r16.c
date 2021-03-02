/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "NR-RRC-Definitions.asn"
 * 	`asn1c -fcompound-names -no-gen-example -pdu=all`
 */

#include "MeasTriggerQuantityCLI-r16.h"

static asn_oer_constraints_t asn_OER_type_MeasTriggerQuantityCLI_r16_constr_1 CC_NOTUSED = {
	{ 0, 0 },
	-1};
asn_per_constraints_t asn_PER_type_MeasTriggerQuantityCLI_r16_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 1,  1,  0,  1 }	/* (0..1) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
asn_TYPE_member_t asn_MBR_MeasTriggerQuantityCLI_r16_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct MeasTriggerQuantityCLI_r16, choice.srs_RSRP_r16),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SRS_RSRP_Range_r16,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"srs-RSRP-r16"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct MeasTriggerQuantityCLI_r16, choice.cli_RSSI_r16),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_CLI_RSSI_Range_r16,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"cli-RSSI-r16"
		},
};
static const asn_TYPE_tag2member_t asn_MAP_MeasTriggerQuantityCLI_r16_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* srs-RSRP-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* cli-RSSI-r16 */
};
asn_CHOICE_specifics_t asn_SPC_MeasTriggerQuantityCLI_r16_specs_1 = {
	sizeof(struct MeasTriggerQuantityCLI_r16),
	offsetof(struct MeasTriggerQuantityCLI_r16, _asn_ctx),
	offsetof(struct MeasTriggerQuantityCLI_r16, present),
	sizeof(((struct MeasTriggerQuantityCLI_r16 *)0)->present),
	asn_MAP_MeasTriggerQuantityCLI_r16_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0,
	-1	/* Extensions start */
};
asn_TYPE_descriptor_t asn_DEF_MeasTriggerQuantityCLI_r16 = {
	"MeasTriggerQuantityCLI-r16",
	"MeasTriggerQuantityCLI-r16",
	&asn_OP_CHOICE,
	0,	/* No effective tags (pointer) */
	0,	/* No effective tags (count) */
	0,	/* No tags (pointer) */
	0,	/* No tags (count) */
	{ &asn_OER_type_MeasTriggerQuantityCLI_r16_constr_1, &asn_PER_type_MeasTriggerQuantityCLI_r16_constr_1, CHOICE_constraint },
	asn_MBR_MeasTriggerQuantityCLI_r16_1,
	2,	/* Elements count */
	&asn_SPC_MeasTriggerQuantityCLI_r16_specs_1	/* Additional specs */
};
