/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "NR-RRC-Definitions.asn"
 * 	`asn1c -fcompound-names -no-gen-example -pdu=all`
 */

#include "FreqSeparationClassDL-Only-r16.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
static asn_oer_constraints_t asn_OER_type_FreqSeparationClassDL_Only_r16_constr_1 CC_NOTUSED = {
	{ 0, 0 },
	-1};
asn_per_constraints_t asn_PER_type_FreqSeparationClassDL_Only_r16_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 3,  3,  0,  5 }	/* (0..5) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static const asn_INTEGER_enum_map_t asn_MAP_FreqSeparationClassDL_Only_r16_value2enum_1[] = {
	{ 0,	6,	"mhz200" },
	{ 1,	6,	"mhz400" },
	{ 2,	6,	"mhz600" },
	{ 3,	6,	"mhz800" },
	{ 4,	7,	"mhz1000" },
	{ 5,	7,	"mhz1200" }
};
static const unsigned int asn_MAP_FreqSeparationClassDL_Only_r16_enum2value_1[] = {
	4,	/* mhz1000(4) */
	5,	/* mhz1200(5) */
	0,	/* mhz200(0) */
	1,	/* mhz400(1) */
	2,	/* mhz600(2) */
	3	/* mhz800(3) */
};
const asn_INTEGER_specifics_t asn_SPC_FreqSeparationClassDL_Only_r16_specs_1 = {
	asn_MAP_FreqSeparationClassDL_Only_r16_value2enum_1,	/* "tag" => N; sorted by tag */
	asn_MAP_FreqSeparationClassDL_Only_r16_enum2value_1,	/* N => "tag"; sorted by N */
	6,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_FreqSeparationClassDL_Only_r16_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
asn_TYPE_descriptor_t asn_DEF_FreqSeparationClassDL_Only_r16 = {
	"FreqSeparationClassDL-Only-r16",
	"FreqSeparationClassDL-Only-r16",
	&asn_OP_NativeEnumerated,
	asn_DEF_FreqSeparationClassDL_Only_r16_tags_1,
	sizeof(asn_DEF_FreqSeparationClassDL_Only_r16_tags_1)
		/sizeof(asn_DEF_FreqSeparationClassDL_Only_r16_tags_1[0]), /* 1 */
	asn_DEF_FreqSeparationClassDL_Only_r16_tags_1,	/* Same as above */
	sizeof(asn_DEF_FreqSeparationClassDL_Only_r16_tags_1)
		/sizeof(asn_DEF_FreqSeparationClassDL_Only_r16_tags_1[0]), /* 1 */
	{ &asn_OER_type_FreqSeparationClassDL_Only_r16_constr_1, &asn_PER_type_FreqSeparationClassDL_Only_r16_constr_1, NativeEnumerated_constraint },
	0, 0,	/* Defined elsewhere */
	&asn_SPC_FreqSeparationClassDL_Only_r16_specs_1	/* Additional specs */
};
