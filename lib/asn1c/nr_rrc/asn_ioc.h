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
 * Run-time support for Information Object Classes.
 * Copyright (c) 2017 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * Redistribution and modifications are permitted subject to BSD license.
 */
#ifndef	ASN_IOC_H
#define	ASN_IOC_H

#include "asn_system.h"		/* Platform-specific types */

#ifdef __cplusplus
extern "C" {
#endif

struct asn_TYPE_descriptor_s;
struct asn_ioc_cell_s;

/*
 * X.681, #13
 */
typedef struct asn_ioc_set_s {
    size_t rows_count;
    size_t columns_count;
    const struct asn_ioc_cell_s *rows;
} asn_ioc_set_t;


typedef struct asn_ioc_cell_s {
    const char *field_name; /* Is equal to corresponding column_name */
    enum {
        aioc__undefined = 0,
        aioc__value,
        aioc__type,
        aioc__open_type,
    } cell_kind;
    struct asn_TYPE_descriptor_s *type_descriptor;
    const void *value_sptr;
    struct {
        size_t types_count;
        struct {
            unsigned choice_position;
        } *types;
    } open_type;
} asn_ioc_cell_t;


#ifdef __cplusplus
}
#endif

#endif	/* ASN_IOC_H */
