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
/*-
 * Copyright (c) 2003, 2004 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * Redistribution and modifications are permitted subject to BSD license.
 */
#include "asn_internal.h"
#include "asn_SEQUENCE_OF.h"

typedef A_SEQUENCE_OF(void) asn_sequence;

void
asn_sequence_del(void *asn_sequence_of_x, int number, int _do_free) {
	asn_sequence *as = (asn_sequence *)asn_sequence_of_x;

	if(as) {
		void *ptr;
		int n;

		if(number < 0 || number >= as->count)
			return;	/* Nothing to delete */

		if(_do_free && as->free) {
			ptr = as->array[number];
		} else {
			ptr = 0;
		}

		/*
		 * Shift all elements to the left to hide the gap.
		 */
		--as->count;
		for(n = number; n < as->count; n++)
			as->array[n] = as->array[n+1];

		/*
		 * Invoke the third-party function only when the state
		 * of the parent structure is consistent.
		 */
		if(ptr) as->free(ptr);
	}
}

