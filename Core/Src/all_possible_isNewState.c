/*
 * all_possible_isNewState.c
 *
 *  Created on: Mar 24, 2021
 *      Author: Vlad
 */

#include <templates.h>
#include <hotplaces.h>
#include "all_possible_isNewState.h"
#ifdef T1
#undef T1
#endif
#define T1 Hotplace
#include "isNewState_as_template.c"
#ifdef T1
#undef T1
#endif
#define T1 Oven
#include "isNewState_as_template.c"
