/*
 * all_possible_NextStep.c
 *
 *  Created on: Mar 24, 2021
 *      Author: Vlad
 */

#include <templates.h>
#include <hotplaces.h>
#include "all_possible_NextStep.h"
#ifdef T2
#undef T2
#endif
#define T2 Hotplace
#include "NextStep_as_template.c"
#ifdef T2
#undef T2
#endif
#define T2 Oven
#include "NextStep_as_template.c"

