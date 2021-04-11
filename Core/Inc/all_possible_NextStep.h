/*
 * all_possible_NextStep.h
 *
 *  Created on: Mar 24, 2021
 *      Author: Vlad
 */

#ifndef __ALL_POSSIBLE_NEXTSTEP_H
#define __ALL_POSSIBLE_NEXTSTEP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <templates.h>
#ifdef T2
#undef T2
#endif
#define T2 Hotplace
#include <NextStep_as_template.h>
#ifdef T2
#undef T2
#endif
#define T2 Oven
#include <NextStep_as_template.h>

#ifdef __cplusplus
}
#endif

#endif /* __ALL_POSSIBLE_NEXTSTEP_H */
