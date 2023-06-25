/*
 * all_possible_NextStep.h
 *
 *  Created on: Mar 24, 2023
 *      Author: Vlad
 */

#ifndef __ALL_POSSIBLE_NEXTSTEP_H
#define __ALL_POSSIBLE_NEXTSTEP_H

#ifdef __cplusplus
extern "C" {
#endif

//#include <templates.h>
//#include <hotplaces.h>
#ifdef T1
#undef T1
#endif
#define T1 Hotplace
#include <NextStep_as_template.h>
#ifdef T1
#undef T1
#endif
#define T1 Oven
#include <NextStep_as_template.h>

#ifdef __cplusplus
}
#endif

#endif /* __ALL_POSSIBLE_NEXTSTEP_H */
/*****END OF FILE****/