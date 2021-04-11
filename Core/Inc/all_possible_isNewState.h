/*
 * all_possible_isNewState.h
 *
 *  Created on: Mar 24, 2021
 *      Author: Vlad
 */

#ifndef __ALL_POSSIBLE_ISNEWSTATE_H
#define __ALL_POSSIBLE_ISNEWSTATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <templates.h>
#ifdef T1
#undef T1
#endif
#define T1 Hotplace
#include <isNewState_as_template.h>
#ifdef T1
#undef T1
#endif
#define T1 Oven
#include <isNewState_as_template.h>

#ifdef __cplusplus
}
#endif

#endif /* __ALL_POSSIBLE_ISNEWSTATE_H */
