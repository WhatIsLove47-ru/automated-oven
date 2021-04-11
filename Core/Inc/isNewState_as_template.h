/*
 * isNewState_as_template.h
 *
 *  Created on: Mar 24, 2021
 *      Author: Vlad
 */


#ifdef __cplusplus
extern "C" {
#endif

#ifdef T1
#include <templates.h>
uint8_t TEMPLATE(isNewState,T1)(T1 *hotplace);
#endif

#ifdef __cplusplus
}
#endif
