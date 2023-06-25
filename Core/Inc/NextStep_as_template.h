/*
 * NextStep_as_template.h
 *
 *  Created on: Mar 24, 2023
 *      Author: Vlad
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifdef T1
#include "templates.h"
uint8_t TEMPLATE(NextStep,T1)(T1 *hotplace);
#endif

#ifdef __cplusplus
}
#endif
/*****END OF FILE****/