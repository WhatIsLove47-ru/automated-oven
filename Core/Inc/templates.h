/*
 * templates.h
 *
 *  Created on: Mar 24, 2021
 *      Author: Vlad
 */

#ifndef __TEMPLATES_H
#define __TEMPLATES_H

#ifdef __cplusplus
extern "C" {
#endif

#define CAT(X,Y) X##_##Y
#define TEMPLATE(X,Y) CAT(X,Y)

#ifdef __cplusplus
}
#endif

#endif /* INC_TEMPLATES_H_ */
