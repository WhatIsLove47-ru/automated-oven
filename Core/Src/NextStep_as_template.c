/*
 * NextStep_as_template.c
 *
 *  Created on: Mar 24, 2021
 *      Author: Vlad
 */

#ifdef T2

#include <templates.h>
#include <string.h>

uint8_t TEMPLATE(NextStep, T2)(T2 *hotplace) {
	if (++hotplace->step < hotplace->algorithm->size) {
		hotplace->profile = hotplace->algorithm->profile[hotplace->step];
		return 0; // Just next step
	}
	else {
		hotplace->isOn = 0;
		memset(&hotplace->profile, 0, sizeof(typeof(hotplace->profile)));
		return 1; // Let's start the PARTY
	}
}

#endif
