/*
 * isNewState_as_template.c
 *
 *  Created on: Mar 24, 2021
 *      Author: Vlad
 */

#ifdef T1

#include <templates.h>
#include <all_possible_NextStep.h>

uint8_t TEMPLATE(isNewState,T1)(T1 *hotplace) {
	if (hotplace->isOn) {
		if (hotplace->profile.time.Seconds > 0) {
			hotplace->profile.time.Seconds--;
		} else if (hotplace->profile.time.Minutes > 0) {
			hotplace->profile.time.Seconds = 59;
			hotplace->profile.time.Minutes--;
		} else if (hotplace->profile.time.Hours > 0) {
			hotplace->profile.time.Seconds = 59;
			hotplace->profile.time.Minutes = 59;
			hotplace->profile.time.Hours--;
		} else {
			return 1 + TEMPLATE(NextStep, T1)(hotplace);
		}
	}
	return 0;
}
#endif
