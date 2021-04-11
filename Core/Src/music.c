#include "music.h"
#include <math.h>

void sound(music_t *Music, float freq, int time_ms) {
	if (freq > 0) {
		__HAL_TIM_SET_AUTORELOAD(Music->htim,
				(unsigned int )lroundf( ((float )SYSCLK / (float)Music->PreScaler / freq)));
		__HAL_TIM_SET_COMPARE(Music->htim, (unsigned int )Music->TIM_Channel,
				lroundf((float)__HAL_TIM_GET_AUTORELOAD(Music->htim) / 2.0f));
	} else {
		__HAL_TIM_SET_AUTORELOAD(Music->htim, 1000);
		__HAL_TIM_SET_COMPARE(Music->htim, (unsigned int )Music->TIM_Channel,
				0);
	}
	__HAL_TIM_SET_COUNTER(Music->htim, 0);
	Music->sound_time = (int) ((float) (((float) SYSCLK
			/ (float) Music->PreScaler
			/ (float) __HAL_TIM_GET_AUTORELOAD(Music->htim)) * (float) time_ms)
			/ 1000.0f);
	Music->sound_counter = 0;
	HAL_TIM_PWM_Start_IT(Music->htim, (unsigned int) Music->TIM_Channel);
}

void PlayMusic(music_t *Music, const SoundTypeDef *Notes,
		TIM_HandleTypeDef *htim, int Musicsize, int PreScaler, int TIM_Channel) {
	Music->Music = Notes;
	Music->htim = htim;
	Music->Musicsize = Musicsize;
	Music->PreScaler = PreScaler;
	Music->TIM_Channel = TIM_Channel;
	Music->Step = 0;
	Music->Playing = 1;
	sound(Music, Music->Music[Music->Step].freq,
			Music->Music[Music->Step].time);
}

void PauseMusic(music_t *Music) {
	Music->Playing = 0;
}

void StopMusic(music_t *Music) {
	Music->Step = 0;
	Music->Playing = 0;
}

void Sound_IRQHandler(music_t *Music) {
	Music->sound_counter++;
	if (Music->sound_counter > Music->sound_time) {
		if (Music->Playing == 0) {
			HAL_TIM_PWM_Stop_IT(Music->htim, (unsigned int) Music->TIM_Channel);
		} else {
			if (Music->Step < Music->Musicsize - 1) {
				if (__HAL_TIM_GET_COMPARE(Music->htim,
						(unsigned int )Music->TIM_Channel) == 0) {
					Music->Step++;
					sound(Music, Music->Music[Music->Step].freq,
							Music->Music[Music->Step].time);
				} else {
					sound(Music, 0.0f, 30);
				}
			} else {
				Music->Playing = 0;
				HAL_TIM_PWM_Stop_IT(Music->htim,
						(unsigned int) Music->TIM_Channel);
			}
		}
	}
}
