/*
 * music.h
 *
 *  Created on: Feb 10, 2021
 *      Author: Vlad
 */

#ifndef __MUSIC_H
#define __MUSIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

typedef struct {
	float freq;
	uint16_t time;
} SoundTypeDef;

typedef struct {
	TIM_HandleTypeDef *htim;
	unsigned char TIM_Channel;
	int Step;
	_Bool Playing;
	int sound_time;
	int sound_counter;
	const SoundTypeDef *Music;
	int Musicsize;
	int PreScaler;
} music_t;

#define C   261.6f //Do
#define C_  277.2f //Do#
#define D   293.7f //Re
#define D_  311.1f //Re#
#define E   329.6f //Mi
#define F   349.2f //Fa
#define F_  370.0f //Fa#
#define G   392.0f //Sol
#define G_  415.3f //Sol#
#define A   440.0f //La
#define A_  466.2f //La#
#define H   494.9f //Si

#define t1      2000
#define t2      1000
#define t4      500
#define t8      250
#define t16     125

#define SYSCLK 72000000

void PlayMusic(music_t *Music, const SoundTypeDef *Notes, TIM_HandleTypeDef *htim,
		int Musicsize, int PreScaler, int TIM_Channel);
void PauseMusic(music_t *Music);
void StopMusic(music_t *Music);
void Sound_IRQHandler(music_t *Music);

#ifdef __cplusplus
}
#endif

#endif /* __MUSIC_H */
