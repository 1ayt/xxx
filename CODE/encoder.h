/*
 * encoder.h
 *
 *  Created on: 2022Äê1ÔÂ15ÈÕ
 *      Author: xqx
 */

#ifndef CODE_ENCODER_H_
#define CODE_ENCODER_H_

#include "zf_ccu6_pit.h"
#include "zf_eru.h"
#include "zf_gpio.h"

void encoder_init(void);
void encoder_read_right(int* a,int count_right);
void encoder_read_left(int* a,int count_left);
void encoder_count_reset(int* count);


#endif /* CODE_ENCODER_H_ */
