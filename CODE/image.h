/*
 * image.h
 *
 *  Created on: 2022��2��10��
 *      Author: xqx
 */

#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

#define GrayScale    256

#include "headfile.h"
#include "zf_uart.h"

uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row);
void Send_Gray_image(uint8 *imag,uint16 col,uint16 row,int threshold);
int Middle_Line(uint8 *imag);

#endif /* CODE_IMAGE_H_ */
