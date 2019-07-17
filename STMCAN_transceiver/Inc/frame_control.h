/*
 * frame_control.h
 *
 *  Created on: 23 mai 2019
 *      Author: cleme
 */

#ifndef FRAME_CONTROL_H_
#define FRAME_CONTROL_H_

#define VAL_ID 		  1930

#define VAL_DLC	  	  2 // octets

#define DATA_VALUE 0xFF11110000000000

#define VAL_CRC 	  0xF001
#define VAL_ACK		  0x1
#define VAL_EOF		  0x7F
#define VAL_INTER	  0x07

#define SOF_SIZE	1
#define ID_SIZE		11
#define RTR_SIZE	1
#define R0_SIZE		1
#define R1_SIZE		1
#define DLC_SIZE	4
#define CRC_SIZE	16
#define ACK_SIZE	2
#define EOF_SIZE	7
#define INTER_SIZE	3


#define __PACKED

typedef union
{
	uint8_t frame_tab[14] __PACKED;
//	uint8_t v[17] __PACKED;

	struct __PACKED
	{
		uint8_t ID_H : 7;
		uint8_t SOF : 1;

		uint8_t DLCH : 1;
		uint8_t R1 : 1;
		uint8_t R0 : 1;
		uint8_t RTR : 1;
		uint8_t ID_L : 4;

		uint8_t DATA1H : 5;
		uint8_t DLCL : 3;

		uint8_t DATA2H : 5;
		uint8_t DATA1L : 3;

		uint8_t DATA3H : 5;
		uint8_t DATA2L : 3;

		uint8_t DATA4H : 5;
		uint8_t DATA3L : 3;

		uint8_t DATA5H : 5;
		uint8_t DATA4L : 3;

		uint8_t DATA6H : 5;
		uint8_t DATA5L : 3;

		uint8_t DATA7H : 5;
		uint8_t DATA6L : 3;

		uint8_t DATA8H : 5;
		uint8_t DATA7L : 3;

		uint8_t CRC_1 : 5;
		uint8_t DATA8L : 3;

		uint8_t CRC_3 : 5;
		uint8_t CRC_2 : 3;

		uint8_t END_OF_FRAME_H : 3;
		uint8_t ACK : 2;
		uint8_t CRC_4 : 3;

		uint8_t NUL : 1;
		uint8_t INTER : 3;
		uint8_t END_OF_FRAME_L : 4;
	} bits;

} frame_t;

typedef struct
{
	uint8_t data_length;
	uint8_t frame_tab[14];
	uint8_t framesize;
	uint8_t fram_tab_stuffing[14];
} frame_correct_t;


frame_correct_t frameControl_Initialization(frame_t* frameCan);
frame_correct_t frame_control_gestion(frame_t* frameCan);
void frameControl_Bits_Stuffing(frame_correct_t);

#endif /* FRAME_CONTROL_H_ */
