/*
 * frame_control.c
 *
 *  Created on: 23 mai 2019
 *      Author: cleme
 */
#include "stdlib.h"
#include "stdint.h"
#include "math.h"
#include "frame_control.h"


frame_correct_t frame_control_gestion(frame_t* frameCan);

/*********************************************************************
 *  Initialisation de la structure sans adaptation des octets inutilisés
 *  ******************************************************************/

frame_correct_t frameControl_Initialization(frame_t* frameCan)
{
	frame_correct_t tab;
	uint16_t val_id = VAL_ID;
	uint16_t val_crc = VAL_CRC;
	uint16_t val_eof = VAL_EOF;
	uint64_t val_data = DATA_VALUE;
	uint64_t val_temp1;
	uint64_t val_temp2;

	for (int i = 0; i <14 ; i++)
	{
		frameCan->frame_tab[i] = 0;
	}

	frameCan->bits.SOF = 1;

	frameCan->bits.ID_L = val_id & 0xF;
	frameCan->bits.ID_H = (val_id >> 4) & 0x7F;

	frameCan->bits.RTR = 0;

	frameCan->bits.R0 = 0;
	frameCan->bits.R1 = 0;

	frameCan->bits.DLCH = (VAL_DLC>>3) & 0x01;
	frameCan->bits.DLCL = VAL_DLC & 0x07;

	frameCan->bits.DATA1H = (val_data >> 59) & 0x1F;
	frameCan->bits.DATA1L = (val_data >> 56) & 0x07;

	frameCan->bits.DATA2H = (val_data >> 51) & 0x1F;
	frameCan->bits.DATA2L = (val_data >> 48) & 0x07;

	frameCan->bits.DATA3H = (val_data >> 43) & 0x1F;
	frameCan->bits.DATA3L = (val_data >> 40) & 0x07;

	frameCan->bits.DATA4H = (val_data >> 35) & 0x1F;
	frameCan->bits.DATA4L = (val_data >> 32) & 0x07;

	frameCan->bits.DATA5H = (val_data >> 27) & 0x1F;
	frameCan->bits.DATA5L = (val_data >> 24) & 0x07;

	frameCan->bits.DATA6H = (val_data >> 19) & 0x1F;
	frameCan->bits.DATA6L = (val_data >> 16) & 0x07;

	frameCan->bits.DATA7H = (val_data >> 11) & 0x1F;
	frameCan->bits.DATA7L = (val_data >> 8)  & 0x07;

	frameCan->bits.DATA8H = (val_data >> 3)  & 0x1F;
	frameCan->bits.DATA8L = val_data & 0x07;

	frameCan->bits.CRC_1 = (val_crc >> 11) & 0x1F;
	frameCan->bits.CRC_2 = (val_crc >> 8) & 0x07;
	frameCan->bits.CRC_3 = (val_crc >> 3) & 0x1F;
	frameCan->bits.CRC_4 = val_crc & 0x07;

	frameCan->bits.ACK = VAL_ACK;
	frameCan->bits.END_OF_FRAME_H = (val_eof >> 4) & 0x07 ;
	frameCan->bits.END_OF_FRAME_L = val_eof & 0x0F;
	frameCan->bits.INTER = VAL_INTER;
	frameCan->bits.NUL = 0;

	tab = frame_control_gestion(frameCan);
	return tab;
}

/*********************************************************************
 *  Modification de la trame en supprimant
 *  ******************************************************************/
frame_correct_t frame_control_gestion(frame_t* frameCan)
{
	frame_correct_t tab;
	uint8_t data_size = (frameCan->bits.DLCH << 4) | frameCan->bits.DLCL;
	uint8_t size_tab = 14 - (8-data_size);
	uint8_t tab_temp[size_tab];
	uint8_t tmp_H;
	uint8_t tmp_L;
	uint8_t j = 0;
	uint8_t y = 0;
	uint8_t flag = 0;

	for (uint8_t i = 0 ; i<14; i++)
	{

		tab_temp[j] = frameCan->frame_tab[i];

		if(i >= 2 && i <= 10 ) {

			if (flag == 0)
			{
				y = data_size;
				flag = 1;
			}

			if(y > 0) {
				y --;
			}
			else {
				tmp_H = (frameCan->frame_tab[i] & 0b11100000) >> 5;
				tmp_L = (frameCan->frame_tab[i+(8-data_size)] & 0b00011111);
				tab_temp[j] = (tmp_H << 5) | tmp_L;
				i = i+(8-data_size);
			}
		}
		j++;
	}

	for(uint8_t i = 0; i<14 ; i++)
	{
		tab.frame_tab[i] = tab_temp[i];
	}
	tab.framesize = size_tab;
	tab.data_length = data_size;
	return tab;
}

void frameControl_Bits_Stuffing(frame_correct_t tab_correct)
{
	uint8_t size_tab = 0;
	uint8_t add_bit = 0;
	size_tab = tab_correct.framesize;
	uint8_t tab[size_tab];
	uint8_t tab_stuff[size_tab + 3]; // ajout de 3 cases (stuff max = 24 = 3 octects)


	uint8_t bit_cpt_tab_stuff = 0;
	uint8_t case_cpt_tab_stuff = 0;
	uint8_t previous_bit = -1;
	uint8_t cpt_stuffing = 0;
	uint8_t stuff = 0;
	uint8_t flag_bit_added = 0;
	uint8_t val_max_stuff = 0;
	uint8_t cpt_val_max_stuff = 0;
	uint8_t nb_bit_stuff_added = 0;

	for(uint8_t i = 0; i<tab_correct.framesize ; i++) // on recopie dans un tableau local le tableau initial mis en forme
	{
		tab[i] = tab_correct.frame_tab[i];
	}
	for(uint8_t i = 0; i<(tab_correct.framesize+3) ; i++) // init tab tab_stuff
	{
		tab_stuff[i] = 0x00;
	}

	val_max_stuff = 34 + (tab_correct.data_length * 8);

	for (uint8_t i=0 ; i < size_tab ; i++) // balayage de toutes les cases du tableau initial
	{
		for (uint8_t j=0 ; j < 8 ; j++) // balayage de tous les bits du tableau initial
		{
			if (previous_bit == ((tab[i] & (0b10000000 >> j)) >> (7-j))) // test du bit actuel par rapport au bit precedent
				cpt_stuffing ++;
			else
				cpt_stuffing = 0;

			if(cpt_stuffing == 4) // si on détecte 5 bits de mêmes valeurs consécutivements il faut forcer le prochain à une valeur complémentaire
			{
				stuff = 1;
				cpt_stuffing = 0;
			}
			else
				stuff=0;

			if(j>bit_cpt_tab_stuff)
				tab_stuff[case_cpt_tab_stuff] |= ((tab[i] & ((0b10000000) >> j)) << (abs(j-bit_cpt_tab_stuff)));
			else
				tab_stuff[case_cpt_tab_stuff] |= ((tab[i] & ((0b10000000) >> j)) >> (abs(j-bit_cpt_tab_stuff)));


			if(bit_cpt_tab_stuff == 7)
			{
				case_cpt_tab_stuff ++;
				bit_cpt_tab_stuff = 0;
			}
			else
			{
				bit_cpt_tab_stuff ++;
			}


			if(stuff && (cpt_val_max_stuff < (val_max_stuff + nb_bit_stuff_added)))
			{
				if(previous_bit == 0)
					add_bit = 0b10000000;
				else
					add_bit = 0b00000000;

				if(j>bit_cpt_tab_stuff)
					tab_stuff[case_cpt_tab_stuff] |= (add_bit >> bit_cpt_tab_stuff) ;//<< (abs((j-bit_cpt_tab_stuff))); // si bit stuffing on ajoute un bit complémentaire au bit j
				else
					tab_stuff[case_cpt_tab_stuff] |= (add_bit >> bit_cpt_tab_stuff) ;//>> (abs((j+1)-bit_cpt_tab_stuff))); // dans le cas ou on est sur une case suivante sur le tableau initial par rapport au tab de stuff alors j<cpt_bit

				if(bit_cpt_tab_stuff == 7)
				{
					case_cpt_tab_stuff ++;
					bit_cpt_tab_stuff = 0;
				}
				else
				{
					bit_cpt_tab_stuff ++;
				}

				nb_bit_stuff_added ++;
				flag_bit_added = 1;
			}

			if(flag_bit_added == 0)
				previous_bit = ((tab[i] & (0b10000000 >> j)) >> (7-j));
			else
			{
				previous_bit = add_bit >> 7;
				flag_bit_added = 0;
			}

			cpt_val_max_stuff ++;
		}
	}
}
