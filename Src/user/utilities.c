/*
 * utilities.c
 *
 * Created: 11. 07. 2017 11:38:56
 *  Author: peter.medvesek
 */ 

#include "utilities.h"


//===========================================================
//! ASCII symbols

//===========================================================

static void _2nd_complement(int32_t* num);

//test: [OK]
static void _2nd_complement(int32_t* num) {
	int32_t temp = 0;
	temp = (~(*num) + 1); 
	*num = temp;
}

// test: [OK]
uint8_t num2str(int32_t num_in, uint8_t* out_str) {
	int32_t num = num_in;

    uint8_t loop_i = 0;
	uint8_t loop2_i = 0;
	uint8_t num_cnt = 0;
	uint8_t sign_offset = 0;
	uint8_t temp_str[15];
	
	if (num == 0)
	{
		out_str[0] = '0';
		out_str[1] = 0;
		return 1;
	} else {

		if (num < 0)
		{
			out_str[0] = '-';
			sign_offset = 1;
			_2nd_complement(&num);
		}
		for (loop_i = 0; num > 0; loop_i++)
		{
			temp_str[loop_i] = (num % 10) + '0'; // ascii shift for numbers 0 -> 48(dec)
			num = num / 10;
		}
		num_cnt = loop_i;
		loop_i += sign_offset;
		// add null termination at the end of string
		out_str[loop_i] = 0;
		// revers array
		for (loop2_i = 0; loop2_i < num_cnt; loop2_i++)
		{
			loop_i--;
			out_str[loop_i] = temp_str[loop2_i];
		}
		// return string size
		return (num_cnt + sign_offset);
	}
}


// test [OK] 25.10.2017
uint32_t str2num(const uint8_t* str) {
	uint8_t loop_i = 0;
	int32_t temp_num = 0;

	for (loop_i = 0; str[loop_i] != 0; ++loop_i)
	{
		if (loop_i > 100) {
			return 0; //protection if string is not NULL terminated
		}

		temp_num *= 10;
		if (str[loop_i] >= '0' && str[loop_i] <= '9')
		{
			temp_num += str[loop_i] - '0';
		} else {
			//NaN
			return 0; //this is not a number (NaN) 
		}
	}
	return temp_num; //ok
}
