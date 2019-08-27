/*!
 *  @file		: utilities.h
 *
 *	@created	: 11. 07. 2017 11:38:39
 *	@date		: 24. 10. 2017
 *  @author		: peter.medvesek
 */ 


#ifndef UTILITIES_INCL_H_
#define UTILITIES_INCL_H_

#include <stdint.h>
#include <stddef.h>

uint8_t	num2str(int32_t num_in, uint8_t* out_str); //return string lenght
uint32_t str2num(const uint8_t* str); //return converted number

#endif /* UTILITIES_INCL_H_ */
