
#ifndef _DEBUG_USER_
#define _DEBUG_USER_

//#include "types.h"
#include <stdint.h>

extern void user_assert_failed(uint8_t* file, uint32_t line);

#define ASSERT_EMBED(expr) ((expr) ? (void)0 : user_assert_failed((uint8_t*)__FILE__, __LINE__))


#endif //_DEBUG_USER_