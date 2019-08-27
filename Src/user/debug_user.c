
#include "debug_user.h"
#include "app_setup.h"

//#define DEBUG

void user_assert_failed(uint8_t* file, uint32_t line);

static uint32_t _line;
static uint8_t* _file;

//#ifdef DEBUG
#if ( USE_ASSERT == 1u )

void user_assert_failed(uint8_t* file, uint32_t line)
{
	_line = line;
	_file = file;
    while(1){}
}


#else
// empty function !
void user_assert_failed(uint8_t * file, uint32_t line) 
{
    (void)file;
    (void)line;
}

#endif
