#ifndef AT_COMMAND_H__
#define AT_COMMAND_H__

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "nrf.h"
#include "nrf_peripherals.h"
#include "nrf_assert.h"

#ifdef __cplusplus
extern "C" {
#endif
	
void PutUARTByte(const char *fmt, ...);

__STATIC_INLINE void AT_Test(void)
{
	PutUARTByte("AT");
}	
	
#ifdef __cplusplus
}
#endif
#endif
