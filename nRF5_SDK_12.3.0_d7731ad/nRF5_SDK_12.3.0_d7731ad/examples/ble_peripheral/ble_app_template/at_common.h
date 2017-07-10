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

__STATIC_INLINE void ata_set_apn(uint8_t *data)
{
	PutUARTByte("AT+EAPN=%s",data);
}	


__STATIC_INLINE void ata_set_ip1_port(uint8_t *ip1, uint8_t *port)
{
	PutUARTByte("AT+ESETIP1=%s,%s",ip1,port);
}	

__STATIC_INLINE void ata_set_ip2_port(uint8_t *ip2, uint8_t *port)
{
	PutUARTByte("AT+ESETIP2=%s,%s",ip2,port);
}	

__STATIC_INLINE void ata_set_ip3_port(uint8_t *ip3, uint8_t *port)
{
	PutUARTByte("AT+ESETIP3=%s,%s",ip3,port);
}	

__STATIC_INLINE void ata_tcp_connect(void)
{
	PutUARTByte("AT+ETCPON");
}	

__STATIC_INLINE void ata_tcp_disconnect(void)
{
	PutUARTByte("AT+ETCPDIS");
}	


__STATIC_INLINE void ata_tcp_send_data(uint8_t *data, uint8_t len)
{
	PutUARTByte("AT+ETCPSED=%s,%d",data,len);
}	


__STATIC_INLINE void ata_tcp_get_data(void)
{
	PutUARTByte("AT+ETCPRECEIVE");
}	

__STATIC_INLINE void ata_gps_on(void)
{
	PutUARTByte("AT+EGPSC=1");
}	

__STATIC_INLINE void ata_gps_off(void)
{
	PutUARTByte("AT+EGPSC=0");
}	

__STATIC_INLINE void ata_gps_get_data(void)
{
	PutUARTByte("AT+EGPSGET");
}	


__STATIC_INLINE void ata_nvram_data_write(uint8_t index, uint8_t *data)
{
	PutUARTByte("AT+ENVWRITE=%d,%s",index,data);
}	


__STATIC_INLINE void ata_nvram_data_read(uint8_t index)
{
	PutUARTByte("AT+ENVREAD=%d",index);
}


__STATIC_INLINE void ata_set_network_domain_name(uint8_t *data)
{
	PutUARTByte("AT+ESETDOMAIN=%s",data);
}	

	
#ifdef __cplusplus
}
#endif
#endif
