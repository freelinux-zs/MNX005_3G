/**
 * Copyright (c) 2009 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

/** @file
* @brief Example template project.
* @defgroup nrf_templates_example Example Template
*
*/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_twi.h"


#include "app_util_platform.h"
#include "nrf_gpio.h"
//#include "nrf_drv_config.h"
#include "nrf_delay.h"
#include "bma2x2_register_map.h"



#define BMA_TWI_SCL_PIN 20
#define BMA_TWI_SDA_PIN 21
#define MPU_TWI_BUFFER_SIZE     	  14
#define BMA2X2_TWI_TIMEOUT 			  10000 
#define BMA2X2_ADDRESS            0X19

static const nrf_drv_twi_t m_twi_instance = NRF_DRV_TWI_INSTANCE(0);

volatile static bool twi_tx_done = false;
volatile static bool twi_rx_done = false;

uint8_t twi_tx_buffer[MPU_TWI_BUFFER_SIZE];

static void nrf_drv_mpu_twi_event_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            switch(p_event->xfer_desc.type)
            {
                case NRF_DRV_TWI_XFER_TX:
                    twi_tx_done = true;
                    break;
                case NRF_DRV_TWI_XFER_TXTX:
                    twi_tx_done = true;
                    break;
                case NRF_DRV_TWI_XFER_RX:
                    twi_rx_done = true;
                    break;
                case NRF_DRV_TWI_XFER_TXRX:
                    twi_rx_done = true;
                    break;
                default:
                    break;
            }
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
            break;
        default:
            break;
    }
}

static uint32_t bma2x2_twi_init(void)
{
    uint32_t err_code;
    
    const nrf_drv_twi_config_t twi_mpu_config = {
       .scl                = BMA_TWI_SCL_PIN,
       .sda                = BMA_TWI_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGHEST
    };
    
    err_code = nrf_drv_twi_init(&m_twi_instance, &twi_mpu_config, nrf_drv_mpu_twi_event_handler, NULL);
    if(err_code != NRF_SUCCESS)
	  {
		    return err_code;
	  }
    
    nrf_drv_twi_enable(&m_twi_instance);
	
	  return NRF_SUCCESS;
}

uint32_t nrf_drv_bma_read_registers(uint8_t reg, uint8_t * p_data, uint32_t length)
{
    uint32_t err_code;
    uint32_t timeout = BMA2X2_TWI_TIMEOUT;

    err_code = nrf_drv_twi_tx(&m_twi_instance, BMA2X2_ADDRESS, &reg, 1, false);
    if(err_code != NRF_SUCCESS) return err_code;

    while((!twi_tx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_tx_done = false;

    err_code = nrf_drv_twi_rx(&m_twi_instance, BMA2X2_ADDRESS, p_data, length);
    if(err_code != NRF_SUCCESS) return err_code;

    timeout = BMA2X2_TWI_TIMEOUT;
    while((!twi_rx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_rx_done = false;

    return err_code;
}

uint32_t bma2x2_init(void)
{
	uint32_t err_code;
	uint8_t id =0;
	
	err_code = bma2x2_twi_init();
	if(err_code != NRF_SUCCESS) return err_code;
	
	for(uint8_t i = 0; i < 3; i++){
		nrf_drv_bma_read_registers(BMA2X2_REG_WHO_AM_I,&id,1);
		if(id == 0xfa)
			break;
	}
	
	return id;
}

