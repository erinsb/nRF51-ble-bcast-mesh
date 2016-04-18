/***********************************************************************************
Copyright (c) Nordic Semiconductor ASA
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of other
  contributors to this software may be used to endorse or promote products
  derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************************************************/

#include "rbc_mesh.h"
#include "bootloader_app.h"

#include "nrf_adv_conn.h"
#include "led_config.h"
#include "gpio_config.h"
#include "mesh_aci.h"

#include "softdevice_handler.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "boards.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "simple_uart.h"

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

void retarget_init(void)
{
    simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, HWFC);
}

int fputc(int ch, FILE * p_file)
{
    simple_uart_put((uint8_t)ch);
    return 0;
}

int analog_read(int pin_num);

/* Debug macros for debugging with logic analyzer */
#define SET_PIN(x) NRF_GPIO->OUTSET = (1 << (x))
#define CLEAR_PIN(x) NRF_GPIO->OUTCLR = (1 << (x))
#define TICK_PIN(x) do { SET_PIN((x)); CLEAR_PIN((x)); }while(0)

#define MESH_ACCESS_ADDR        (RBC_MESH_ACCESS_ADDRESS_BLE_ADV)
#define MESH_INTERVAL_MIN_MS    (100)
#define MESH_CHANNEL            (38)
#define MESH_CLOCK_SOURCE       (NRF_CLOCK_LFCLKSRC_XTAL_75_PPM)

/**
* @brief General error handler.
*/
static void error_loop(void)
{
		#ifdef DEBUG
			printf("error");
		#endif
    led_config(2, 1);
    led_config(3, 0);
    while (true)
    {
    }
}



/**
* @brief Softdevice crash handler, never returns
*
* @param[in] pc Program counter at which the assert failed
* @param[in] line_num Line where the error check failed
* @param[in] p_file_name File where the error check failed
*/
void sd_assert_handler(uint32_t pc, uint16_t line_num, const uint8_t* p_file_name)
{

	#ifdef DEBUG
	printf("sd_assert_handler");
	printf("error_code: %u\n", pc);
	printf("line_number: %u\n", line_num);
	printf("file_name: %s\n", (char *)p_file_name);
	#endif
    error_loop();
}

/**
* @brief App error handle callback. Called whenever an APP_ERROR_CHECK() fails.
*   Never returns.
*
* @param[in] error_code The error code sent to APP_ERROR_CHECK()
* @param[in] line_num Line where the error check failed
* @param[in] p_file_name File where the error check failed
*/
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
		//FILE *fp;
		//fp = fopen("error.log", "w+");
	//fprintf(fp, "error_code: %u\n", error_code);
	//fprintf(fp, "line_number: %u\n", line_num);
	//fprintf(fp, "file_name: %s\n", (char *)p_file_name);

	#ifdef DEBUG
	printf("app_error_handler");
	printf("error_code: %u\n", error_code);
	printf("line_number: %u\n", line_num);
	printf("file_name: %s\n", (char *)p_file_name);
	#endif
    error_loop();
}

void HardFault_Handler(void)
{
		#ifdef DEBUG
		printf("HardFault_Handler");
	#endif
    error_loop();
}

/**
* @brief Softdevice event handler
*/
void sd_ble_evt_handler(ble_evt_t* p_ble_evt)
{
    rbc_mesh_ble_evt_handler(p_ble_evt);
    nrf_adv_conn_evt_handler(p_ble_evt);
}
/**
* @brief RBC_MESH framework event handler. Defined in rbc_mesh.h. Handles
*   events coming from the mesh. Sets LEDs according to data
*
* @param[in] evt RBC event propagated from framework
*/
static void rbc_mesh_event_handler(rbc_mesh_event_t* evt)
{
	/*if(evt->event_type == RBC_MESH_EVENT_TYPE_UPDATE_VAL) {
		evt->data -= 32;
		printf("eve->data addr %p \n", &evt->data);
	}*/

#ifdef DEBUG
	printf("evt->event_type %x \n", evt->event_type);
	printf("evt->value_handle %u \n", evt->value_handle);
#endif

    TICK_PIN(28);
    switch (evt->event_type)
    {
        case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL:
        case RBC_MESH_EVENT_TYPE_NEW_VAL:
        case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
            if (evt->value_handle > 32)
                break;

            #ifndef RBC_MESH_SERIAL
						gpio_config(evt->value_handle, evt->data[0]);
						#ifdef DEBUG
							printf("gpio_config");
						#endif
						#endif
            break;
        case RBC_MESH_EVENT_TYPE_TX:
            break;
        case RBC_MESH_EVENT_TYPE_INITIALIZED:
            /* init BLE gateway softdevice application: */
            nrf_adv_conn_init();
            break;
        case RBC_MESH_EVENT_TYPE_REFRESH_VAL:
        if (evt->value_handle > 32)
            break;
        #ifdef HVAC
        if(evt->value_handle == 5 || evt->value_handle == 6){
            uint8_t val = analog_read(evt->value_handle); //get analog value on pin
					#ifdef DEBUG
					printf("value returned is: %u \n", val);
					#endif
            rbc_mesh_value_set(evt->value_handle,&val,1);
        }
        #endif
    }
}

/**
* @brief Initialize GPIO pins, for LEDs and debugging
*/
void gpio_init(void)
{
    nrf_gpio_range_cfg_output(LED_START, LED_STOP);

    for (uint32_t i = 0; i < LEDS_NUMBER; ++i)
    {
        nrf_gpio_pin_set(LED_START + i);
    }

#if defined(BOARD_PCA10001) || defined(BOARD_PCA10028)
    //nrf_gpio_range_cfg_output(0, 32);
		nrf_gpio_range_cfg_input(5, 6, 	NRF_GPIO_PIN_NOPULL);
#endif

#ifdef BOARD_PCA10028
    #ifdef BUTTONS
        nrf_gpio_cfg_input(BUTTON_1, NRF_GPIO_PIN_PULLUP);
        nrf_gpio_cfg_input(BUTTON_2, NRF_GPIO_PIN_PULLUP);
        nrf_gpio_cfg_input(BUTTON_3, NRF_GPIO_PIN_PULLUP);
        nrf_gpio_cfg_input(BUTTON_4, NRF_GPIO_PIN_PULLUP);
    #endif
#endif

}

int analog_read(int pin_num)

{
    uint16_t adc_result;

    NRF_ADC->INTENSET = (ADC_INTENSET_END_Disabled << ADC_INTENSET_END_Pos);     /*!< Interrupt enabled. */

    // config ADC
    //*** pin_num= ADC_CONFIG_PSEL_AnalogInputX-1
    NRF_ADC->CONFIG = (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos) /*!< Analog external reference inputs disabled. */
                    | ((pin_num) << ADC_CONFIG_PSEL_Pos)
                    | (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos)   /*!< Use internal 1.2V bandgap voltage as reference for conversion. */
                    | (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) /*!< Analog input specified by PSEL with 1/3 prescaling used as input for the conversion. */
                    | (ADC_CONFIG_RES_8bit << ADC_CONFIG_RES_Pos);  /*!< 10bit ADC resolution. */
    //nrf_gpio_cfg_input(NRF_GPIO_PORT_SELECT_PORT0, GPIO_PIN_CNF_PULL_Disabled);
    // enable ADC
    NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled; /* Bit 0 : ADC enable. */

    // start ADC conversion
    NRF_ADC->TASKS_START = 1;

    // wait for conversion to end
    while (!NRF_ADC->EVENTS_END)
    {}
    NRF_ADC->EVENTS_END = 0;

    //Save your ADC result
    adc_result = NRF_ADC->RESULT;

    //Use the STOP task to save current. Workaround for PAN_028 rev1.1 anomaly 1.
    NRF_ADC->TASKS_STOP = 1;

    return adc_result;
}

/** @brief main function */
int main(void)
{
		#ifdef DEBUG
			retarget_init();
			printf("Start...\r\n");
		#endif
    /* init leds and pins */
    gpio_init();
    NRF_GPIO->OUTSET = (1 << 4);
    /* Enable Softdevice (including sd_ble before framework */
    SOFTDEVICE_HANDLER_INIT(MESH_CLOCK_SOURCE, NULL);
    softdevice_ble_evt_handler_set(sd_ble_evt_handler); /* app-defined event handler, as we need to send it to the nrf_adv_conn module and the rbc_mesh */
    softdevice_sys_evt_handler_set(rbc_mesh_sd_evt_handler);

#ifdef RBC_MESH_SERIAL

    /* only want to enable serial interface, and let external host setup the framework */
    mesh_aci_init();

	  rbc_mesh_init_params_t init_params;

    init_params.access_addr = MESH_ACCESS_ADDR;
    init_params.interval_min_ms = MESH_INTERVAL_MIN_MS;
    init_params.channel = MESH_CHANNEL;
    init_params.lfclksrc = MESH_CLOCK_SOURCE;

    uint32_t error_code = rbc_mesh_init(init_params);
    APP_ERROR_CHECK(error_code);

    /* request values for both LEDs on the mesh */
    for (uint32_t i = 0; i < 32; ++i)
    {
        error_code = rbc_mesh_value_enable(i);
        APP_ERROR_CHECK(error_code);
    }

    /* init BLE gateway softdevice application: */
    nrf_adv_conn_init();

#else

    rbc_mesh_init_params_t init_params;

    init_params.access_addr = MESH_ACCESS_ADDR;
    init_params.interval_min_ms = MESH_INTERVAL_MIN_MS;
    init_params.channel = MESH_CHANNEL;
    init_params.lfclksrc = MESH_CLOCK_SOURCE;

    uint32_t error_code = rbc_mesh_init(init_params);
    APP_ERROR_CHECK(error_code);

    /* request values for both LEDs on the mesh */
    for (uint32_t i = 0; i < 32; ++i)
    {
        error_code = rbc_mesh_value_enable(i);
        APP_ERROR_CHECK(error_code);
    }

    /* init BLE gateway softdevice application: */
    nrf_adv_conn_init();

#endif
    NRF_GPIO->OUTCLR = (1 << 4);

#if !(defined(BUTTONS))
    /* fetch events */
    rbc_mesh_event_t evt;
    while (true)
    {
        if (rbc_mesh_event_get(&evt) == NRF_SUCCESS)
        {
						#ifdef DEBUG
						printf("rbc_mesh_event_get(1): evt popped from g_rbc_event_fifo \n");
						#endif
            rbc_mesh_event_handler(&evt);
            rbc_mesh_packet_release(evt.data);
						#ifdef DEBUG
						printf("after packet_release \n");
						#endif
						//#ifdef CUSTOM_BOARD_SERIAL
							free(evt.data);
						#ifdef DEBUG
							printf("freed evt.data \n");
						#endif
						memset(&evt, 0, sizeof(evt));
					//printf("rbc_mesh_event_get(1): evt popped from g_rbc_event_fifo \n");
        }
        //TODO: optional - wrap in ifdef for HVAC board
        //update analog values
        int analog_val = 0;
				//Digital Variable Define
            bool flag=0;
            //int Dig_in_pin= Dig_PIN_1;
            //int Dig_val_saver;
            int Dig_out_1;
            int Dig_out_2;

        //Variable Define

        //int Ana_output;
        static uint8_t gas_read;
        static uint8_t temp_read;
        //TODO: get GPIO value on pin 5 to analog_val
				/*******************Analog Read**********************/
        //C_ana_output_1=int2byte(analog_read(5));
        //C_ana_output_2=int2byte(analog_read(6));
				//gas_read=(uint8_t)analog_read(ADC_CONFIG_PSEL_AnalogInput6);
				//temp_read = (uint8_t)analog_read(ADC_CONFIG_PSEL_AnalogInput7);
				#ifdef DEBUG
					//printf("analog read is: %d \n", gas_read);
					//printf("temperature is: %d \n", temp_read);
				#endif

        //TODO: convert data value to data
        //uint8_t data1;

				// Pins 3 & 4 are digital read
				//mesh_update(3, &data, 1);
				//mesh_update(4, &data, 1);

				// Pins 5 & 6 are analog read
				//mesh_update(5, &gas_read, 1);
        //mesh_update(6, &temp_read, 1);

        sd_app_evt_wait();
    }
#else
    uint8_t mesh_data[16] = {0,0};
    rbc_mesh_event_t evt;
    while (true)
    {
        // red off
        if(nrf_gpio_pin_read(BUTTON_1) == 0)
        {
            while(nrf_gpio_pin_read(BUTTON_1) == 0);
            mesh_data[0] = 0;
            rbc_mesh_value_set(0, mesh_data, 1);
            led_config(0, 0);
        }
        // red on
        if(nrf_gpio_pin_read(BUTTON_2) == 0)
        {
            while(nrf_gpio_pin_read(BUTTON_2) == 0);
            mesh_data[0] = 1;
            rbc_mesh_value_set(0, mesh_data, 1);
            led_config(0, 1);
        }
        // green off
        if(nrf_gpio_pin_read(BUTTON_3) == 0)
        {
            while(nrf_gpio_pin_read(BUTTON_3) == 0);
            mesh_data[0] = 0;
            rbc_mesh_value_set(1, mesh_data, 1);
            led_config(1, 0);
        }
        // green on
        if(nrf_gpio_pin_read(BUTTON_4) == 0)
        {
            while(nrf_gpio_pin_read(BUTTON_4) == 0);
            mesh_data[0] = 1;
            rbc_mesh_value_set(1, mesh_data, 1);
            led_config(1, 1);
        }

        if (rbc_mesh_event_get(&evt) == NRF_SUCCESS)
        {
					#ifdef DEBUG
					printf("rbc_mesh_event_get (2): evt popped from g_rbc_event_fifo \n");
					#endif
            rbc_mesh_event_handler(&evt);
            rbc_mesh_packet_release(evt.data);
        }
    }
#endif

}
