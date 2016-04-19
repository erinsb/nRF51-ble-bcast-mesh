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
#include "timeslot_handler.h"
#include "mesh_aci.h"

#include "softdevice_handler.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "simple_uart.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* Debug macros for debugging with logic analyzer */
#define SET_PIN(x) NRF_GPIO->OUTSET = (1 << (x))
#define CLEAR_PIN(x) NRF_GPIO->OUTCLR = (1 << (x))
#define TICK_PIN(x) do { SET_PIN((x)); CLEAR_PIN((x)); }while(0)

#define MESH_ACCESS_ADDR        (0xA541A68F)
#define MESH_INTERVAL_MIN_MS    (100)
#define MESH_CHANNEL            (38)
#define MESH_CLOCK_SRC          (NRF_CLOCK_LFCLKSRC_XTAL_75_PPM)

//Available pin configurations
typedef enum{
  PIN_UNSET, //pin is not connected to device
  PIN_OUTPUT, //pin is set to output
  PIN_DINPUT, //pin is set to digital input
  PIN_AINPUT, //pin is set to analog input
} pin_config;
//array of pin configurations
pin_config pin_configs[32];

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE * p_file)
{
    simple_uart_put((uint8_t)ch);
    return 0;
}

void retarget_init(void)
{
    simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, HWFC);
}

void sd_ble_evt_handler(ble_evt_t* p_ble_evt) {
	rbc_mesh_ble_evt_handler(p_ble_evt);
	nrf_adv_conn_evt_handler(p_ble_evt);
}

/**
* @brief General error handler.
*/
static void error_loop(void)
{
	#ifdef DEBUG
	printf("error \n");
	#endif
    SET_PIN(7);
    while (true)
    {
        __WFE();
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

static void set_pin(int pin, int set){
  if(set) SET_PIN(pin);
  else CLEAR_PIN(pin);
  #ifdef DEBUG
  printf("Pin %d set to %d", pin, set);
  #endif
}

int analog_read(int pin_num)

{
    uint16_t adc_result;
    NRF_ADC->INTENSET = (ADC_INTENSET_END_Disabled << ADC_INTENSET_END_Pos);     /*!< Interrupt enabled. */
    // config ADC
    //*** pin_num= ADC_CONFIG_PSEL_AnalogInputX-1
    NRF_ADC->CONFIG = (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos) /*!< Analog external reference inputs disabled. */
                    | ((1 << pin_num) << ADC_CONFIG_PSEL_Pos)
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

int Pulsemeasure(int Pin_Num)
{

    int count=0;
    int output=0;
    bool flag;
    int pin_val= nrf_gpio_pin_read(Pin_Num);

    if(pin_val==1)
    {
        flag=1;
    }
    while(flag)
    {
        pin_val= nrf_gpio_pin_read(Pin_Num);
        count++;
        if(pin_val!=1)
        {
          flag=0;
          output=count;
          count=0;
        }
    }

        return output;
}

static void ping_handle(uint16_t handle){
  uint16_t pin = handle - 32;
  uint8_t val;
  if(pin <= 6){
    val = analog_read(pin);
  } else{
    val = Pulsemeasure(pin);
  }
  #ifdef DEBUG
    printf("Ping received: Updated pin %d with value = %d \n",pin,val);
  #endif

  rbc_mesh_value_set(pin,&val, 1);
  uint8_t tmp = 0;
  rbc_mesh_value_set(pin+32,&tmp,1);
}

/**
* @brief RBC_MESH framework event handler. Defined in rbc_mesh.h. Handles
*   events coming from the mesh.
*
* @param[in] evt RBC event propagated from framework
*/
static void rbc_mesh_event_handler(rbc_mesh_event_t* evt)
{
		#ifdef DEBUG
		printf("rbc_mesh_event_handler\n");
		#endif

    #ifdef RECEIVER
    printf("Event type: ");
    switch (evt->event_type) {
      case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL:
        printf("CONFLICTING_VAL\n");
        break;
      case RBC_MESH_EVENT_TYPE_NEW_VAL:
        printf("NEW_VAL\n");
        break;
      case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
        printf("UPDATE_VAL\n");
        break;
      case RBC_MESH_EVENT_TYPE_TX:
        printf("TX\n");
        break;
      case RBC_MESH_EVENT_TYPE_INITIALIZED:
        printf("INITIALIZED\n");
        break;
    }
    printf("Handle: %d; ",evt->value_handle);
    printf("Data Len: %d\n",evt->data_len);
    int i;
    printf("Data (hex): ")
    for(i = 0; i < data_len; i++){
      printf("%x, ",data[i]);
    }
    printf("\n\n");
    #endif

    switch (evt->event_type)
    {
        case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL:
        case RBC_MESH_EVENT_TYPE_NEW_VAL:
        case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
          if (evt->value_handle >= 32){
            if(evt->value_handle < 64 && evt->data[0] == 1) { //else do nothing
							#ifndef RBC_MESH_SERIAL
              if(pin_configs[evt->value_handle - 32] == PIN_AINPUT || pin_configs[evt->value_handle - 32] == PIN_DINPUT)
                ping_handle(evt->value_handle);
							#endif
						}
          } else{
            if(pin_configs[evt->value_handle] == PIN_OUTPUT)
              set_pin(evt->value_handle, evt->data[0]);
          }
          break;
        case RBC_MESH_EVENT_TYPE_TX:
        case RBC_MESH_EVENT_TYPE_INITIALIZED:
            break;
    }
}


static void pin_init(int pin, pin_config cfg){
  pin_configs[pin] = cfg;
  switch (cfg){
    case PIN_UNSET:
      break;
    case PIN_OUTPUT:
      nrf_gpio_cfg_output(pin);
      break;
    case PIN_AINPUT:
      nrf_gpio_cfg_input(pin, NRF_GPIO_PIN_NOPULL);
      break;
    case PIN_DINPUT:
      nrf_gpio_cfg_input(pin, NRF_GPIO_PIN_PULLDOWN);
  }
}


/** @brief main function */
int main(void)
{
    #ifdef DEBUG
      retarget_init();
      printf("Start...\r\n");
    #endif
    int i;
    for(i = 0; i < 32; i++) pin_configs[i] = PIN_UNSET;

    /* init each device */
    #ifdef RBC_MESH_SERIAL
      mesh_aci_init();
    #endif
    #ifdef SMART_HVAC
      pin_init(5,PIN_AINPUT);
      pin_init(6,PIN_AINPUT);
      pin_init(17,PIN_OUTPUT);
      pin_init(18,PIN_OUTPUT);
      pin_init(19,PIN_OUTPUT);
      pin_init(20,PIN_OUTPUT);
    #endif
    #ifdef SMART_SWITCH
      pin_init(21,PIN_OUTPUT);
      pin_init(22,PIN_OUTPUT);
      pin_init(23,PIN_OUTPUT);
      pin_init(24,PIN_OUTPUT);
    #endif
    #ifdef SMART_VALVE
      pin_init(30,PIN_OUTPUT);
      pin_init(3, PIN_DINPUT);
    #endif


    /* Enable Softdevice (including sd_ble before framework */
    SOFTDEVICE_HANDLER_INIT(MESH_CLOCK_SRC, NULL);
    softdevice_ble_evt_handler_set(sd_ble_evt_handler);
    softdevice_sys_evt_handler_set(rbc_mesh_sd_evt_handler);


    rbc_mesh_init_params_t init_params;

    init_params.access_addr = MESH_ACCESS_ADDR;
    init_params.interval_min_ms = MESH_INTERVAL_MIN_MS;
    init_params.channel = MESH_CHANNEL;
    init_params.lfclksrc = MESH_CLOCK_SRC;

    uint32_t error_code;
    error_code = rbc_mesh_init(init_params);
    APP_ERROR_CHECK(error_code);

    error_code = rbc_mesh_value_enable(1);
    APP_ERROR_CHECK(error_code);
    error_code = rbc_mesh_value_enable(2);
    APP_ERROR_CHECK(error_code);

    for (uint32_t i = 0; i < 64; ++i)
    {
        error_code = rbc_mesh_value_enable(i);
        APP_ERROR_CHECK(error_code);
    }

    rbc_mesh_event_t evt;
    while (true)
    {
        if (rbc_mesh_event_get(&evt) == NRF_SUCCESS)
        {
            rbc_mesh_event_handler(&evt);
            rbc_mesh_packet_release(evt.data);
            free(evt.data);
            memset(&evt,0,sizeof(evt));
        }

        sd_app_evt_wait();
    }
}
