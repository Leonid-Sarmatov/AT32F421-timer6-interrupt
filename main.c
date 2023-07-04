/**
  **************************************************************************
  * @file     main.c
  * @brief    main program
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to 
  * download from Artery official website is the copyrighted work of Artery. 
  * Artery authorizes customers to use, copy, and distribute the BSP 
  * software and its related documentation for the purpose of design and 
  * development in conjunction with Artery microcontrollers. Use of the 
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
#include "at32f421_conf.h"
#include "at32f421_gpio.h"
#include "at32f421.h"
#include "RTE_Components.h"

/*data*/
// Numbers for 7-seg indicator
uint8_t numbers[10] = {
		0b00111111,
		0b00000110,
		0b01011011,
		0b01001111,
		0b01100110,
		0b01101101,
		0b01111101,
		0b00000111,
		0b01111111,  
		0b01101111  
	};

// Timer speed values
uint8_t timer_speed = 0;

// Values
uint8_t counter;
uint32_t i;
uint16_t e = 100;
/*******/

	
/*prototypes*/
void printNumber(uint8_t array[], uint8_t number);
void timer_init();
/************/
	

/*functions*/
void gpio_set(gpio_type *PORT, uint32_t PIN, gpio_drive_type DRIVE, gpio_mode_type MODE, gpio_output_type OUT_TYPE, gpio_pull_type PULL ) {


    gpio_init_type pinx;

    gpio_init_type *pina = &pinx;

    pinx.gpio_drive_strength= DRIVE;
    pinx.gpio_mode =MODE;
    pinx.gpio_out_type=OUT_TYPE;
    pinx.gpio_pins = PIN;
    pinx.gpio_pull = PULL;

    gpio_init( PORT,pina);

}

void hardware_init() {


    wdt_register_write_enable(TRUE);
    wdt_divider_set(WDT_CLK_DIV_8);
    wdt_register_write_enable(FALSE);

    crm_hick_sclk_frequency_select(CRM_HICK_SCLK_8MHZ);
    crm_clock_source_enable (CRM_CLOCK_SOURCE_HICK,TRUE);

    crm_hick_divider_select(CRM_HICK48_NODIV);
    crm_ahb_div_set(CRM_AHB_DIV_1);


    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK,TRUE);
		/* reset crm */
  crm_reset();

  /* config flash psr register */
  flash_psr_set(FLASH_WAIT_CYCLE_1);

  /* enable hick */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HICK, TRUE);

   /* wait till hick is ready */
  while(crm_flag_get(CRM_HICK_STABLE_FLAG) != SET)
  {
  }

  /* config ahbclk */
  crm_ahb_div_set(CRM_AHB_DIV_1);

  /* config apb2clk */
  crm_apb2_div_set(CRM_APB2_DIV_1);

  /* config apb1clk */
  crm_apb1_div_set(CRM_APB1_DIV_1);

  /* config sclk from hick48 */
  crm_hick_sclk_frequency_select(CRM_HICK_SCLK_48MHZ);

  /* select hick as system clock source */
  crm_sysclk_switch(CRM_SCLK_HICK);

  /* wait till hick is used as system clock source */
  while(crm_sysclk_switch_status_get() != CRM_SCLK_HICK)
  {
  }

  /* update system_core_clock global variable */
  system_core_clock_update();
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK,TRUE);
  //crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK,TRUE);
  crm_periph_clock_enable(CRM_TMR6_PERIPH_CLOCK,TRUE);


    gpio_set(GPIOA,
             GPIO_PINS_0,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);

    gpio_set(GPIOA,
             GPIO_PINS_1,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);

    gpio_set(GPIOA,
             GPIO_PINS_2,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);

    gpio_set(GPIOA,
             GPIO_PINS_3,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);

    gpio_set(GPIOA,
             GPIO_PINS_4,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);

    gpio_set(GPIOA,
             GPIO_PINS_5,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);

    gpio_set(GPIOA,
             GPIO_PINS_6,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);

    gpio_set(GPIOB,
             GPIO_PINS_1,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_INPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_UP);
		/************/

    timer_init();
}

void timer_init() {
    nvic_irq_enable(TMR6_GLOBAL_IRQn,35,36);
    TMR6->iden_bit.ovfien =1;
    TMR6->ctrl1_bit.ocmen = 0;
    TMR6->ctrl1_bit.ovfen = 0;
		TMR6->ctrl1_bit.prben = 1;
    
		TMR6->div_bit.div = 48000-1;
		TMR6->pr_bit.pr = 1000;
    
		TMR6->ctrl1_bit.tmren = 1;
}

void printNumber(uint8_t array[], uint8_t number) {
	
	for (uint8_t i = 0; i < 8; i++) {
		if (array[number] & (1<<i)) {
			gpio_bits_set(GPIOA, 1<<i);
		} else {
			gpio_bits_reset(GPIOA, 1<<i);
		}
	}
}

void TMR6_GLOBAL_IRQHandler(void) {
		counter++;
		if (counter == 255) {
			counter=0;
		}
		
		printNumber(numbers, counter%10);	
    TMR6 ->ists_bit.ovfif =0;
}

void buttonRead() {
	if (gpio_input_data_bit_read(GPIOB, GPIO_PINS_1) == RESET) {
		
		while (gpio_input_data_bit_read(GPIOB, GPIO_PINS_1) == RESET) {
			i = 4000000;
			while(i != 0) i--;
		}
		
		timer_speed = (timer_speed > 2) ? 0 : timer_speed+1;
		
		if (timer_speed == 0) TMR6->pr_bit.pr = 1000;
		if (timer_speed == 1) TMR6->pr_bit.pr = 300;
		if (timer_speed == 2) TMR6->pr_bit.pr = 50;
		
		i = 4000000000;
		while(i != 0) i--;
		
	}
}
/***********/

/**
  * @brief  main function.
  * @param  none
  * @retval none
  */
int main(void)
{
	hardware_init();
  while(1)
  {
		buttonRead();
  }
}

