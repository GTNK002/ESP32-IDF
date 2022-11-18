#include <stdio.h>  // ADC input
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_types.h"
#include "freertos/queue.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"

static esp_adc_cal_characteristics_t adc1_chars;

float checksec = 5.000;

#define TIMER_INTR_SEL TIMER_INTR_LEVEL  /*!< Timer level interrupt */
#define TIMER_GROUP    TIMER_GROUP_0     /*!< Test on timer group 0 */
#define TIMER_DIVIDER   80               /*!< Hardware timer clock divider, 80 to get 1MHz clock to timer */
#define TIMER_SCALE    (TIMER_BASE_CLK / TIMER_DIVIDER)  /*!< used to calculate counter value */
#define TIMER_FINE_ADJ   (0*(TIMER_BASE_CLK / TIMER_DIVIDER)/1000000) /*!< used to compensate alarm value */
#define TIMER_INTERVAL0_SEC   (checksec)   /*!< test interval for timer 0 */

void check_adc_value()
{
    int adc_value = adc1_get_raw(ADC1_CHANNEL_4);
    if(adc_value > 500) {
        while (1) 
        {
            gpio_set_level(2,0);
            vTaskDelay(200/portTICK_RATE_MS);
            gpio_set_level(2,1);
            vTaskDelay(200/portTICK_RATE_MS);
        }
    }
    else{ 
        while (1) 
        {
            gpio_set_level(2,0);
            vTaskDelay(1000/portTICK_RATE_MS);
            gpio_set_level(2,1);
            vTaskDelay(1000/portTICK_RATE_MS);
        }
    }
}

volatile int cnt = 0;
void IRAM_ATTR timer_group0_isr(void *para){// timer group 0, ISR
    int timer_idx = (int) para;
     uint32_t intr_status = TIMERG0.int_st_timers.val;
      if((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0) {
          TIMERG0.hw_timer[timer_idx].update = 1;
          TIMERG0.int_clr_timers.t0 = 1;
          TIMERG0.hw_timer[timer_idx].config.alarm_en = 1;
          //gpio_set_level(GPIO_NUM_16,cnt%2);
          if(cnt%2 == 1){
            check_adc_value();
          }
          else{
            check_adc_value();
          }
          cnt++;
      }
}

static void tg0_timer0_init()
{
    int timer_group = TIMER_GROUP_0;
    int timer_idx = TIMER_0;
    timer_config_t config;
    config.alarm_en = 1;
    config.auto_reload = 1;
    config.counter_dir = TIMER_COUNT_UP;
    config.divider = TIMER_DIVIDER;
    config.intr_type = TIMER_INTR_SEL;
    config.counter_en = TIMER_PAUSE;
    /*Configure timer*/
    timer_init(timer_group, timer_idx, &config);
    /*Stop timer counter*/
    timer_pause(timer_group, timer_idx);
    /*Load counter value */
    timer_set_counter_value(timer_group, timer_idx, 0x00000000ULL);
    /*Set alarm value*/
    timer_set_alarm_value(timer_group, timer_idx, (TIMER_INTERVAL0_SEC * TIMER_SCALE) - TIMER_FINE_ADJ);
    /*Enable timer interrupt*/
    timer_enable_intr(timer_group, timer_idx);
    /*Set ISR handler*/
    timer_isr_register(timer_group, timer_idx, timer_group0_isr, (void*) timer_idx, ESP_INTR_FLAG_IRAM, NULL);
    /*Start timer counter*/
    timer_start(timer_group, timer_idx);
}



void app_main(void)
{
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
    adc1_config_width(ADC_WIDTH_BIT_DEFAULT);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11); // pin D33 || GPIO33
    gpio_set_direction(GPIO_NUM_16,GPIO_MODE_OUTPUT);
    gpio_set_direction(2,GPIO_MODE_OUTPUT); // declare inbuilt pin out
    tg0_timer0_init();

    while (1) 
    {
        check_adc_value();

    }
}
