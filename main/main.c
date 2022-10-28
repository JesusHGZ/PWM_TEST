#include <stdio.h>
#include <driver/ledc.h>
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


// uint32_t duty_1 = 2868;
// uint32_t duty_2 = 1228;
ledc_channel_config_t pwm_1;
ledc_channel_config_t pwm_2;

void init_pwm(void);

void app_main(void)
{
    init_pwm();
    while (1)
    {
        ledc_set_duty(pwm_1.speed_mode, pwm_1.channel, pwm_1.duty);
        ledc_update_duty(pwm_1.speed_mode, pwm_1.channel);

        vTaskDelay(200 / portTICK_PERIOD_MS);

        ledc_set_duty(pwm_2.speed_mode, pwm_2.channel, pwm_2.duty);
        ledc_update_duty(pwm_2.speed_mode, pwm_2.channel);

        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

void init_pwm(void)
{
    ledc_timer_config_t timer_1 = {
        .duty_resolution = LEDC_TIMER_12_BIT,
        .freq_hz = 240,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };

     ledc_timer_config_t timer_2 = {
        .duty_resolution = LEDC_TIMER_12_BIT,
        .freq_hz = 240,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ledc_timer_config(&timer_1);
    ledc_timer_config(&timer_2);

    pwm_1.channel = LEDC_CHANNEL_0;
    pwm_1.duty = 2868;
    pwm_1.gpio_num = GPIO_NUM_18;
    pwm_1.hpoint = 0;
    pwm_1.timer_sel = LEDC_TIMER_0;
    pwm_1.speed_mode = LEDC_HIGH_SPEED_MODE;

    pwm_2.channel = LEDC_CHANNEL_1;
    pwm_2.duty = 1228;
    pwm_2.gpio_num = GPIO_NUM_19;
    pwm_2.hpoint = 0;
    pwm_2.timer_sel = LEDC_TIMER_1;
    pwm_2.speed_mode = LEDC_HIGH_SPEED_MODE;

    ledc_channel_config(&pwm_1);
    ledc_channel_config(&pwm_2);
    return ESP_OK;
}