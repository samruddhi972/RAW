// Master as a receiver for SPI communication

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/igmp.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "soc/rtc_periph.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_spi_flash.h"

#include "driver/gpio.h"
#include "esp_intr_alloc.h"

// Pins in use
#define GPIO_MOSI 12
#define GPIO_MISO 13
#define GPIO_SCLK 15
#define GPIO_CS 14

// Main application
void app_main(void)
{
    spi_device_handle_t handle;

    // Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1};

    // Configuration for the SPI device on the other side of the bus
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .clock_speed_hz = 5000000,
        .duty_cycle_pos = 128, // 50% duty cycle
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .cs_ena_posttrans = 3, // Keep the CS low 3 cycles after transaction
        .queue_size = 3};

    spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(HSPI_HOST, &devcfg, &handle);

    char recvbuf[129] = "";
    memset(recvbuf, 0, 33);
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    printf("Master input:\n");
    while (1)
    {
        t.length = 128 * 8;
        t.rx_buffer = recvbuf;
        spi_device_transmit(handle, &t);
        printf("Received: %s\n", recvbuf);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// new
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_hidd_api.h"

static const char *TAG = "MASTER_ESP32";

// SPI pins
#define GPIO_MOSI 23
#define GPIO_MISO 19
#define GPIO_SCLK 18
#define GPIO_CS 5

// MDDS10 Driver 1 pins
#define MOTOR1_PWM_PIN 25
#define MOTOR1_DIR_PIN 26

// MDDS10 Driver 2 pins
#define MOTOR2_PWM_PIN 27
#define MOTOR2_DIR_PIN 14

// PWM configuration
#define PWM_FREQUENCY 1000
#define PWM_RESOLUTION LEDC_TIMER_10_BIT
#define PWM_MAX_DUTY 1023

// SPI communication structure
typedef struct
{
    float setpoint_motor1;
    float setpoint_motor2;
    uint8_t command;
} spi_tx_data_t;

typedef struct
{
    float pid_output_motor1;
    float pid_output_motor2;
    float encoder_pos_motor1;
    float encoder_pos_motor2;
    uint8_t status;
} spi_rx_data_t;

// Global variables
static spi_device_handle_t spi_handle;
static QueueHandle_t ps4_queue;

// PS4 controller data structure
typedef struct
{
    int16_t left_stick_x;
    int16_t left_stick_y;
    int16_t right_stick_x;
    int16_t right_stick_y;
    uint16_t buttons;
} ps4_data_t;

// PWM setup for MDDS10 drivers
void setup_pwm(void)
{
    // Timer configuration
    ledc_timer_config_t timer_conf = {
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQUENCY,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_conf);

    // Motor 1 PWM channel
    ledc_channel_config_t motor1_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = MOTOR1_PWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
        .hpoint = 0,
    };
    ledc_channel_config(&motor1_channel);

    // Motor 2 PWM channel
    ledc_channel_config_t motor2_channel = {
        .channel = LEDC_CHANNEL_1,
        .duty = 0,
        .gpio_num = MOTOR2_PWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
        .hpoint = 0,
    };
    ledc_channel_config(&motor2_channel);

    // Direction pins
    gpio_config_t dir_conf = {
        .pin_bit_mask = (1ULL << MOTOR1_DIR_PIN) | (1ULL << MOTOR2_DIR_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&dir_conf);
}

// Control motor with PID output
void control_motor(int motor_num, float pid_output)
{
    uint32_t duty = abs((int)(pid_output * PWM_MAX_DUTY / 100.0)); // Assuming PID output is -100 to +100
    duty = (duty > PWM_MAX_DUTY) ? PWM_MAX_DUTY : duty;

    if (motor_num == 1)
    {
        // Set direction
        gpio_set_level(MOTOR1_DIR_PIN, (pid_output >= 0) ? 1 : 0);
        // Set PWM duty
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
    else if (motor_num == 2)
    {
        // Set direction
        gpio_set_level(MOTOR2_DIR_PIN, (pid_output >= 0) ? 1 : 0);
        // Set PWM duty
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    }
}

// SPI setup
void setup_spi(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = sizeof(spi_rx_data_t),
    };

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .clock_speed_hz = 1000000, // 1MHz
        .duty_cycle_pos = 128,
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle));
}

// SPI communication with slave
esp_err_t spi_communicate(spi_tx_data_t *tx_data, spi_rx_data_t *rx_data)
{
    spi_transaction_t t = {
        .length = sizeof(spi_tx_data_t) * 8,
        .tx_buffer = tx_data,
        .rxlength = sizeof(spi_rx_data_t) * 8,
        .rx_buffer = rx_data,
    };

    return spi_device_transmit(spi_handle, &t);
}

// PS4 Controller callback (simplified - you'll need proper PS4 library)
void ps4_callback(void *param)
{
    // This is a placeholder - you'll need to implement actual PS4 connection
    // using libraries like ESP32-PS4Controller or similar
    ps4_data_t ps4_data = {0};

    while (1)
    {
        // Simulate PS4 input (replace with actual PS4 reading)
        ps4_data.left_stick_y = 0;  // Forward/backward
        ps4_data.right_stick_x = 0; // Turn left/right

        xQueueSend(ps4_queue, &ps4_data, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz update rate
    }
}

// Main control task
void control_task(void *param)
{
    spi_tx_data_t tx_data = {0};
    spi_rx_data_t rx_data = {0};
    ps4_data_t ps4_data = {0};

    while (1)
    {
        // Get PS4 controller input
        if (xQueueReceive(ps4_queue, &ps4_data, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            // Convert joystick to motor setpoints (differential drive)
            float forward = ps4_data.left_stick_y / 32768.0 * 100.0; // -100 to +100
            float turn = ps4_data.right_stick_x / 32768.0 * 50.0;    // -50 to +50

            tx_data.setpoint_motor1 = forward + turn; // Left motor
            tx_data.setpoint_motor2 = forward - turn; // Right motor
            tx_data.command = 1;                      // Normal operation

            // Constrain setpoints
            tx_data.setpoint_motor1 = (tx_data.setpoint_motor1 > 100) ? 100 : tx_data.setpoint_motor1;
            tx_data.setpoint_motor1 = (tx_data.setpoint_motor1 < -100) ? -100 : tx_data.setpoint_motor1;
            tx_data.setpoint_motor2 = (tx_data.setpoint_motor2 > 100) ? 100 : tx_data.setpoint_motor2;
            tx_data.setpoint_motor2 = (tx_data.setpoint_motor2 < -100) ? -100 : tx_data.setpoint_motor2;
        }

        // Communicate with slave ESP32s3
        if (spi_communicate(&tx_data, &rx_data) == ESP_OK)
        {
            // Apply PID outputs to motors
            control_motor(1, rx_data.pid_output_motor1);
            control_motor(2, rx_data.pid_output_motor2);

            ESP_LOGI(TAG, "Motor1: SP=%.2f, PID=%.2f, Pos=%.2f",
                     tx_data.setpoint_motor1, rx_data.pid_output_motor1, rx_data.encoder_pos_motor1);
            ESP_LOGI(TAG, "Motor2: SP=%.2f, PID=%.2f, Pos=%.2f",
                     tx_data.setpoint_motor2, rx_data.pid_output_motor2, rx_data.encoder_pos_motor2);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz control loop
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Master ESP32");

    // Initialize components
    setup_pwm();
    setup_spi();

    // Create queue for PS4 data
    ps4_queue = xQueueCreate(5, sizeof(ps4_data_t));

    //  need proper PS4 library
    // This is just a placeholder structure

    // Start tasks
    xTaskCreate(ps4_callback, "ps4_task", 4096, NULL, 5, NULL);
    xTaskCreate(control_task, "control_task", 4096, NULL, 6, NULL);

    ESP_LOGI(TAG, "Master ESP32 initialized");
}