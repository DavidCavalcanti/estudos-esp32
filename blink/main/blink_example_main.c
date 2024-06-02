#include <stdio.h> /* Inclui a biblioteca padrão de entrada/saída do C. */
#include "freertos/FreeRTOS.h" /* iblioteca principal do FreeRTOS, um sistema operacional de tempo real usado no ESP-IDF. */
#include "freertos/task.h" /* Inclui a biblioteca do FreeRTOS que lida com tarefas */
#include "driver/gpio.h" /* Inclui a biblioteca de driver de GPIO, que permite o controle dos pinos GPIO. */
#include "esp_log.h" /*  Inclui a biblioteca para funções de logging, útil para depuração */
#include "sdkconfig.h" /* Inclui a configuração do SDK (Software Development Kit) do ESP-IDF */

/* Define uma string constante para ser usada nos logs, identificando a origem das mensagens de log. */
static const char *TAG = "example";

/* Define o pino GPIO 2 como BLINK_GPIO, onde o LED está conectado */
#define BLINK_GPIO 2

/* Define uma variável estática para armazenar o estado do LED (1 para ligado, 0 para desligado). */
static uint8_t s_led_state = 1;

static void blink_led(void)
{
    /* Defina o nível GPIO de acordo com o estado (BAIXO ou ALTO)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

/* Função para configurar o pino GPIO */
static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    /* Reseta a configuração do pino GPIO especificado. */
    gpio_reset_pin(BLINK_GPIO);
    /* Defina o GPIO como uma saída push/pull */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

void app_main(void)
{    
    configure_led();

    while (1) {
        ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        blink_led();
        /* Alternar o estado do LED */
        s_led_state = !s_led_state;
        vTaskDelay(750 / portTICK_PERIOD_MS);
    }
}
