#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "ir_nec_encoder.h"

#define IR_RESOLUTION_HZ             1000000 // 1MHz resolution, 1 tick = 1us
#define IR_TX_GPIO_NUM               14
#define IR_RX_GPIO_NUM               15
#define IR_NEC_DECODE_MARGIN         200     // Tolerance for parsing RMT symbols into bit stream

static const char *TAG = "AC_IR_Control";

// Structure to store decoded IR commands
typedef struct {
    uint16_t address;
    uint16_t command;
} ir_command_t;

#define COMMANDS_MAX_NUM 10
static ir_command_t s_stored_commands[COMMANDS_MAX_NUM];
static int s_command_count = 0;

static uint16_t s_nec_code_address;
static uint16_t s_nec_code_command;

static inline bool nec_check_in_range(uint32_t signal_duration, uint32_t spec_duration) {
    return (signal_duration < (spec_duration + IR_NEC_DECODE_MARGIN)) &&
           (signal_duration > (spec_duration - IR_NEC_DECODE_MARGIN));
}

static bool nec_parse_logic0(rmt_symbol_word_t *rmt_nec_symbols) {
    return nec_check_in_range(rmt_nec_symbols->duration0, 560) &&
           nec_check_in_range(rmt_nec_symbols->duration1, 560);
}

static bool nec_parse_logic1(rmt_symbol_word_t *rmt_nec_symbols) {
    return nec_check_in_range(rmt_nec_symbols->duration0, 560) &&
           nec_check_in_range(rmt_nec_symbols->duration1, 1690);
}

static bool nec_parse_frame(rmt_symbol_word_t *rmt_nec_symbols) {
    rmt_symbol_word_t *cur = rmt_nec_symbols;
    uint16_t address = 0;
    uint16_t command = 0;
    bool valid_leading_code = nec_check_in_range(cur->duration0, 9000) &&
                              nec_check_in_range(cur->duration1, 4500);
    if (!valid_leading_code) {
        return false;
    }
    cur++;
    for (int i = 0; i < 16; i++) {
        if (nec_parse_logic1(cur)) {
            address |= 1 << i;
        } else if (nec_parse_logic0(cur)) {
            address &= ~(1 << i);
        } else {
            return false;
        }
        cur++;
    }
    for (int i = 0; i < 16; i++) {
        if (nec_parse_logic1(cur)) {
            command |= 1 << i;
        } else if (nec_parse_logic0(cur)) {
            command &= ~(1 << i);
        } else {
            return false;
        }
        cur++;
    }
    // save address and command
    s_nec_code_address = address;
    s_nec_code_command = command;
    return true;
}

static bool nec_parse_frame_repeat(rmt_symbol_word_t *rmt_nec_symbols) {
    return nec_check_in_range(rmt_nec_symbols->duration0, 9000) &&
           nec_check_in_range(rmt_nec_symbols->duration1, 2250);
}

static void example_parse_nec_frame(rmt_symbol_word_t *rmt_nec_symbols, size_t symbol_num) {
    ESP_LOGI(TAG, "NEC frame received");
    switch (symbol_num) {
    case 34: // NEC normal frame
        if (nec_parse_frame(rmt_nec_symbols)) {
            ESP_LOGI(TAG, "Address=%04X, Command=%04X", s_nec_code_address, s_nec_code_command);
            if (s_command_count < COMMANDS_MAX_NUM) {
                s_stored_commands[s_command_count].address = s_nec_code_address;
                s_stored_commands[s_command_count].command = s_nec_code_command;
                s_command_count++;
            } else {
                ESP_LOGW(TAG, "Command storage full");
            }
        }
        break;
    case 2: // NEC repeat frame
        if (nec_parse_frame_repeat(rmt_nec_symbols)) {
            ESP_LOGI(TAG, "Address=%04X, Command=%04X (repeat)", s_nec_code_address, s_nec_code_command);
        }
        break;
    default:
        ESP_LOGW(TAG, "Unknown NEC frame");
        break;
    }
}

static bool example_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data) {
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

void app_main(void) {
    ESP_LOGI(TAG, "create RMT RX channel");
    rmt_rx_channel_config_t rx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = IR_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .gpio_num = IR_RX_GPIO_NUM,
    };
    rmt_channel_handle_t rx_channel = NULL;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_cfg, &rx_channel));

    ESP_LOGI(TAG, "register RX done callback");
    QueueHandle_t receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    assert(receive_queue);
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = example_rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, receive_queue));

    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 1250,
        .signal_range_max_ns = 12000000,
    };

    ESP_LOGI(TAG, "create RMT TX channel");
    rmt_tx_channel_config_t tx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = IR_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
        .gpio_num = IR_TX_GPIO_NUM,
    };
    rmt_channel_handle_t tx_channel = NULL;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_channel_cfg, &tx_channel));

    ESP_LOGI(TAG, "modulate carrier to TX channel");
    rmt_carrier_config_t carrier_cfg = {
        .duty_cycle = 0.33,
        .frequency_hz = 38000, // 38KHz
    };
    ESP_ERROR_CHECK(rmt_apply_carrier(tx_channel, &carrier_cfg));

    rmt_transmit_config_t transmit_config = {
        .loop_count = 0,
    };

    ESP_LOGI(TAG, "install IR NEC encoder");
    ir_nec_encoder_config_t nec_encoder_cfg = {
        .resolution = IR_RESOLUTION_HZ,
    };
    rmt_encoder_handle_t nec_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_ir_nec_encoder(&nec_encoder_cfg, &nec_encoder));

    ESP_LOGI(TAG, "enable RMT TX and RX channels");
    ESP_ERROR_CHECK(rmt_enable(tx_channel));
    ESP_ERROR_CHECK(rmt_enable(rx_channel));

    rmt_symbol_word_t raw_symbols[64];
    rmt_rx_done_event_data_t rx_data;
    ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));

    while (1) {
        if (xQueueReceive(receive_queue, &rx_data, pdMS_TO_TICKS(1000)) == pdPASS) {
            example_parse_nec_frame(rx_data.received_symbols, rx_data.num_symbols);
            ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
        } else if (s_command_count > 0) {
            // Transmit the last stored command (example logic)
            ir_command_t *command_to_send = &s_stored_commands[s_command_count - 1];
            const ir_nec_scan_code_t scan_code = {
                .address = command_to_send->address,
                .command = command_to_send->command,
            };
            ESP_ERROR_CHECK(rmt_transmit(tx_channel, nec_encoder, &scan_code, sizeof(scan_code), &transmit_config));
            vTaskDelay(pdMS_TO_TICKS(2000)); // Delay between transmissions
        }
    }
}
