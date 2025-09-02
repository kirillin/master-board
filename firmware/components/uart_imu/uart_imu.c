#include "uart_imu.h"
#include "driver/uart.h"
#include "esp_intr_alloc.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#define UART_NUM            UART_NUM_1
#define BUF_SIZE            128
#define PIN_TXD             32
#define PIN_RXD             35

#define IMU_PACKET_SIZE     0
#define EF_PACKET_SIZE      38

#define IMU_HEADER_POS      0
#define IMU_ACCX_POS        2   // acc_x: bytes 2-3
#define IMU_ACCY_POS        4
#define IMU_ACCZ_POS        6

#define IMU_GYRX_POS        13  // gyr_x: bytes 13-14
#define IMU_GYRY_POS        15
#define IMU_GYRZ_POS        17

#define IMU_ROLL_POS        24  // pitch: bytes 13-14
#define IMU_PITCH_POS       26
#define IMU_YAW_POS         28

#define IMU_QN_ACC          15
#define IMU_QN_GYR          15
#define IMU_QN_EF           15

static inline int16_t raw_int16_from_byte_array(const uint8_t *buf, size_t pos, size_t buf_len)
{
    if (pos + 1 >= buf_len) return 0;
    return (int16_t)((buf[pos + 1] << 8) | buf[pos]);
}

static inline float float_from_byte_array_qn(const uint8_t *buf, size_t pos, size_t n, size_t buf_len)
{
    int16_t r = raw_int16_from_byte_array(buf, pos, buf_len);
    return (float)r / (float)(1 << n);
}

static inline int16_t float_to_d16qn(float a, size_t n)
{
    return (int16_t)((a) * (1 << n));
}

static inline bool check_IMU_CRC(const uint8_t *data, size_t length)
{
    if (!data || length < 11) return false;
    uint8_t chsum = 0;
    for (size_t i = 0; i < 10; ++i) {
        chsum = (uint8_t)((chsum + (uint8_t)data[i]) & 0xFF);
    }
    uint8_t received_cs = data[10];
    return chsum == received_cs;
}

union float_int {
    float f;
    uint32_t ul;
};

struct struct_imu_data {
    union float_int acc_x;
    union float_int acc_y;
    union float_int acc_z;
    union float_int gyr_x;
    union float_int gyr_y;
    union float_int gyr_z;
    union float_int roll;
    union float_int pitch;
    union float_int yaw;
    union float_int linacc_x;
    union float_int linacc_y;
    union float_int linacc_z;
};

static struct struct_imu_data imu_data;
static QueueHandle_t uart_mailbox = NULL;
static uint8_t rxbuf_local[BUF_SIZE] __attribute__((aligned(4)));
static const char *TAG = "IMU";

static volatile int intr_cpt = 0;

static void IRAM_ATTR uart_intr_handle(void* arg)
{
    uint32_t uart_intr_status = UART1.int_st.val;
    uint16_t rx_fifo_len = UART1.status.rxfifo_cnt;
    intr_cpt++;

    if (rx_fifo_len == 0) {
        uart_clear_intr_status(UART_NUM, uart_intr_status);
        return;
    }

    if (rx_fifo_len > BUF_SIZE) rx_fifo_len = BUF_SIZE;

    for (uint16_t i = 0; i < rx_fifo_len; ++i) {
        rxbuf_local[i] = UART1.fifo.rw_byte;
    }

    while (UART1.status.rxfifo_cnt || (UART1.mem_rx_status.wr_addr != UART1.mem_rx_status.rd_addr)) {
        volatile uint8_t dummy = UART1.fifo.rw_byte;
        (void)dummy;
    }

    // sync: 0x55 и 0x5x
    if (rx_fifo_len >= 2 && rxbuf_local[0] == 0x55) {
        uint8_t second = rxbuf_local[1];
        if (second == 0x51 || second == 0x52 || second == 0x53) {
            if (uart_mailbox) {
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                xQueueOverwriteFromISR(uart_mailbox, rxbuf_local, &xHigherPriorityTaskWoken);
                if (xHigherPriorityTaskWoken) {
                    portYIELD_FROM_ISR();
                }
            }        
        }
    }

    uart_clear_intr_status(UART_NUM, uart_intr_status);
}

void print_table(uint8_t *ptr, int len)
{
    if (!ptr || len <= 0) return;
    for (int i = 0; i < len; i++) {
        printf("%02x ", ptr[i]);
    }
    printf("\n");
}

void print_imu()
{
    printf("\n%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t\n",
           imu_data.acc_x.f,
           imu_data.acc_y.f,
           imu_data.acc_z.f,
           imu_data.gyr_x.f,
           imu_data.gyr_y.f,
           imu_data.gyr_z.f,
           imu_data.roll.f,
           imu_data.pitch.f,
           imu_data.yaw.f,
           imu_data.linacc_x.f,
           imu_data.linacc_y.f,
           imu_data.linacc_z.f);
}

int imu_init()
{
    ESP_LOGI(TAG, "WIT IMU initializing...");

    if (uart_mailbox == NULL) {
        uart_mailbox = xQueueCreate(1, BUF_SIZE);
        if (uart_mailbox == NULL) {
            ESP_LOGE(TAG, "Failed to create UART mailbox");
            return -1;
        }
    }

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_rx_timeout(UART_NUM, 3));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, PIN_TXD, PIN_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));

    uart_disable_tx_intr(UART_NUM);
    uart_disable_rx_intr(UART_NUM);
    uart_isr_free(UART_NUM);

    ESP_ERROR_CHECK(uart_isr_register(UART_NUM, uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, NULL));
    uart_enable_rx_intr(UART_NUM);

    const char test_msg[] = "AT\r\n";
    int bytes_written = uart_write_bytes(UART_NUM, test_msg, strlen(test_msg));
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t buffer[64];
    int length = uart_read_bytes(UART_NUM, buffer, sizeof(buffer) - 1, 20 / portTICK_PERIOD_MS);

    if (bytes_written == (int)strlen(test_msg) && length >= 0) {
        ESP_LOGI(TAG, "UART OK: Connection established, sent %d bytes, received %d bytes", bytes_written, length);
        return 0;
    } else {
        ESP_LOGE(TAG, "UART ERROR: Connection failed. Sent: %d, Received: %d", bytes_written, length);
        return -1;
    }
}

inline int parse_IMU_data()
{
    uint8_t rxbuf_imu[BUF_SIZE];

    if (uart_mailbox == NULL) return -1;
    if (xQueuePeek(uart_mailbox, rxbuf_imu, 0) != pdTRUE) {
        return -1;
    }

    if (!check_IMU_CRC(rxbuf_imu, BUF_SIZE)) {

        return -1;
    }

    if (rxbuf_imu[IMU_HEADER_POS] != 0x55) {
        return -1;
    }

    uint8_t id = rxbuf_imu[1];
    if (id == 0x51) {
        // ACC
        imu_data.acc_x.f = float_from_byte_array_qn(rxbuf_imu, IMU_ACCX_POS, IMU_QN_ACC, BUF_SIZE);
        imu_data.acc_y.f = float_from_byte_array_qn(rxbuf_imu, IMU_ACCY_POS, IMU_QN_ACC, BUF_SIZE);
        imu_data.acc_z.f = float_from_byte_array_qn(rxbuf_imu, IMU_ACCZ_POS, IMU_QN_ACC, BUF_SIZE);

        // if all data in one package
        if (12 < BUF_SIZE && rxbuf_imu[12] == 0x52) {
            imu_data.gyr_x.f = float_from_byte_array_qn(rxbuf_imu, IMU_GYRX_POS, IMU_QN_GYR, BUF_SIZE);
            imu_data.gyr_y.f = float_from_byte_array_qn(rxbuf_imu, IMU_GYRY_POS, IMU_QN_GYR, BUF_SIZE);
            imu_data.gyr_z.f = float_from_byte_array_qn(rxbuf_imu, IMU_GYRZ_POS, IMU_QN_GYR, BUF_SIZE);
        }

        if (23 < BUF_SIZE && rxbuf_imu[23] == 0x53) {
            imu_data.roll.f  = float_from_byte_array_qn(rxbuf_imu, IMU_ROLL_POS,  IMU_QN_EF, BUF_SIZE);
            imu_data.pitch.f = float_from_byte_array_qn(rxbuf_imu, IMU_PITCH_POS, IMU_QN_EF, BUF_SIZE);
            imu_data.yaw.f   = float_from_byte_array_qn(rxbuf_imu, IMU_YAW_POS,   IMU_QN_EF, BUF_SIZE);
        }
    } else if (id == 0x52) {
        // if sensor data in individual package
        imu_data.gyr_x.f = float_from_byte_array_qn(rxbuf_imu, IMU_GYRX_POS, IMU_QN_GYR, BUF_SIZE);
        imu_data.gyr_y.f = float_from_byte_array_qn(rxbuf_imu, IMU_GYRY_POS, IMU_QN_GYR, BUF_SIZE);
        imu_data.gyr_z.f = float_from_byte_array_qn(rxbuf_imu, IMU_GYRZ_POS, IMU_QN_GYR, BUF_SIZE);
    } else if (id == 0x53) {
        // if sensor data in individual package
        imu_data.roll.f  = float_from_byte_array_qn(rxbuf_imu, IMU_ROLL_POS,  IMU_QN_EF, BUF_SIZE);
        imu_data.pitch.f = float_from_byte_array_qn(rxbuf_imu, IMU_PITCH_POS, IMU_QN_EF, BUF_SIZE);
        imu_data.yaw.f   = float_from_byte_array_qn(rxbuf_imu, IMU_YAW_POS,   IMU_QN_EF, BUF_SIZE);
    } else {
        return -1;
    }

    return 0;
}

// original interface
uint16_t get_acc_x_in_D16QN() { return (uint16_t)float_to_d16qn(imu_data.acc_x.f, IMU_QN_ACC); }
uint16_t get_acc_y_in_D16QN() { return (uint16_t)float_to_d16qn(imu_data.acc_y.f, IMU_QN_ACC); }
uint16_t get_acc_z_in_D16QN() { return (uint16_t)float_to_d16qn(imu_data.acc_z.f, IMU_QN_ACC); }

uint16_t get_gyr_x_in_D16QN() { return (uint16_t)float_to_d16qn(imu_data.gyr_x.f, IMU_QN_GYR); }
uint16_t get_gyr_y_in_D16QN() { return (uint16_t)float_to_d16qn(imu_data.gyr_y.f, IMU_QN_GYR); }
uint16_t get_gyr_z_in_D16QN() { return (uint16_t)float_to_d16qn(imu_data.gyr_z.f, IMU_QN_GYR); }

uint16_t get_roll_in_D16QN()  { return (uint16_t)float_to_d16qn(imu_data.roll.f,  IMU_QN_EF); }
uint16_t get_pitch_in_D16QN() { return (uint16_t)float_to_d16qn(imu_data.pitch.f, IMU_QN_EF); }
uint16_t get_yaw_in_D16QN()   { return (uint16_t)float_to_d16qn(imu_data.yaw.f,   IMU_QN_EF); }

uint16_t get_linacc_x_in_D16QN() { return (uint16_t)float_to_d16qn(imu_data.linacc_x.f, IMU_QN_ACC); }
uint16_t get_linacc_y_in_D16QN() { return (uint16_t)float_to_d16qn(imu_data.linacc_y.f, IMU_QN_ACC); }
uint16_t get_linacc_z_in_D16QN() { return (uint16_t)float_to_d16qn(imu_data.linacc_z.f, IMU_QN_ACC); }
