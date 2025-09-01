#include "uart_imu.h"
#include "driver/uart.h"
#include "esp_intr_alloc.h"
#include "freertos/queue.h"


#define FLOAT_TO_D16QN(a, n) ((int16_t)((a) * (1 << (n))))

#define UART_NUM UART_NUM_1
#define BUF_SIZE 128
#define PIN_TXD 32
#define PIN_RXD 35

#define IMU_QN_ACC 11
#define IMU_QN_GYR 11
#define IMU_QN_EF 13

union float_int
{
    float f;
    unsigned long ul;
};

struct struct_imu_data
{
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

struct struct_imu_data imu_data;
static QueueHandle_t uart_mailbox;
static uint8_t rxbuf[BUF_SIZE];
static uint8_t rxbuf_imu[BUF_SIZE];
int intr_cpt = 0;

static intr_handle_t handle_console;

static void IRAM_ATTR uart_intr_handle(void* arg)
{
    uint16_t rx_fifo_len, status;
    uint16_t i = 0;
    status = UART1.int_st.val;
    rx_fifo_len = UART1.status.rxfifo_cnt;
    intr_cpt++;

    // read all bytes from rx fifo
    rx_fifo_len = rx_fifo_len > BUF_SIZE ? BUF_SIZE : rx_fifo_len;

    for (i = 0; i < rx_fifo_len && i < BUF_SIZE; ++i) {
        rxbuf[i] = UART1.fifo.rw_byte; // read all bytes
    }
    // Fix of esp32 hardware bug as in https://github.com/espressif/arduino-esp32/pull/1849
    while (UART1.status.rxfifo_cnt || (UART1.mem_rx_status.wr_addr != UART1.mem_rx_status.rd_addr)) {
        UART1.fifo.rw_byte;
    }

    // check header - descriptor
    if (rxbuf[0] == 0x55) {
        if ((rxbuf[1] == 0x51) || (rxbuf[1] == 0x52) || (rxbuf[1] == 0x53)) {
            xQueueOverwriteFromISR(uart_mailbox, &rxbuf, NULL);
        }
    }

    // clear UART interrupt status
    uart_clear_intr_status(UART_NUM, status);
    // uart_clear_intr_status(UART_NUM, UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT);
    // uart_clear_intr_status(UART_NUM, UART_INTR_RXFIFO_OVF);
}

void print_table(uint8_t *ptr, int len)
{
    for (int i = 0; i < len; i++)
    {
        printf("%02x ", ptr[i]);
    }
    printf("\n");
}


void print_imu()
{
    printf("\n%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t",
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

    printf("Initialising uart for WIT IMU...\n");

    uart_mailbox = xQueueCreate(1, 128);

    uart_config_t uart_config = { 
        .baud_rate = 115200, 
        .data_bits = UART_DATA_8_BITS, 
        .parity = UART_PARITY_DISABLE, 
        .stop_bits = UART_STOP_BITS_1, 
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE 
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_set_rx_timeout(UART_NUM, 3);
    uart_set_pin(UART_NUM, PIN_TXD, PIN_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    uart_disable_tx_intr(UART_NUM);
    uart_disable_rx_intr(UART_NUM);
    uart_isr_free(UART_NUM);

    uart_isr_register(UART_NUM, uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, &handle_console);
    uart_enable_rx_intr(UART_NUM);

    while (0) //for debug
    {
        parse_IMU_data();
        printf(" intr_cpt:%d\n", intr_cpt);
        printf("rxbuf:     ");
        print_table(rxbuf, 80);
        printf("rxbuf_imu: ");
        print_table(rxbuf_imu, 80);
        // printf("rxbuf_ef:  ");
        // print_table(rxbuf_ef, 80);
        print_imu();
        vTaskDelay(10);
    }

    return 0;
}

inline int parse_IMU_data()
{
    xQueueReceive(uart_mailbox, &rxbuf_imu, portMAX_DELAY);

    switch (rxbuf_imu[1]) {
    case 0x51:  // Acceleration data
        imu_data.acc_x.f = ((float)((int16_t)(rxbuf_imu[3] << 8 | rxbuf_imu[2]))) / 32768.0f * 16.0f;
        imu_data.acc_y.f = ((float)((int16_t)(rxbuf_imu[5] << 8 | rxbuf_imu[4]))) / 32768.0f * 16.0f;
        imu_data.acc_z.f = ((float)((int16_t)(rxbuf_imu[7] << 8 | rxbuf_imu[6]))) / 32768.0f * 16.0f;
        break;
    case 0x52:  // Gyroscope data
        imu_data.gyr_x.f = ((float)((int16_t)(rxbuf_imu[3] << 8 | rxbuf_imu[2]))) / 32768.0f * 2000.0f;
        imu_data.gyr_y.f = ((float)((int16_t)(rxbuf_imu[5] << 8 | rxbuf_imu[4]))) / 32768.0f * 2000.0f;
        imu_data.gyr_z.f = ((float)((int16_t)(rxbuf_imu[7] << 8 | rxbuf_imu[6]))) / 32768.0f * 2000.0f;
        break;
    case 0x53:  // Orientation data
        imu_data.roll.f  = ((float)((int16_t)(rxbuf_imu[3] << 8 | rxbuf_imu[2]))) / 32768.0f * 180.0f;
        imu_data.pitch.f = ((float)((int16_t)(rxbuf_imu[5] << 8 | rxbuf_imu[4]))) / 32768.0f * 180.0f;
        imu_data.yaw.f   = ((float)((int16_t)(rxbuf_imu[7] << 8 | rxbuf_imu[6]))) / 32768.0f * 180.0f;
        break;
    default:
        return -1;
    }

    return 0;
}


uint16_t get_acc_x_in_D16QN() { return FLOAT_TO_D16QN(imu_data.acc_x.f, IMU_QN_ACC); }
uint16_t get_acc_y_in_D16QN() { return FLOAT_TO_D16QN(imu_data.acc_y.f, IMU_QN_ACC); }
uint16_t get_acc_z_in_D16QN() { return FLOAT_TO_D16QN(imu_data.acc_z.f, IMU_QN_ACC); }

uint16_t get_gyr_x_in_D16QN() { return FLOAT_TO_D16QN(imu_data.gyr_x.f, IMU_QN_GYR); }
uint16_t get_gyr_y_in_D16QN() { return FLOAT_TO_D16QN(imu_data.gyr_y.f, IMU_QN_GYR); }
uint16_t get_gyr_z_in_D16QN() { return FLOAT_TO_D16QN(imu_data.gyr_z.f, IMU_QN_GYR); }

uint16_t get_roll_in_D16QN() { return FLOAT_TO_D16QN(imu_data.roll.f, IMU_QN_EF); }
uint16_t get_pitch_in_D16QN() { return FLOAT_TO_D16QN(imu_data.pitch.f, IMU_QN_EF); }
uint16_t get_yaw_in_D16QN() { return FLOAT_TO_D16QN(imu_data.yaw.f, IMU_QN_EF); }

uint16_t get_linacc_x_in_D16QN() { return FLOAT_TO_D16QN(imu_data.linacc_x.f, IMU_QN_ACC); }
uint16_t get_linacc_y_in_D16QN() { return FLOAT_TO_D16QN(imu_data.linacc_y.f, IMU_QN_ACC); }
uint16_t get_linacc_z_in_D16QN() { return FLOAT_TO_D16QN(imu_data.linacc_z.f, IMU_QN_ACC); }
