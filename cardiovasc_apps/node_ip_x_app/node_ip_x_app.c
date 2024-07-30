// These macros are defined in the CMakeLists.txt in order to generate two distinct target apps
// #define PICO_VALVES
// #define PICO_SENSORS

/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */

/* C Standard library includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h> // memcpy, strlen

/* Wiznet includes */
#include "socket.h"
#include "wizchip_conf.h"

/* W6100 includes */
#include "port_common.h"
#include "wizchip_conf.h"
#include "w6x00_spi.h"
#include "timer.h"

#ifdef PICO_SENSORS
#include "hardware/adc.h"
#endif

/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */

/* Debug message printout enable */
//#define _PRINT_DEBUG_

/* Clock */
#define PLL_SYS_KHZ (133 * 1000)

/* TCP Buffer */
#define CLIENT_MAX_RX_BUF_SIZE (2 * 1024)
#define CLIENT_MAX_TX_BUF_SIZE 128

/* TCP Socket */
#define SOCKET_TCP_CLIENT 1

/* TCP Port */
#define PORT_TCP_CLIENT_DEST 4243

/* On-board LED  (for comms check) */
const uint LED_PIN = PICO_DEFAULT_LED_PIN;

/* Constants for TCP messages */
#define MSG_PICO_NODE_ID  0x00
#define MSG_SET_OUTPUT    0x10
#define MSG_GET_INPUT     0x11
#define MSG_GET_ANALOG    0x12
#define MSG_INPUT_CHANGED 0x13

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */

/* Network */
static wiz_NetInfo g_net_info = {
#ifdef PICO_VALVES
        .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56}, // MAC address
        .ip = {192, 168, 11, 2},                     // IP address
#endif
#ifdef PICO_SENSORS
        .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x57}, // MAC address
        .ip = {192, 168, 11, 3},                     // IP address
#endif
        .sn = {255, 255, 255, 0},                    // Subnet Mask
        .gw = {192, 168, 11, 1},                     // Gateway
        .dns = {8, 8, 8, 8},                         // DNS server
        .lla = {0xfe, 0x80, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x02, 0x08, 0xdc, 0xff,
                0xfe, 0x57, 0x57, 0x25},             // Link Local Address
        .gua = {0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00},             // Global Unicast Address
        .sn6 = {0xff, 0xff, 0xff, 0xff,
                0xff, 0xff, 0xff, 0xff,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00},             // IPv6 Prefix
        .gw6 = {0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00},             // Gateway IPv6 Address
        .dns6 = {0x20, 0x01, 0x48, 0x60,
                0x48, 0x60, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x88, 0x88},             // DNS6 server
        .ipmode = NETINFO_STATIC_ALL
};

/* TCP Client */

#define MSG_RESPONSE_OFFSET 128

static uint8_t tcp_client_destip[] = {
    192, 168, 11, 10,
};

static uint16_t tcp_client_destport = PORT_TCP_CLIENT_DEST;

static uint8_t g_client_rx_buf[CLIENT_MAX_RX_BUF_SIZE];
static uint8_t g_client_tx_buf[CLIENT_MAX_TX_BUF_SIZE];

/* Timer */
static volatile uint16_t g_msec_cnt = 0;

#ifdef PICO_SENSORS
#define NUMBER_OF_SENSORS 8
#define NUMBER_OF_ANALOGS 3
bool sensors_curr_state[NUMBER_OF_SENSORS];
bool sensors_last_state[NUMBER_OF_SENSORS];
#endif

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */

/* Clock */
static void set_clock_khz(void);

/* Timer */
static void repeating_timer_callback(void);

/* TCP Client */
int32_t tcp_client(uint8_t sn, uint8_t* received_data, uint8_t* destip, uint16_t destport);
void consume_pico_frame(uint8_t sn, uint8_t* received_data, datasize_t received_size);
void process_pico_message(uint8_t* frame_data, uint8_t** to_send_data, datasize_t* to_send_size);
datasize_t create_pico_frame(uint8_t* to_send_data, uint16_t msg_type, uint16_t msg_data_size, uint8_t* msg_data);

/**
 * ----------------------------------------------------------------------------------------------------
 * Main
 * ----------------------------------------------------------------------------------------------------
 */

int main()
{
    /* Initialize Pico */
    int retval = 0;
    set_clock_khz();
    stdio_init_all();
    sleep_ms(1000 * 3);

    printf("==========================================================\n");
    printf("Compiled @ %s, %s\n", __DATE__, __TIME__);
    printf("==========================================================\n");

    /* Initialize Wiznet */
    wizchip_spi_initialize();
    wizchip_cris_initialize();
    wizchip_reset();
    wizchip_initialize();
    wizchip_check();
    wizchip_1ms_timer_initialize(repeating_timer_callback);

    /* Initialize Network */
    network_initialize(g_net_info);
    print_network_information(g_net_info);

    /* On-board LED  (for comms check) */
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

#ifdef PICO_VALVES
    /* CardioVasc Pneumatic Valves */
    #define FIRST_VALVE_GPIO 2
    #define NUMBER_OF_VALVES 8
    for (int gpio = FIRST_VALVE_GPIO; gpio < (FIRST_VALVE_GPIO + NUMBER_OF_VALVES); ++gpio) {
        gpio_init(gpio);
        gpio_set_dir(gpio, GPIO_OUT);
    }
#endif
#ifdef PICO_SENSORS
    /* CardioVasc Digital Sensors */
    #define FIRST_SENSOR_GPIO 2
    for (int gpio = FIRST_SENSOR_GPIO; gpio < (FIRST_SENSOR_GPIO + NUMBER_OF_SENSORS); ++gpio) {
        gpio_init(gpio);
        gpio_set_dir(gpio, GPIO_IN);
        gpio_pull_up(gpio);
        // gpio_set_inover(gpio, GPIO_OVERRIDE_INVERT);
        gpio_set_input_hysteresis_enabled(gpio, true);
        gpio_set_input_enabled(gpio, true);
    }

    for (int io_device = 1; io_device <= NUMBER_OF_SENSORS; ++io_device) {
        sensors_curr_state[io_device - 1] = gpio_get(FIRST_SENSOR_GPIO + io_device - 1);
        sensors_last_state[io_device - 1] = sensors_curr_state[io_device - 1];
    }

    /* CardioVasc Analog Sensors */
    #define FIRST_ANALOG_GPIO 26
    adc_init();
    for (int gpio = FIRST_ANALOG_GPIO; gpio < (FIRST_ANALOG_GPIO + NUMBER_OF_ANALOGS); ++gpio) {
        adc_gpio_init(gpio);
    }
#endif

    /* Infinite loop */
    while (1)
    {
        if ((retval = tcp_client(SOCKET_TCP_CLIENT, g_client_rx_buf, tcp_client_destip, tcp_client_destport)) < 0)
        {
            printf("tcp_client error: %d\r\n", retval);
        }
    }
}

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */

/* Clock */
static void set_clock_khz(void)
{
    // set a system clock frequency in khz
    set_sys_clock_khz(PLL_SYS_KHZ, true);

    // configure the specified clock
    clock_configure(
        clk_peri,
        0,                                                // No glitchless mux
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
        PLL_SYS_KHZ * 1000,                               // Input frequency
        PLL_SYS_KHZ * 1000                                // Output (must be same as no divider)
    );
}

/* Timer */
static void repeating_timer_callback(void)
{
    g_msec_cnt++;

    if (g_msec_cnt >= 1000 - 1)
    {
        g_msec_cnt = 0;
    }
}

/* TCP Client */
int32_t tcp_client(uint8_t sn, uint8_t* received_data, uint8_t* destip, uint16_t destport)
{
    static uint16_t any_port = 50000;
    datasize_t received_size;
    datasize_t sent_size;
    int32_t ret; // return value for SOCK_ERRORs
    uint8_t status;
    uint8_t inter;
    uint8_t tmp;
    uint8_t arg_tmp8;

    // Check the W6100 SocketNum status register (Sn_SR, The 'Sn_SR' controlled by Sn_CR command or Packet send/recv status)
    getsockopt(sn, SO_STATUS, &status);

    // Socket Status Transitions
    switch (status)
    {
    case SOCK_CLOSE_WAIT:
        printf("%d:SOCK_CLOSE_WAIT\r\n", sn);
        if ((ret = disconnect(sn)) != SOCK_OK)
        {
            printf("%d:disconnect() failed: %d\r\n", sn, ret);
            close(sn);
            return ret;
        }
        printf("%d:Socket closed\r\n", sn);
        break;

    case SOCK_CLOSED:
        tmp = socket(sn, Sn_MR_TCP4, any_port++, SOCK_IO_NONBLOCK);
        if (tmp != sn)
        {
            printf("%d:socket() failed: %d\r\n", sn, tmp);
            return SOCKERR_SOCKNUM;
        }
        printf("%d:Socket opened[%d]\r\n", sn, getSn_SR(sn));
        break;

    case SOCK_INIT:
        printf("%d:Connecting to: %d.%d.%d.%d, on port: %d\r\n", sn, destip[0], destip[1], destip[2], destip[3], destport);
        ret = connect(sn, destip, destport, 4); /* Try to connect to TCP server(Socket, DestIP, DestPort) */
        if (ret < 0)
        {
            printf("%d:connect() failed: %d\r\n", sn, ret);
            return ret;
        }
        else
        if (ret == SOCK_BUSY)
        {
            printf("%d:connect() in progress..\r\n", sn);
        }
        else
        if (ret == SOCK_OK)
        {
            printf("%d:Socket connected!\r\n", sn);
        }
        break;

    case SOCK_ESTABLISHED:
        ctlsocket(sn, CS_GET_INTERRUPT, &inter);
        if (inter & Sn_IR_CON) // SocketNum interrupt register mask; TCP CON interrupt = connection with peer is successful
        {
            // Connected!!
            printf("%d:Connected to: %d.%d.%d.%d, on port: %d\r\n", sn, destip[0], destip[1], destip[2], destip[3], destport);
            arg_tmp8 = Sn_IR_CON;
            ctlsocket(sn, CS_CLR_INTERRUPT, &arg_tmp8); // this interrupt should write the bit to be cleared to '1'

            // Send Pico's IP address last octate
            uint16_t msg_type = MSG_PICO_NODE_ID;
            uint8_t msg_data = g_net_info.ip[3];
            datasize_t to_send_size = create_pico_frame(g_client_tx_buf, msg_type, sizeof(msg_data), &msg_data);
            if (to_send_size > 0) {
                sent_size = 0;
                while(sent_size < to_send_size)
                {
                    ret = send(sn, g_client_tx_buf + sent_size, to_send_size - sent_size);
                    if(ret < 0) // Send Error occurred (sent data length < 0)
                    {
                        printf("%d:send() failed: %d\r\n", sn, ret);
                        close(sn); // socket close
                        return ret;
                    }
                    sent_size += ret;
                }
            }
        }

        //////////////////////////////////////////////////////////////////////////////////////////////
        // Data Transaction Parts; Handle the [data receive and send] process
        //////////////////////////////////////////////////////////////////////////////////////////////

        getsockopt(sn, SO_RECVBUF, &received_size);
        if (received_size > 0) // Sn_RX_RSR: SocketNum Received Size Register, Receiving data length
        {
            if (received_size > CLIENT_MAX_RX_BUF_SIZE) received_size = CLIENT_MAX_RX_BUF_SIZE; // Max user defined buffer size (array)
            ret = recv(sn, received_data, received_size); // Data Receive process (H/W Rx socket buffer -> User's buffer)
            if (ret <= 0) // If the received data length <= 0, receive failed and process ends
            {
                printf("%d:recv() failed: %d\r\n", sn, ret);
                close(sn); // socket close
                return ret;
            }
            received_size = (uint16_t) ret;

            // Process the received data
            consume_pico_frame(sn, received_data, received_size);
        }

#ifdef PICO_SENSORS
        for (int io_device = 1; io_device <= NUMBER_OF_SENSORS; ++io_device) {
            sensors_curr_state[io_device - 1] = gpio_get(FIRST_SENSOR_GPIO + io_device - 1);
            if (sensors_last_state[io_device - 1] != sensors_curr_state[io_device - 1]) {
                sensors_last_state[io_device - 1] = sensors_curr_state[io_device - 1];
                // Send Pico's changed input
                uint16_t msg_type = MSG_INPUT_CHANGED;
                uint8_t msg_data[] = { io_device, sensors_curr_state[io_device - 1] ? 1 : 0 };
                datasize_t to_send_size = create_pico_frame(g_client_tx_buf, msg_type, sizeof(msg_data), msg_data);
                if (to_send_size > 0) {
                    sent_size = 0;
                    while(sent_size < to_send_size)
                    {
                        ret = send(sn, g_client_tx_buf + sent_size, to_send_size - sent_size);
                        if(ret < 0) // Send Error occurred (sent data length < 0)
                        {
                            printf("%d:send() failed: %d\r\n", sn, ret);
                            close(sn); // socket close
                            return ret;
                        }
                        sent_size += ret;
                    }
                }
            }
        }
#endif
        //////////////////////////////////////////////////////////////////////////////////////////////
        break;

    default:
        break;
    }

    return 1;
}

void consume_pico_frame(uint8_t sn, uint8_t* received_data, datasize_t received_size) {
    static uint8_t frame_data[CLIENT_MAX_RX_BUF_SIZE] = { 0 };
    static int frame_size = 0;

    // Append received_data to frame_data until a full frame is completed
    if (frame_size + received_size <= CLIENT_MAX_RX_BUF_SIZE) {
        memcpy(frame_data + frame_size, received_data, received_size);
        frame_size += received_size;
    } else {
        printf("Parser Error: frame_size + received_size > CLIENT_MAX_RX_BUF_SIZE: %d, %d, %d,\r\n",
            frame_size, received_size, CLIENT_MAX_RX_BUF_SIZE);
        frame_size = 0;
        return;
    }

check_full_frame:
    // Check if we have a full frame to process. The format is:
    // - delimiter: 4 bytes of fixed data {0x3A, 0xF9, 0x02, 0xB1}.
    // - msg_type: 2 bytes, big endian.
    // - msg_data_size: 2 bytes, big endian. The size of msg_data
    // - msg_data: msg_data_size bytes.
    // - checksum: 2 bytes, big endian.
    if (frame_size >= 8) {
        // Check the delimiter
        if (frame_data[0] != 0x3A ||
            frame_data[1] != 0xF9 ||
            frame_data[2] != 0x02 ||
            frame_data[3] != 0xB1 ) {
            printf("Parser Error: Wrong delimiter: %02X:%02X:%02X:%02X\r\n",
                frame_data[0], frame_data[1], frame_data[2], frame_data[3]);
            frame_size = 0;
            return;
        }

        // Check the rest of the frame
        uint16_t msg_data_size = ((((uint16_t)frame_data[6]) & 0xFF) << 8) | frame_data[7]; // big-endian
        if (frame_size >= (msg_data_size + 10)) {
            // Verify the checksum

            const uint16_t salt = 0xA35C;
            uint16_t calc_checksum = 0;
            for (int i = 0; i < msg_data_size + 8; ++i) {
                // alternate adding data (plus one to avoid zeros) to the high and low bytes of the checksum
                calc_checksum += ((((uint16_t)frame_data[i]) & 0xFF) + 1) << ((i & 1)? 8 : 0);
                calc_checksum ^= ((frame_data[i] & 1)? (salt << 1) : (salt >> 1)); // "salt" the checksum
            }

            uint16_t frame_checksum = ((((uint16_t)frame_data[msg_data_size + 8]) & 0xFF) << 8) | frame_data[msg_data_size + 9];

            if (calc_checksum != frame_checksum) {
                printf("Parser Error: Wrong checksum!\r\n");
                printf("\tcalc_checksum:  %04X\r\n", calc_checksum);
                printf("\tframe_checksum: %04X\r\n", frame_checksum);

                frame_size = 0;
                return;
            }

            // Everything okay, execute the command in the message
            uint8_t* to_send_data = NULL;
            datasize_t to_send_size = 0;

            process_pico_message(frame_data, &to_send_data, &to_send_size);

            if (to_send_data != NULL && to_send_size > 0) {
                datasize_t sent_size = 0;
                while(sent_size < to_send_size)
                {
                    int32_t ret = send(sn, to_send_data + sent_size, to_send_size - sent_size);
                    if(ret < 0) // Send Error occurred (sent data length < 0)
                    {
                        printf("%d:send() failed: %d\r\n", sn, ret);

                        frame_size = 0;
                        return;
                    }
                    sent_size += ret;
                }
            }

            if (frame_size > (msg_data_size + 10)) {
                memmove(frame_data, frame_data + (msg_data_size + 10), frame_size - (msg_data_size + 10));
                frame_size -= (msg_data_size + 10);
                goto check_full_frame;
            } else {
                frame_size = 0;
            }
        }
    }
}

void process_pico_message(uint8_t* frame_data, uint8_t** to_send_data, datasize_t* to_send_size) {
    *to_send_data = g_client_tx_buf;
    uint16_t msg_type = ((((uint16_t)frame_data[4]) & 0xFF) << 8) | frame_data[5]; // big-endian
    uint16_t msg_data_size = ((((uint16_t)frame_data[6]) & 0xFF) << 8) | frame_data[7]; // big-endian
    switch(msg_type) {
        case MSG_SET_OUTPUT: // Set an output
            if (msg_data_size == 2) { // Verify the expected length of the msg
                uint8_t state = frame_data[9];
                if (state == 0 || state == 1) {
                    uint8_t io_device = frame_data[8];
                    if (io_device == 0) { // Device 0 is the on-board LED
                        printf("LED: %d\r\n", state);
                        gpio_put(LED_PIN, state);

                        char* msg_data = "OK";
                        *to_send_size = create_pico_frame(g_client_tx_buf, msg_type + MSG_RESPONSE_OFFSET, strlen(msg_data), msg_data);
                    }
    #ifdef PICO_VALVES
                    else if (io_device >= 1 && io_device <= NUMBER_OF_VALVES) {
                        printf("Valve %d: %d\r\n", io_device, state);
                        gpio_put(FIRST_VALVE_GPIO + io_device - 1, state);

                        char* msg_data = "OK";
                        *to_send_size = create_pico_frame(g_client_tx_buf, msg_type + MSG_RESPONSE_OFFSET, strlen(msg_data), msg_data);
                    }
    #endif
                    else {
                        char* msg_data = "Error: unrecognized io_device";
                        *to_send_size = create_pico_frame(g_client_tx_buf, msg_type + MSG_RESPONSE_OFFSET, strlen(msg_data), msg_data);
                    }
                } else {
                    char* msg_data = "Error: state should be 0 or 1";
                    *to_send_size = create_pico_frame(g_client_tx_buf, msg_type + MSG_RESPONSE_OFFSET, strlen(msg_data), msg_data);
                }
            } else {
                char* msg_data = "Error: msg_data_size should be 2";
                *to_send_size = create_pico_frame(g_client_tx_buf, msg_type + MSG_RESPONSE_OFFSET, strlen(msg_data), msg_data);
            }
        break;

#ifdef PICO_SENSORS
        case MSG_GET_INPUT: // Get a digital input
            if (msg_data_size == 1) { // Verify the expected length of the msg
                uint8_t io_device = frame_data[8];
                if (io_device >= 1 && io_device <= NUMBER_OF_SENSORS) {
                    uint8_t state = gpio_get(FIRST_SENSOR_GPIO + io_device - 1) ? 1 : 0;
                    printf("Sensor %d: %d\r\n", io_device, state);

                    uint8_t msg_data[] = { io_device, state };
                    *to_send_size = create_pico_frame(g_client_tx_buf, msg_type + MSG_RESPONSE_OFFSET, sizeof(msg_data), msg_data);
                }
                else {
                    char* msg_data = "Error: unrecognized digital sensor number";
                    *to_send_size = create_pico_frame(g_client_tx_buf, msg_type + MSG_RESPONSE_OFFSET, strlen(msg_data), msg_data);
                }
            } else {
                char* msg_data = "Error: msg_data_size should be 1";
                *to_send_size = create_pico_frame(g_client_tx_buf, msg_type + MSG_RESPONSE_OFFSET, strlen(msg_data), msg_data);
            }
        break;

        case MSG_GET_ANALOG: // Get an analog input
            if (msg_data_size == 1) { // Verify the expected length of the msg
                uint8_t io_device = frame_data[8];
                if (io_device >= 1 && io_device <= NUMBER_OF_ANALOGS) {
                    adc_select_input(io_device - 1);
                    uint16_t value = adc_read();
                    printf("Analog %d: %d\r\n", io_device, value);

                    uint8_t msg_data[] = { io_device, ((value >> 8) & 0xFF), (value & 0xFF) }; // big-endian
                    *to_send_size = create_pico_frame(g_client_tx_buf, msg_type + MSG_RESPONSE_OFFSET, sizeof(msg_data), msg_data);
                }
                else {
                    char* msg_data = "Error: unrecognized analog sensor number";
                    *to_send_size = create_pico_frame(g_client_tx_buf, msg_type + MSG_RESPONSE_OFFSET, strlen(msg_data), msg_data);
                }
            } else {
                char* msg_data = "Error: msg_data_size should be 1";
                *to_send_size = create_pico_frame(g_client_tx_buf, msg_type + MSG_RESPONSE_OFFSET, strlen(msg_data), msg_data);
            }
        break;
#endif

        default:
            {
                char* msg_data = "Error: unrecognized msg_type";
                *to_send_size = create_pico_frame(g_client_tx_buf, msg_type + MSG_RESPONSE_OFFSET, strlen(msg_data), msg_data);
            }
        break;
    }
}

datasize_t create_pico_frame(uint8_t* to_send_data, uint16_t msg_type, uint16_t msg_data_size, uint8_t* msg_data) {
    // Set the delimiter
    to_send_data[0] = 0x3A;
    to_send_data[1] = 0xF9;
    to_send_data[2] = 0x02;
    to_send_data[3] = 0xB1;

    // Set the msg_type
    to_send_data[4] = (uint8_t)((msg_type >> 8) & 0xFF);
    to_send_data[5] = (uint8_t)(msg_type & 0xFF);

    // Set the msg_data_size
    to_send_data[6] = (uint8_t)((msg_data_size >> 8) & 0xFF);
    to_send_data[7] = (uint8_t)(msg_data_size & 0xFF);

    // Set the msg_data
    for (int i = 0; i < msg_data_size; ++i) {
        to_send_data[8 + i] = msg_data[i];
    }

    // Set the checksum (simplified algorithm created by JGonzalez)
    const uint16_t salt = 0xA35C;
    uint16_t checksum = 0;
    for (int i = 0; i < msg_data_size + 8; ++i) {
        // alternate adding data (plus one to avoid zeros) to the high and low bytes of the checksum
        checksum += (((uint16_t)(to_send_data[i]) & 0xFF) + 1) << ((i & 1)? 8 : 0);
        checksum ^= ((to_send_data[i] & 1)? (salt << 1) : (salt >> 1)); // "salt" the checksum
    }

    to_send_data[8 + msg_data_size] = (uint8_t)((checksum >> 8) & 0xFF);
    to_send_data[9 + msg_data_size] = (uint8_t)(checksum & 0xFF);

    return 10 + msg_data_size;
}
