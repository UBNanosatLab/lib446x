/** @file */
#ifndef SI446X_H
#define SI446X_H

#include <stdbool.h>
#include <stdint.h>

#define ESUCCESS 0 /**< @brief command succeeded */
#define ETIMEOUT 1 /**< @brief timeout waitng for CTS*/
#define EWRONGPART 2 /**< @brief unsupported part number*/
#define EINVAL 3 /**< @brief invalid parameter*/
#define EINVALSTATE 4 /**< @brief invalid internal state*/
#define ETOOLONG 5 /**< @brief packet too long*/
#define ECHKSUM 6 /**< @brief invalid check sum*/
#define EBUSY 7 /**< @brief pending operation*/

#define ENOTIMPL 127  /**< @brief not yet implemented functionality*/

#define CRC_SEED_0                                          0x00
#define CRC_SEED_1                                          0x80
#define CRC_NO_CRC                                          0x00
#define CRC_ITU_T_8                                         0x01
//...
#define CRC_IBM_16                                          0x04
#define CRC_CCIT_16                                         0x05

#define OPT_XTAL                                            0x00
#define OPT_TCXO                                            0x01

#define MAX_PACKET_SIZE 255 /**< @brief maximum packet size in bytes*/
#define FIFO_SIZE 64
#define RX_FIFO_THRESH 48
#define TX_FIFO_THRESH 16

/**
 * @brief The internal state of the library
 */
enum si446x_state {
    IDLE,
    LISTEN,
    RX,
    TX
};


/**
 * @brief Represents a buffer with a length and position
 */
struct buffer {
    uint8_t *data;
    int len;
};

/**
 *  @brief Represents a unique Si446x device
 */
struct si446x_device {
    uint32_t xo_freq;
    uint16_t part;
    int nsel_pin;
    int sdn_pin;
    int int_pin;
    void (*pkt_tx_handler)(struct si446x_device *dev, int err);
    void (*pkt_rx_handler)(struct si446x_device *dev, int err, int len,
                          uint8_t *data);
    volatile struct buffer tx_buf;
    volatile struct buffer rx_buf;
    volatile uint8_t rx_pkt_len;
    volatile uint8_t tx_pkt_index;
    volatile uint8_t rx_pkt_index;
    volatile bool has_rx_len;
    uint8_t config;
    enum si446x_state state;
    int err; // Used in blocking send / recv
};

/**
 *  @brief Info about an Si446x part
 */
struct si446x_part_info {
    uint8_t chip_rev;   /**< @brief Chip revision */
    uint16_t part;      /**< @brief Part number (e.g. 0x4463) */
    uint8_t part_build; /**< @brief Part build */
    uint16_t id;        /**< @brief ID number */
    uint8_t customer;   /**< @brief Customer */
    uint8_t rom_id;     /**< @brief ROM ID */
};

/**
 * Populate an si446x_device struct
 * @param dev The structure to populate
 * @param nsel_pin The NSEL (chip select) pin number
 * @param sdn_pin The SDN (shutdown) pin number
 * @param int_pin The INT (interrupt) pin number
 * @param xo_freq The frequency of the crystal attached to the Si446x
 * @param xo_freq Configuration parameters: xtal type
 * @return negative error code or 0 for success
 */
int si446x_create(struct si446x_device *dev, int nsel_pin, int sdn_pin,
                  int int_pin, uint32_t xo_freq, uint8_t config);

/**
 * Initalize an Si446x device
 * @param device si446x device to initalize
 * @return negative error code or 0 for success
 */
int si446x_init(struct si446x_device *device);

/**
 * Reset the Si446x
 * @return negative error code or 0 for success
 */
int si446x_reset(struct si446x_device *dev);

/**
 * Get part info
 * @param device the si446x device
 * @param info part_info structure to populate
 * @return negative error code or 0 for success
 */
int si446x_get_part_info(struct si446x_device *device,
                         struct si446x_part_info *info);

/**
 * Set the transmit / receive frequency
 * @param device the si446x device
 * @param freq the desired frequency in hertz
 * @return negative error code or 0 for success
 */
int si446x_set_frequency(struct si446x_device *device, uint32_t freq);

/**
 * Configure the CRC polynomial
 * @param device the si446x device
 * @param cfg the CRC polynomial and seed value
 * @return negative error code or 0 for success
 */
int si446x_config_crc(struct si446x_device *dev, uint8_t cfg);

/**
 * Set Si446x transmit power
 * @param device the si446x device
 * @param pwr power setting (see graph in datasheet)
 * @return negative error code or 0 for success
 */
int si446x_set_tx_pwr(struct si446x_device *dev, uint8_t pwr);

/**
 * Set GPIO pin configurations
 * @param device the si446x device
 * @param gpio0 config for GPIO 0
 * @param gpio1 config for GPIO 1
 * @param gpio2 config for GPIO 2
 * @param gpio3 config for GPIO 3
 * @return negative error code or 0 for success
 */
int si446x_cfg_gpio(struct si446x_device *dev, uint8_t gpio0, uint8_t gpio1,
                    uint8_t gpio2, uint8_t gpio3);

/**
 * Transmit the provided data
 * @param device the si446x device
 * @param len the length of the data in bytes
 * @param data pointer to the data
 * @return negative error code or 0 for success
 */
int si446x_send(struct si446x_device *dev, int len, uint8_t *data);

/**
 * Receive data into the provided buffer
 * @param device the si446x device
 * @param len pointer to length of the data in bytes, updated with actual rx len
 * @param data pointer to the data
 * @return negative error code or 0 for success
 */
int si446x_recv(struct si446x_device *dev, int *len, uint8_t *data);

/**
 * Transmit data asynchronously from the provided buffer
 * @param device the si446x device
 * @param len pointer to length of the buffer in bytes
 * @param data pointer to the buffer
 * @param handler function to call upon packet completion
 * @return negative error code or 0 for success
 */
int si446x_send_async(struct si446x_device *dev, int len, uint8_t *buf,
                      void (*handler)(struct si446x_device *dev, int err));

/**
 * Receive data asynchronously into the provided buffer
 * @param device the si446x device
 * @param len pointer to length of the buffer in bytes
 * @param data pointer to the buffer
 * @param handler function to call upon packet reception
 * @return negative error code or 0 for success
 */
int si446x_recv_async(struct si446x_device *dev, int len, uint8_t *buf,
                      void (*handler)(struct si446x_device *dev, int err,
                                      int len, uint8_t *data));

/**
 * Update internal state in response to an IRQ
 * @param device the si446x device
 * @return negative error code or 0 for success
 */
int si446x_update(struct si446x_device *dev);

/**
 * Ready the given data for transmission
 * @param device the si446x device
 * @param len the length of the data in bytes
 * @param data pointer to the data
 * @return negative error code or 0 for success
 */
int si446x_setup_tx(struct si446x_device *dev, int len, uint8_t *data);

/**
 * Transmit the previously setup packet
 * @return negative error code or 0 for success
 */
int si446x_fire_tx(struct si446x_device *dev);

#endif