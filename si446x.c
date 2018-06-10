#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "si446x.h"
#include "si446x_cmd.h"
#include "mcu.h"

#include "radio_config.h"

#define MAX_CTS_RETRY 5000

static int send_command(struct si446x_device *dev, uint8_t cmd, int data_len,
                        uint8_t *data)
{
    gpio_write(dev->nsel_pin, LOW);
    spi_write_byte(cmd);
    spi_write_data(data_len, data);
    gpio_write(dev->nsel_pin, HIGH);

    return 0;
}

static int send_buffer(struct si446x_device *dev, int data_len, const uint8_t *data)
{
    gpio_write(dev->nsel_pin, LOW);
    spi_write_data(data_len, data);
    gpio_write(dev->nsel_pin, HIGH);

    return 0;
}

static int wait_cts(struct si446x_device *dev)
{
    uint8_t resp = 0x00;
    uint16_t attempts = 0;

    while (attempts < MAX_CTS_RETRY) {
        gpio_write(dev->nsel_pin, LOW);
        spi_write_byte(CMD_READ_CMD_BUF);
        spi_read_data(1, &resp);
        gpio_write(dev->nsel_pin, HIGH);

        if (resp == 0xFF) {
            return 0;
        }

        attempts++;

        delay_micros(25);
    }

    return -ETIMEOUT;

}

static int send_cfg_data_wait(struct si446x_device *dev, int data_len,
                              const uint8_t *data)
{
    int err;
    err = send_buffer(dev, data_len, data);

    if (err) {
        return err;
    }

    err = wait_cts(dev);

    if (err) {
        return err;
    }

    return 0;
}

static int read_response(struct si446x_device *dev, int len, uint8_t *data)
{
    uint8_t resp = 0x00;
    uint16_t attempts = 0;

    //Wait for CTS
    while (resp != 0xFF) {
        gpio_write(dev->nsel_pin, LOW);
        spi_write_byte(CMD_READ_CMD_BUF);
        spi_read_data(1, &resp);
        if (resp != 0xFF) {
            gpio_write(dev->nsel_pin, HIGH);
            attempts++;
            if (attempts > MAX_CTS_RETRY) {
                return -ETIMEOUT;
            }
            delay_micros(50);
        }
    }

    spi_read_data(len, data);

    gpio_write(dev->nsel_pin, HIGH);
    return 0;
}

static int set_property(struct si446x_device *dev, uint8_t prop_grp,
                        uint8_t prop, uint8_t value)
{
    int err;
    uint8_t buf[] = {
        prop_grp,
        0x01, //length
        prop,
        value
    };

    err = send_command(dev, CMD_SET_PROPERTY, 4, buf);

    if (err) {
        return err;
    }

    return wait_cts(dev);

}

static int read_rx_fifo(struct si446x_device *dev, int len, uint8_t *data)
{
    if (len < 0 || len > 0xFF) {
        return -EINVAL;
    }

    gpio_write(dev->nsel_pin, LOW);
    spi_write_byte(CMD_READ_RX_FIFO);
    spi_read_data(len, data);
    gpio_write(dev->nsel_pin, HIGH);

    return ESUCCESS;
}

static int config_interrupts(struct si446x_device *dev, uint8_t int_gbl,
                             uint8_t int_ph, uint8_t int_modem,
                             uint8_t int_chip)
{
    int err = 0;
    err = set_property(dev, PROP_INT_CTL_GROUP, PROP_INT_CTL_ENABLE, int_gbl);
    if (err) {
        return err;
    }
    err = set_property(dev, PROP_INT_CTL_GROUP, PROP_INT_CTL_PH_ENABLE, int_ph);
    if (err) {
        return err;
    }
    err = set_property(dev, PROP_INT_CTL_GROUP, PROP_INT_CTL_MODEM_ENABLE,
                       int_modem);
    if (err) {
        return err;
    }
    err = set_property(dev, PROP_INT_CTL_GROUP, PROP_INT_CTL_CHIP_ENABLE,
                       int_chip);
    return err;
}

static int write_tx_fifo(struct si446x_device *dev, int len, uint8_t *data)
{
    int err = send_command(dev, CMD_WRITE_TX_FIFO, len, data);

    if (err) {
        return err;
    }

    // TODO: Pretty sure I shouldn't wait for CTS here

    err = wait_cts(dev);

    if (err) {
        return err;
    }

    return ESUCCESS;
}

static int handle_fifo_empty(struct si446x_device *dev)
{
    int rem = dev->tx_buf.len - dev->tx_pkt_index;
    int err = 0;

    if (rem > 0) {
        // Write out next chunk of the packet
        if (rem > FIFO_SIZE - TX_FIFO_THRESH) {
            err = write_tx_fifo(dev, FIFO_SIZE - TX_FIFO_THRESH,
                               dev->tx_buf.data + dev->tx_pkt_index);
            dev->tx_pkt_index += FIFO_SIZE - TX_FIFO_THRESH;
        } else {
            err = write_tx_fifo(dev, rem, dev->tx_buf.data + dev->tx_pkt_index);
            dev->tx_pkt_index += rem;
        }
    }

    return err;
}

static int handle_fifo_full(struct si446x_device *dev)
{
    int err = 0;

    int to_read = RX_FIFO_THRESH;

    if (!dev->has_rx_len) {
        // TODO: Is discarding the volatile qualifier safe here?
        err = read_rx_fifo(dev, sizeof(dev->rx_pkt_len),
                           (uint8_t *) &dev->rx_pkt_len);
        dev->has_rx_len = true;
        to_read -= sizeof(dev->rx_pkt_len);
    }

    if (err) {
        return err;
    }

    int rem = dev->rx_pkt_len - dev->rx_pkt_index;

    if (to_read > rem) {
        // ok, so something is just confused now...
        // Read what should remain then clear the FIFO
        // If the buffer is too small, just discard the packet
        if (dev->rx_pkt_len > dev->rx_buf.len) {
            err = read_rx_fifo(dev, rem, NULL);
        } else {
            err = read_rx_fifo(dev, rem, dev->rx_buf.data + dev->rx_pkt_index);
        }

        dev->rx_pkt_index += rem;

        // Clear the FIFO
        uint8_t fifo_cmd_buf = CLR_RX_FIFO;
        err = send_command(dev, CMD_FIFO_INFO, sizeof(fifo_cmd_buf), &fifo_cmd_buf);

        if (err) {
            return err;
        }

        return wait_cts(dev);

    } else {
        // Read in next chunk of the packet
        // If the buffer is too small, just discard the packet
        if (dev->rx_pkt_len > dev->rx_buf.len) {
            err = read_rx_fifo(dev, to_read, NULL);
        } else {
            err = read_rx_fifo(dev, to_read, dev->rx_buf.data + dev->rx_pkt_index);
        }

        dev->rx_pkt_index += to_read;

        return err;
    }
    // Not reachable
}

static int handle_packet_rx(struct si446x_device *dev)
{

    int err = 0;

    dev->state = IDLE;

    if (!dev->has_rx_len) {
        // TODO: Is discarding the volatile qualifier safe here?
        err = read_rx_fifo(dev, sizeof(dev->rx_pkt_len),
                           (uint8_t *) &dev->rx_pkt_len);
        dev->has_rx_len = true;
    }

    if (err) {
        if (dev->pkt_rx_handler) {
            dev->pkt_rx_handler(dev, err, 0, NULL);
            return ESUCCESS; // We don't report errors twice
        } else {
            return err;
        }
    }

    int rem = dev->rx_pkt_len - dev->rx_pkt_index;

    // Read in whatever remains of the packet
    // If the buffer is too small, just discard the packet
    if (dev->rx_pkt_len > dev->rx_buf.len) {
        err = read_rx_fifo(dev, rem, NULL);
    } else {
        err = read_rx_fifo(dev, rem, dev->rx_buf.data + dev->rx_pkt_index);
    }

    dev->rx_pkt_index += rem;

    if (err) {
        if (dev->pkt_rx_handler) {
            dev->pkt_rx_handler(dev, err, 0, NULL);
            return ESUCCESS; // We don't report errors twice
        } else {
            return err;
        }
    }

    // Packet received, clear whatever remains in the FIFO
    // In case packet handler screwed up...
    uint8_t fifo_cmd_buf = CLR_RX_FIFO;
    err = send_command(dev, CMD_FIFO_INFO, sizeof(fifo_cmd_buf), &fifo_cmd_buf);

    if (err) {
        if (dev->pkt_rx_handler) {
            dev->pkt_rx_handler(dev, err, 0, NULL);
            return ESUCCESS; // We don't report errors twice
        } else {
            return err;
        }
    }

    err = wait_cts(dev);


    if (err && dev->pkt_rx_handler) {
        dev->pkt_rx_handler(dev, err, 0, NULL);
    } else if (dev->rx_pkt_len > dev->rx_buf.len && dev->pkt_rx_handler) {
        dev->pkt_rx_handler(dev, -ETOOLONG, 0, NULL);
    } else if (dev->pkt_rx_handler){
        dev->pkt_rx_handler(dev, ESUCCESS,  dev->rx_pkt_len, dev->rx_buf.data);
    } else if (err) { // Error but no callback registered
        return err;
    }

    return ESUCCESS;
}

static int handle_invalid_chksum(struct si446x_device *dev)
{
    int err = 0;

    dev->state = IDLE;

    if (!dev->has_rx_len) {
        // TODO: Is discarding the volatile qualifier safe here?
        err = read_rx_fifo(dev, sizeof(dev->rx_pkt_len),
                           (uint8_t *) &dev->rx_pkt_len);
        dev->has_rx_len = true;

        if (err) {
            if (dev->pkt_rx_handler) {
                dev->pkt_rx_handler(dev, err, 0, NULL);
                return ESUCCESS; // We don't report errors twice
            } else {
                return err;
            }
        }
    }

    int rem = dev->rx_pkt_len - dev->rx_pkt_index;

    // Read in whatever remains of the packet
    // If the buffer is too small, just discard the packet
    if (dev->rx_pkt_len > dev->rx_buf.len) {
        err = read_rx_fifo(dev, rem, NULL);
    } else {
        err = read_rx_fifo(dev, rem, dev->rx_buf.data + dev->rx_pkt_index);
    }

    // TODO: Figre out a better way to do this...
    // Consider the case where the length byte was corrupt
    // In the event of a checksum miss, we need to purge the FIFO
    uint8_t fifo_cmd_buf = CLR_RX_FIFO;
    err = send_command(dev, CMD_FIFO_INFO, sizeof(fifo_cmd_buf), &fifo_cmd_buf);

    if (err) {
        if (dev->pkt_rx_handler) {
            dev->pkt_rx_handler(dev, err, 0, NULL);
            return ESUCCESS; // We don't report errors twice
        } else {
            return err;
        }
    }

    err = wait_cts(dev);

    dev->rx_pkt_index += rem;

    if (err && dev->pkt_rx_handler) {
        dev->pkt_rx_handler(dev, err, 0, NULL);
    } else if (dev->rx_pkt_len > dev->rx_buf.len && dev->pkt_rx_handler) {
        dev->pkt_rx_handler(dev, -ETOOLONG, 0, NULL);
    } else if (dev->pkt_rx_handler) {
        dev->pkt_rx_handler(dev, -ECHKSUM, dev->rx_pkt_len, dev->rx_buf.data);
    } else if (err) { // Error but no callback registered
        return err;
    }

    return ESUCCESS;
}

static int handle_packet_tx(struct si446x_device *dev)
{

    dev->state = IDLE;

    int err;

    err = set_property(dev, PROP_PKT_GROUP, PROP_PKT_FIELD_2_LENGTH_12_8,
                       ((MAX_PACKET_SIZE) >> 8) & 0xFF);
    if (err) {
      return err;
    }

    err = set_property(dev, PROP_PKT_GROUP, PROP_PKT_FIELD_2_LENGTH_7_0,
                 (MAX_PACKET_SIZE) & 0xFF);
    if (err) {
      return err;
    }

    // TODO: Call the callback with the err if the pkt didn't get sent
    if (dev->pkt_tx_handler) {
        dev->pkt_tx_handler(dev, ESUCCESS);
    }

    return ESUCCESS;
}

// Callback for blocking send
void blocking_tx_complete(struct si446x_device *dev, int err) {
    dev->err = err;
}

// Callback for blocking recv
void blocking_rx_complete(struct si446x_device *dev, int err, int len,
                          uint8_t *data) {
    dev->err = err;
}

int si446x_update(struct si446x_device *dev)
{

    // TODO: RSSI / Preamble / Sync word detect to put us in RX state

    int err = 0;

    // Clear interrupts
    // TODO: struct for this?, use designated initialization
    uint8_t int_data[] = {
        0xC0, // PACKET_SENT_PEND_CLR & PACKET_RX_PEND_CLR & CRC_ERR_PEND_CLR &
              // TX_FIFO_ALMOST_EMPTY_PEND_CLR & RX_FIFO_ALMOST_FULL_PEND_CLR
        0x3F, //
        0xFF  //
    };

    // TODO: Struct for this
    uint8_t int_status[8];

    if (dev->rx_timeout) {
        uint8_t state = STATE_READY;
        err = send_command(dev, CMD_CHANGE_STATE, sizeof(state), &state);

        if (err) {
            return err;
        }

        err = wait_cts(dev);

        if (err) {
            return err;
        }

        uint8_t fifo_cmd_buf = CLR_RX_FIFO;
        err = send_command(dev, CMD_FIFO_INFO, sizeof(fifo_cmd_buf), &fifo_cmd_buf);

        err = wait_cts(dev);

        if (err) {
            return err;
        }

        dev->state = IDLE;
        dev->rx_timeout = false;

        if (dev->pkt_rx_handler) {
            dev->pkt_rx_handler(dev, -ERXTIMEOUT, 0, NULL);
            return ESUCCESS;
        } else {
            return -ERXTIMEOUT;
        }
    }

    err = send_command(dev, CMD_GET_INT_STATUS, sizeof(int_data), int_data);

    if (err) {
        return err;
    }

    err = read_response(dev, 8, int_status);

    if (err) {
        return err;
    }

    // TX_FIFO_ALMOST_EMPTY_PEND
    if (int_status[2] & TX_FIFO_ALMOST_EMPTY) {
        err = handle_fifo_empty(dev);
        if (err) {
            return err;
        }
    }

    // RX_FIFO_ALMOST_FULL_PEND
    if (int_status[2] & RX_FIFO_ALMOST_FULL) {
        err = handle_fifo_full(dev);
        if (err) {
            return err;
        }
    }

    // PACKET_RX_PEND
    if (int_status[2] & PACKET_RX) {
        err = handle_packet_rx(dev);
        if (err) {
            return err;
        }
    }

    // CRC_ERR_PEND
    if (int_status[2] & CRC_ERROR) {
        err = handle_invalid_chksum(dev);
        if (err) {
            return err;
        }
    }

    // PACKET_SENT_PEND
    if (int_status[2] & PACKET_SENT) {
        err = handle_packet_tx(dev);
        if (err) {
            return err;
        }
    }

    return ESUCCESS;
}

int si446x_create(struct si446x_device *dev, int nsel_pin, int sdn_pin,
                  int int_pin, uint32_t xo_freq, uint8_t config)
{
    dev->nsel_pin = nsel_pin;
    dev->sdn_pin = sdn_pin;
    dev->xo_freq = xo_freq;
    dev->int_pin = int_pin;

    dev->pkt_tx_handler = NULL;
    dev->tx_buf.data = NULL;
    dev->tx_buf.len = 0;
    dev->tx_pkt_index = 0;

    dev->pkt_rx_handler = NULL;
    dev->rx_pkt_len = 0;
    dev->rx_buf.data = NULL;
    dev->rx_buf.len = 0;
    dev->rx_pkt_index = 0;
    dev->has_rx_len = false;

    dev->config = config;

    dev->rx_timeout = false;

    return 0;
}

int si446x_init(struct si446x_device *dev)
{
    int err;

    //TODO: struct for this?
    uint8_t pwr_up_data[] = {
        BOOT_OPT_NORMAL,
        (dev->config & 0x01), // XTAL_OPT
        (dev->xo_freq >> 24) & 0xFF,
        (dev->xo_freq >> 16) & 0xFF,
        (dev->xo_freq >>  8) & 0xFF,
        (dev->xo_freq >>  0) & 0xFF
    };

    spi_init();
    gpio_config(dev->int_pin, INPUT);

    gpio_config(dev->nsel_pin, OUTPUT);
    gpio_write(dev->nsel_pin, HIGH);
    gpio_config(dev->sdn_pin, OUTPUT);

    // Reset the Si446x (300 us strobe of SDN)
    gpio_write(dev->sdn_pin, HIGH);
    delay_micros(300);
    gpio_write(dev->sdn_pin, LOW);

    // Wait for POR (6 ms)
    delay_micros(6000);

    err = send_command(dev, CMD_POWER_UP, sizeof(pwr_up_data), pwr_up_data);

    if (err) {
        return err;
    }

    err = wait_cts(dev);

    if (err) {
        return err;
    }

    // uint8_t buf_RF_POWER_UP[] = {RF_POWER_UP};
    // err = send_cfg_data_wait(dev, 0x07, buf_RF_POWER_UP);
    // if (err) {
    //      return err;
    // }

    struct si446x_part_info info;
    err = si446x_get_part_info(dev, &info);

    if (err) {
        return err;
    }

    dev->part = info.part;

    if (info.part != 0x4460 && info.part != 0x4461 && info.part != 0x4463 &&
        info.part != 0x4464) {
            return -EWRONGPART;
    }

    // Clear all interrupts
    // TODO: struct  for this?, use designated initialization
    uint8_t int_data[] = {
        0x00,
        0x00,
        0x00
    };

    uint8_t buffer[8];

    err = send_command(dev, CMD_GET_INT_STATUS, sizeof(int_data), int_data);

    if (err) {
        return err;
    }

    err = read_response(dev, 8, buffer);

    if (err) {
        return err;
    }

    // Black-box stuff, send cfg blob

    const uint8_t *cfg = NULL;

    if (dev->part == 0x4463) {
        cfg = si4463_cfg;
    } else if (dev->part == 0x4464) {
        cfg = si4464_cfg;
    } else {
        return -ENOTIMPL;
    }

    int i = 0;
    int len = cfg[i++];
    while (len) {
        err = send_cfg_data_wait(dev, len, cfg + i);
        if (err) {
             return err;
        }
        i += len;
        len = cfg[i++];
    }

    err = set_property(dev, PROP_PKT_GROUP, PROP_PKT_FIELD_2_LENGTH_12_8,
                 ((MAX_PACKET_SIZE) >> 8) & 0xFF);
    if (err) {
      return err;
    }

    err = set_property(dev, PROP_PKT_GROUP, PROP_PKT_FIELD_2_LENGTH_7_0,
                 (MAX_PACKET_SIZE) & 0xFF);
    if (err) {
      return err;
    }

    err = set_property(dev, PROP_RF_PKT_LEN_12_GROUP, PROP_PKT_TX_THRESHOLD,
                       TX_FIFO_THRESH);
    if (err) {
      return err;
    }

    err = set_property(dev, PROP_RF_PKT_LEN_12_GROUP, PROP_PKT_RX_THRESHOLD,
                       RX_FIFO_THRESH);
    if (err) {
      return err;
    }

    err = config_interrupts(dev, PH_INT_STATUS_EN, PACKET_RX | PACKET_SENT |
                                 CRC_ERROR | TX_FIFO_ALMOST_EMPTY |
                                 RX_FIFO_ALMOST_FULL, 0, 0);
    if (err) {
      return err;
    }

    return 0;
}

int si446x_reset(struct si446x_device *dev)
{
    // Reset the Si446x (300 us strobe of SDN)
    gpio_write(dev->sdn_pin, HIGH);
    delay_micros(300);
    gpio_write(dev->sdn_pin, LOW);

    // Wait for POR (6 ms)
    delay_micros(6000);
    return ESUCCESS;
}

int si446x_get_part_info(struct si446x_device *device,
                         struct si446x_part_info *info)
{
    uint8_t buf[8];
    int err;

    err = send_command(device, CMD_PART_INFO, 0, NULL);

    if (err) {
        return err;
    }

    err = read_response(device, 8, buf);

    if (err) {
        return err;
    }

    info->chip_rev = buf[0];
    info->part = (buf[1] << 8) | buf[2];
    info->part_build = buf[3];
    info->id = (buf[4] << 8) | buf[5];
    info->customer = buf[6];
    info->rom_id = buf[7];

    return 0;
}

int si446x_set_frequency(struct si446x_device *dev, uint32_t freq)
{
    uint8_t outdiv;
    uint8_t band;

    if (dev->part == 0x4464) {
        if (freq > 119000000 && freq < 159000000) {
            outdiv = 24;
            band = 5;
        } else if (freq > 177000000 && freq < 239000000) {
            outdiv = 16;
            band = 4;
        } else if (freq > 235000000 && freq < 319000000) {
            outdiv = 12;
            band = 3;
        } else if (freq > 353000000 && freq < 479000000) {
            outdiv = 8;
            band = 2;
        } else if (freq > 470000000 && freq < 639000000) {
            outdiv = 6;
            band = 1;
        } else if (freq > 705000000 && freq < 960000000) {
            outdiv = 4;
            band = 0;
        }  else {
            return -EINVAL;
        }
    } else { // 4460, 4461, 4463
        if (freq > 142000000 && freq < 175000000) {
            outdiv = 24;
            band = 5;
        } else if (freq > 284000000 && freq < 350000000) {
            outdiv = 12;
            band = 3;
        } else if (freq > 420000000 && freq < 525000000) {
            outdiv = 8;
            band = 2;
        } else if (freq > 850000000 && freq < 1050000000) {
            outdiv = 4;
            band = 0;
        }  else {
            return -EINVAL;
        }
    }

    // fc = F / (2 * xo_freq / outdiv) * 2^19
    uint32_t fc = (uint32_t) (outdiv * ((uint64_t) freq << 18) / dev->xo_freq);

    set_property(dev, PROP_FREQ_CTRL_GROUP, PROP_FREQ_CTRL_INTE,
                 (fc >> 19) - 1);

    set_property(dev, PROP_FREQ_CTRL_GROUP, PROP_FREQ_CTRL_FRAC_0, fc & 0xFF);
    set_property(dev, PROP_FREQ_CTRL_GROUP, PROP_FREQ_CTRL_FRAC_1,
                 (fc >> 8) & 0xFF);
    set_property(dev, PROP_FREQ_CTRL_GROUP, PROP_FREQ_CTRL_FRAC_2,
                 0x08 | (fc >> 16 & 0x07));

    set_property(dev, PROP_MODEM_GROUP, PROP_MODEM_CLKGEN_BAND, 0x08 | band);

    //TODO: Set FREQ_CONTROL_VCOCNT_RX_ADJ

    return 0;

}

int si446x_config_crc(struct si446x_device *dev, uint8_t cfg)
{
    int err;

    if ((cfg & 0x70) != 0 || (cfg & 0x0F) > 0x08) {
        return -EINVAL;
    }

    err = set_property(dev, PROP_PKT_GROUP, PROP_PKT_CRC_CONFIG, cfg);

    if (err) {
        return err;
    }

    return 0;
}

int si446x_send(struct si446x_device *dev, int len, uint8_t *data)
{
    int err = si446x_send_async(dev, len, data, blocking_tx_complete);

    if (err) {
        return err;
    }

    while (dev->state != IDLE) {
        si446x_update(dev);
        delay_micros(1000);
    }

    return dev->err;
}

int si446x_recv(struct si446x_device *dev, int *len, uint8_t *buf)
{
    int err = si446x_recv_async(dev, *len, buf, blocking_rx_complete);

    if (err) {
        return err;
    }

    while (dev->state != IDLE) {
        si446x_update(dev);
        delay_micros(1000);
    }

    *len = dev->rx_pkt_len;

    return dev->err;
}

int si446x_send_async(struct si446x_device *dev, int len, uint8_t *data,
                      void (*handler)(struct si446x_device *dev, int err))
{
    int err;
    uint8_t packet_len = len;

    if (len < 0) {
        return -EINVAL;
    }

    if (len > MAX_PACKET_SIZE) {
        return -ETOOLONG;
    }

    if (dev->state == RX || dev->state == TX) {
        return -EBUSY;
    }

    // Clear the both FIFOs
    uint8_t fifo_cmd_buf = CLR_RX_FIFO | CLR_TX_FIFO;
    err = send_command(dev, CMD_FIFO_INFO, sizeof(fifo_cmd_buf), &fifo_cmd_buf);

    if (err) {
        return err;
    }

    err = wait_cts(dev);

    if (err) {
        return err;
    }

    err = send_command(dev, CMD_WRITE_TX_FIFO, sizeof(packet_len), &packet_len);

    if (err) {
        return err;
    }

    dev->pkt_tx_handler = handler;
    dev->tx_buf.data = data;
    dev->tx_buf.len = len;

    if (len > FIFO_SIZE - sizeof(packet_len)) {
        dev->tx_pkt_index = FIFO_SIZE - sizeof(packet_len);
    } else {
        dev->tx_pkt_index = len;
    }

    err = send_command(dev, CMD_WRITE_TX_FIFO, dev->tx_pkt_index, data);

    if (err) {
        return err;
    }

    err = wait_cts(dev);

    if (err) {
        return err;
    }

    uint8_t tx_config[] = {
        0x00, // Channel 0
        0x30, //READY, NO RETRANS, START NOW
        0x00, // length[15:8] (0 = use packet handler)
        0x00, // length[7:0]
    };

    set_property(dev, PROP_PKT_GROUP, PROP_PKT_FIELD_2_LENGTH_12_8,
                 (len >> 8) & 0xFF);
    if (err) {
        return err;
    }

    set_property(dev, PROP_PKT_GROUP, PROP_PKT_FIELD_2_LENGTH_7_0,
                 len & 0xFF);
    if (err) {
        return err;
    }

    dev->state = TX;

    err = send_command(dev, CMD_START_TX, sizeof(tx_config), tx_config);

    if (err) {
        dev->state = IDLE;
        return err;
    }

    err = wait_cts(dev);

    if (err) {
        return err;
    }

    return 0;
}

int si446x_recv_async(struct si446x_device *dev, int len,
                      uint8_t *buf,
                      void (*handler)(struct si446x_device *dev, int err,
                                      int len, uint8_t *data))
{

    if (dev->state == RX || dev->state == TX) {
        return -EBUSY;
    }

    // TODO: This should probably be a critical section with interrupts disabled
    // NULL out the buffer so nothing can fill it while we reset the length
    dev->rx_buf.data = NULL;
    dev->rx_buf.len = len;
    dev->rx_pkt_index = 0;
    dev->has_rx_len = false;
    dev->pkt_rx_handler = handler;
    dev->rx_buf.data = buf;

    int err;

    // Clear the both FIFOs
    uint8_t fifo_cmd_buf = CLR_RX_FIFO | CLR_TX_FIFO;
    err = send_command(dev, CMD_FIFO_INFO, sizeof(fifo_cmd_buf), &fifo_cmd_buf);

    if (err) {
        return err;
    }

    err = wait_cts(dev);

    if (err) {
        return err;
    }

    uint8_t rx_config[] = {
        0x00,               // Channel 0
        0x00,               // RX immediately
        0x00,               // Length[15:8] (0 = use packet handler)
        0x00,               // Length[7:0]
        STATE_RX,           // RX timeout state
        STATE_READY,        // RX valid state
        STATE_READY,        // RX invalid state
    };

    err = send_command(dev, CMD_START_RX, sizeof(rx_config), rx_config);

    if (err) {
        return err;
    }

    err = wait_cts(dev);

    if (err) {
        return err;
    }

    dev->state = LISTEN;

    return ESUCCESS;
}

int si446x_setup_tx(struct si446x_device *dev, int len, uint8_t *data,
                    void (*handler)(struct si446x_device *dev, int err))
{


    int err;
    uint8_t packet_len = len;

    if (len < 0) {
        return -EINVAL;
    }

    if (len > MAX_PACKET_SIZE) {
        return -ETOOLONG;
    }

    err = send_command(dev, CMD_WRITE_TX_FIFO, sizeof(packet_len), &packet_len);

    if (err) {
        return err;
    }

    dev->pkt_tx_handler = handler;
    dev->tx_buf.data = data;
    dev->tx_buf.len = len;

    if (len > FIFO_SIZE - sizeof(packet_len)) {
        dev->tx_pkt_index = FIFO_SIZE - sizeof(packet_len);
    } else {
        dev->tx_pkt_index = len;
    }

    err = send_command(dev, CMD_WRITE_TX_FIFO, dev->tx_pkt_index, data);

    if (err) {
        return err;
    }

    err = wait_cts(dev);

    if (err) {
        return err;
    }

    set_property(dev, PROP_PKT_GROUP, PROP_PKT_FIELD_2_LENGTH_12_8,
                 (len >> 8) & 0xFF);
    if (err) {
        return err;
    }

    set_property(dev, PROP_PKT_GROUP, PROP_PKT_FIELD_2_LENGTH_7_0,
                 len & 0xFF);
    if (err) {
        return err;
    }

    return 0;
}

int si446x_fire_tx(struct si446x_device *dev)
{
    int err;

    if (dev->state == RX || dev->state == TX) {
        return -EBUSY;
    }

    uint8_t tx_config[] = {
        0x00, // Channel 0
        0x30, //READY, NO RETRANS, START NOW
        0x00, // length[15:8] (0 = use packet handler)
        0x00, // length[7:0]
    };

    dev->state = TX;

    err = send_command(dev, CMD_START_TX, sizeof(tx_config), tx_config);

    if (err) {
        dev->state = IDLE;
        return err;
    }

    err = wait_cts(dev);

    if (err) {
        return err;
    }

    err = set_property(dev, PROP_PKT_GROUP, PROP_PKT_FIELD_2_LENGTH_12_8,
                 ((MAX_PACKET_SIZE) >> 8) & 0xFF);
    if (err) {
      return err;
    }

    err = set_property(dev, PROP_PKT_GROUP, PROP_PKT_FIELD_2_LENGTH_7_0,
                 (MAX_PACKET_SIZE) & 0xFF);
    if (err) {
      return err;
    }

    // Clear the RX FIFO
    uint8_t fifo_cmd_buf = CLR_RX_FIFO;
    err = send_command(dev, CMD_FIFO_INFO, sizeof(fifo_cmd_buf), &fifo_cmd_buf);

    if (err) {
        return err;
    }

    err = wait_cts(dev);

    if (err) {
        return err;
    }

    return 0;
}

int si446x_set_tx_pwr(struct si446x_device *dev, uint8_t pwr)
{
    return set_property(dev, PROP_PA_GROUP, PROP_PA_PWR_LVL, pwr);
}

int si446x_cfg_gpio(struct si446x_device *dev, uint8_t gpio0, uint8_t gpio1,
                    uint8_t gpio2, uint8_t gpio3)
{
    uint8_t gpio_cfg[] = {
        gpio0,
        gpio1,
        gpio2,
        gpio3,
        0x00, // NIRQ
        0x00, // SDO
        0x00, // Drive strength (strongest)
    };

    int err = send_command(dev, CMD_GPIO_PIN_CFG, sizeof(gpio_cfg), gpio_cfg);

    if (err) {
        return err;
    }

    return wait_cts(dev);
}

int si446x_set_mod_type(struct si446x_device *dev, uint8_t mod_type) {
    return set_property(dev, PROP_MODEM_GROUP, PROP_MODEM_MOD_TYPE, mod_type);
}

int si446x_rx_timeout(struct si446x_device *dev)
{
    dev->rx_timeout = true;
    return ESUCCESS;
}

//int si446x_get_temp(struct si446x_device *dev, int *temp) {
//
//    int err;
//
//    *temp = 0;
//
//    uint8_t adc_config[] = {
//        0x10 // TEMP only
//    };
//
//    uint8_t adc_results[]
//
//    send_command(dev, CMD_START_TX, sizeof(tx_config), tx_config);
//}
