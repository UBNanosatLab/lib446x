#if 0

#include <stdint.h>
#include <stdio.h>

// Define on command line: -DHEADER=...
#include HEADER


uint8_t cfg[] = RADIO_CONFIGURATION_DATA_ARRAY;
int main(int argc, char const *argv[]) {

    int i = cfg[0]; // Skip power up
    int len = cfg[++i];

    printf("#define CFG_10K_DATA_5K_DEV { \\\n");
    printf(".modem_mod_type_12          = { " #RF_MODEM_MOD_TYPE_12 " }, \")
    printf(".modem_tx_ramp_delay_8          = { " #RF_MODEM_TX_RAMP_DELAY_8 " }, \")
    .modem_tx_ramp_delay_8      = { FILL_IN }, \
    .modem_bcr_osr_1_9          = { FILL_IN }, \
    .modem_afc_gear             = { FILL_IN }, \
    .modem_agc_window_size_9    = { FILL_IN }, \
    .modem_ook_cnt1_9           = { FILL_IN }, \
    .modem_chflt_coe13_7_0_12   = { FILL_IN }, \
    .modem_chflt_coe1_7_0_12    = { FILL_IN }, \
    .modem_chflt_coe7_7_0_12    = { FILL_IN }, \
}

    #ifdef SI4464
    printf("const uint8_t si4464_cfg[] = {\n");
    #endif


    while (len) {
        printf("    0x%02X,", len);
        for (int j = 0; j < len; j++) {
            printf(" 0x%02X,", cfg[++i]);
        }
        len = cfg[++i];
        printf("\n");
    }
    printf("    0x00,\n};\n");

    return 0;
}

#endif
