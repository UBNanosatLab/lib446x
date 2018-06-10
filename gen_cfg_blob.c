#if 0

#include <stdint.h>
#include <stdio.h>

// Uncomment ONLY 1
#define SI4463
//#define SI4464

#ifdef SI4464
#include "radio_config_Si4464.h"
#endif

#ifdef SI4463
#include "radio_config_Si4463.h"
#endif


uint8_t cfg[] = RADIO_CONFIGURATION_DATA_ARRAY;
int main(int argc, char const *argv[]) {

    int i = cfg[0]; // Skip power up
    int len = cfg[++i];

    #ifdef SI4464
    printf("const uint8_t si4464_cfg[] = {\n");
    #endif
    #ifdef SI4463
    printf("const uint8_t si4463_cfg[] = {\n");
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
