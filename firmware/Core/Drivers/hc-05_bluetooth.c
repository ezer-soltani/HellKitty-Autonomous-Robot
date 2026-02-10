#include "hc-05_bluetooth.h"
#include "strategy.h"
#include <string.h>
#include <ctype.h>

#define RX_BUFFER_SIZE 32
static uint8_t rx_byte;
static char cmd_buffer[RX_BUFFER_SIZE];
static uint8_t cmd_idx = 0;

void HC05_Init(void) {
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
}

static void HC05_ParseCommand(char* cmd) {
    for(int i = 0; cmd[i]; i++){
        cmd[i] = toupper((unsigned char)cmd[i]);
    }

    if (strcmp(cmd, "START") == 0) {
        Strategy_SetEnabled(1);
    } else if (strcmp(cmd, "STOP") == 0) {
        Strategy_SetEnabled(0);
    } else if (strcmp(cmd, "CHAT") == 0) {
        Strategy_SetRole(ROLE_CHAT);
    } else if (strcmp(cmd, "SOURIS") == 0) {
        Strategy_SetRole(ROLE_SOURIS);
    }
}

void HC05_RxCallback(void) {
    if (rx_byte == '\n' || rx_byte == '\r') {
        if (cmd_idx > 0) {
            cmd_buffer[cmd_idx] = '\0';
            HC05_ParseCommand(cmd_buffer);
            cmd_idx = 0;
        }
    } else {
        if (cmd_idx < RX_BUFFER_SIZE - 1) {
            cmd_buffer[cmd_idx++] = (char)rx_byte;
        }
    }
    // Relancer la réception pour l'octet suivant
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
}
