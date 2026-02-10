#ifndef HC05_BLUETOOTH_H
#define HC05_BLUETOOTH_H

#include "usart.h"
#include <stdint.h>

void HC05_Init(void);
void HC05_RxCallback(void);

#endif /* HC05_BLUETOOTH_H */
