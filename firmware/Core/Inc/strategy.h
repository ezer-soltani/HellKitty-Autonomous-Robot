#ifndef STRATEGY_H
#define STRATEGY_H

#include <stdint.h>

typedef enum {
    STATE_SEARCH,
    STATE_ATTACK,
    STATE_FLEE,
    STATE_IDLE
} Strategy_State_t;

typedef enum {
    ROLE_CHAT,
    ROLE_SOURIS
} Game_Role_t;

void Strategy_Init(void);
void Strategy_Update(void);
void Strategy_ToggleRole(void);
void Strategy_SetRole(Game_Role_t role); // Nouvelle
void Strategy_SetEnabled(uint8_t enabled); // Nouvelle
uint8_t Strategy_IsEnabled(void); // Nouvelle
Strategy_State_t Strategy_GetCurrentState(void);
Game_Role_t Strategy_GetCurrentRole(void);

#endif /* STRATEGY_H */
