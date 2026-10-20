#pragma once
// Durum makinesi: INIT -> MANUAL <-> AUTO, herhangi biri -> FAULT
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    MODE_INIT   = 0,
    MODE_MANUAL = 1,
    MODE_AUTO   = 2,
    MODE_FAULT  = 3,
};

void    fsm_init(void);
uint8_t fsm_mode(void);
int     fsm_request(uint8_t mode);      // 1 kabul, 0 ret
void    fsm_fault(uint16_t code);       // arıza bitini ekler, FAULT'a geçer
void    fsm_clear_fault(void);          // FAULT -> MANUAL
uint16_t fsm_fault_code(void);
const char *fsm_mode_name(uint8_t mode);

#ifdef __cplusplus
}
#endif
