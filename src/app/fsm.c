#include "fsm.h"

static uint8_t  s_mode = MODE_INIT;
static uint16_t s_fault = 0;

void fsm_init(void) { s_mode = MODE_INIT; s_fault = 0; }

uint8_t fsm_mode(void) { return s_mode; }

int fsm_request(uint8_t mode)
{
    if (s_mode == MODE_FAULT) return 0;                 // önce arıza temizlenir
    if (mode == MODE_MANUAL) { s_mode = MODE_MANUAL; return 1; }
    if (mode == MODE_AUTO) {
        if (s_mode != MODE_MANUAL) return 0;            // AUTO'ya yalnız MANUAL'dan
        s_mode = MODE_AUTO; return 1;
    }
    return 0;
}

void fsm_fault(uint16_t code)
{
    s_fault |= code;
    s_mode = MODE_FAULT;
}

void fsm_clear_fault(void)
{
    s_fault = 0;
    if (s_mode == MODE_FAULT) s_mode = MODE_MANUAL;
}

uint16_t fsm_fault_code(void) { return s_fault; }

const char *fsm_mode_name(uint8_t mode)
{
    switch (mode) {
    case MODE_INIT:   return "INIT";
    case MODE_MANUAL: return "MANUAL";
    case MODE_AUTO:   return "AUTO";
    case MODE_FAULT:  return "FAULT";
    default:          return "?";
    }
}
