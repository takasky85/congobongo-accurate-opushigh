/* Minimal but complete Z80 core (documented + common undocumented behaviour).
 * Written for the Congo Bongo sound-board proof of concept.  Cycle counts
 * follow the standard Zilog timings (no wait states). */
#ifndef Z80_H
#define Z80_H
#include <stdint.h>

typedef union { uint16_t w; struct { uint8_t l, h; } b; } reg16;

typedef struct Z80 {
    reg16 af, bc, de, hl, af2, bc2, de2, hl2, ix, iy, sp, pc;
    uint8_t i, r;
    int iff1, iff2, im, halted, ei_delay;
    int irq;                     /* HOLD_LINE style: cleared on acknowledge */
    uint64_t cycles;
    void *ud;
    uint8_t (*rd)(void *ud, uint16_t a);
    void    (*wr)(void *ud, uint16_t a, uint8_t v);
    uint8_t (*in)(void *ud, uint16_t a);
    void    (*out)(void *ud, uint16_t a, uint8_t v);
} Z80;

void z80_reset(Z80 *z);
int  z80_step(Z80 *z);           /* executes one instruction (or interrupt), returns cycles */
#endif
