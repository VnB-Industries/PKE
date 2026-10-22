/* Resolves the fob's rotating Resolvable Private Address with its enrolled IRK. */
#ifndef RPA_H
#define RPA_H

#include <stdint.h>

/* Loads the fob IRK from keys.h. Returns 0 or a negative errno. */
int rpa_init(void);

/*
 * True when addr, a random address stored little-endian (as NimBLEAddress::getVal() gives it),
 * is an RPA generated from the fob IRK.
 */
bool rpa_is_fob(const uint8_t addr[6]);

#endif /* RPA_H */
