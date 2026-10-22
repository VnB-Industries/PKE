/* Step 2: recognise the enrolled fob behind its rotating resolvable private address. */
#ifndef RPA_H
#define RPA_H

#include <stdbool.h>
#include <zephyr/bluetooth/addr.h>

int rpa_init(void);

/* True when addr is an RPA generated from the enrolled fob IRK. */
bool rpa_is_fob(const bt_addr_le_t *addr);

#endif /* RPA_H */
