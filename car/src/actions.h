/* Unlock and lock decisions (README steps 5 and 8). */
#ifndef ACTIONS_H
#define ACTIONS_H

#include <stdbool.h>
#include <stdint.h>

/* Lock once the fob is beyond the lock radius for this many consecutive rounds. */
#define LOCK_K_ROUNDS 2

/* Step 5: decides and performs the unlock. distance_cm < 0 means unknown. True when unlocked. */
bool unlock_decide(uint32_t session_key_id, int32_t distance_cm);

/* Step 8: decides and performs the lock. True when locked. */
bool lock_decide(int rounds_above_radius);

#endif /* ACTIONS_H */
