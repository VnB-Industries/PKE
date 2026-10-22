/* Unlock and lock decisions. */
#ifndef ACTIONS_H
#define ACTIONS_H

/* Decides and performs the unlock once the fob is detected close by. True when unlocked. */
bool unlock_decide(int rssi_dbm);

/* Decides and performs the lock once the fob has walked away. True when locked. */
bool lock_decide(void);

#endif /* ACTIONS_H */
