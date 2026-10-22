/* PSA crypto (CryptoCell-310) helpers for the PKE handshake. All return 0 or a negative errno. */
#ifndef PKE_CRYPTO_H
#define PKE_CRYPTO_H

#include <stddef.h>
#include <stdint.h>

#include "pke_protocol.h"

int pke_crypto_init(void);

int pke_random(uint8_t *buf, size_t len);

/* HMAC(K, label | nonce_C | nonce_F), K being the enrolled shared secret. */
int pke_proof(const char *label, const uint8_t nonce_c[PKE_NONCE_LEN],
	      const uint8_t nonce_f[PKE_NONCE_LEN], uint8_t mac[PKE_MAC_LEN]);

/* Constant-time check of a pke_proof() MAC. Returns -EACCES on mismatch. */
int pke_proof_verify(const char *label, const uint8_t nonce_c[PKE_NONCE_LEN],
		     const uint8_t nonce_f[PKE_NONCE_LEN], const uint8_t mac[PKE_MAC_LEN]);

/* Step 3: HKDF-SHA256(IKM = K, salt = nonce_C | nonce_F, info = PKE_INFO_SESSION). */
int pke_session_key(const uint8_t nonce_c[PKE_NONCE_LEN], const uint8_t nonce_f[PKE_NONCE_LEN],
		    uint8_t key[PKE_KEY_LEN]);

/* Step 7: HKDF-SHA256(IKM = session key, salt = nonce_R, info = PKE_INFO_STS). */
int pke_sts_key(const uint8_t session_key[PKE_KEY_LEN], const uint8_t nonce_r[PKE_NONCE_LEN],
		uint8_t sts_key[PKE_KEY_LEN]);

/*
 * First 4 bytes of SHA-256(key), for logging. Both sides log it, so equal ids
 * prove they derived the same key without ever printing the key. 0 on error.
 */
uint32_t pke_key_id(const uint8_t key[PKE_KEY_LEN]);

#endif /* PKE_CRYPTO_H */
