/*
 * Host-side RPA resolution against the enrolled fob IRK, ported from
 * car/src/rpa.c on nrf52840/main. Costs one AES block (ESP32 hardware AES
 * through mbedTLS) per RPA advert.
 */
#include <errno.h>
#include <string.h>

#include <mbedtls/aes.h>

#include "keys.h"
#include "rpa.h"

static mbedtls_aes_context irk_ctx;

int rpa_init(void)
{
	uint8_t irk[16] = PKE_FOB_IRK_INIT;
	uint8_t irk_be[16];
	int err;

	/* keys.h keeps the IRK little-endian, as Zephyr does; AES takes it most significant byte first. */
	for (int i = 0; i < 16; i++) {
		irk_be[i] = irk[15 - i];
	}

	mbedtls_aes_init(&irk_ctx);
	err = mbedtls_aes_setkey_enc(&irk_ctx, irk_be, 128);

	memset(irk, 0, sizeof(irk));
	memset(irk_be, 0, sizeof(irk_be));
	return err ? -EIO : 0;
}

bool rpa_is_fob(const uint8_t addr[6])
{
	const uint8_t *a = addr;
	uint8_t in[16] = {0};
	uint8_t out[16];

	/* An RPA has 0b01 in its two most significant bits. */
	if ((a[5] & 0xC0) != 0x40) {
		return false;
	}

	/*
	 * ah() from Core spec Vol 3 Part H 2.2.2: hash = e(IRK, padding | prand) mod 2^24.
	 * The address is little-endian: prand is a[3..5], hash is a[0..2].
	 */
	in[13] = a[5];
	in[14] = a[4];
	in[15] = a[3];
	if (mbedtls_aes_crypt_ecb(&irk_ctx, MBEDTLS_AES_ENCRYPT, in, out)) {
		return false;
	}
	return out[15] == a[0] && out[14] == a[1] && out[13] == a[2];
}
