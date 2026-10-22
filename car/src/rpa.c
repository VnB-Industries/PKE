/*
 * Host-side RPA resolution against the enrolled fob IRK.
 *
 * The README plans this through the controller's Filter Accept List, but Zephyr
 * only loads a peer IRK into the resolving list from a real bond. Until the fob
 * is bonded once, resolving here costs one AES block per RPA advert.
 */
#include <errno.h>
#include <string.h>

#include <psa/crypto.h>
#include <zephyr/sys/byteorder.h>

#include "keys.h"
#include "rpa.h"

static psa_key_id_t irk_key_id;

int rpa_init(void)
{
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	uint8_t irk[16] = PKE_FOB_IRK_INIT;
	uint8_t irk_be[16];
	psa_status_t status;

	/* Zephyr keeps the IRK little-endian; AES takes it most significant byte first. */
	sys_memcpy_swap(irk_be, irk, sizeof(irk));

	psa_set_key_type(&attr, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&attr, 128);
	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_ENCRYPT);
	psa_set_key_algorithm(&attr, PSA_ALG_ECB_NO_PADDING);
	status = psa_import_key(&attr, irk_be, sizeof(irk_be), &irk_key_id);

	memset(irk_be, 0, sizeof(irk_be));
	return status == PSA_SUCCESS ? 0 : -EIO;
}

bool rpa_is_fob(const bt_addr_le_t *addr)
{
	const uint8_t *a = addr->a.val;
	uint8_t in[16] = {0};
	uint8_t out[16];
	size_t out_len;

	if (addr->type != BT_ADDR_LE_RANDOM || !BT_ADDR_IS_RPA(&addr->a)) {
		return false;
	}

	/*
	 * ah() from Core spec Vol 3 Part H 2.2.2: hash = e(IRK, padding | prand) mod 2^24.
	 * bt_addr_t is little-endian: prand is val[3..5], hash is val[0..2].
	 */
	in[13] = a[5];
	in[14] = a[4];
	in[15] = a[3];
	if (psa_cipher_encrypt(irk_key_id, PSA_ALG_ECB_NO_PADDING, in, sizeof(in), out, sizeof(out),
			       &out_len) != PSA_SUCCESS) {
		return false;
	}
	return out[15] == a[0] && out[14] == a[1] && out[13] == a[2];
}
