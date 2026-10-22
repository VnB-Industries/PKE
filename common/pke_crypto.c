#include <errno.h>
#include <string.h>

#include <psa/crypto.h>
#include <zephyr/sys/byteorder.h>

#include "keys.h"
#include "pke_crypto.h"

static const uint8_t shared_secret[PKE_KEY_LEN] = PKE_SHARED_SECRET_INIT;
static psa_key_id_t secret_key_id;

#define PROOF_INPUT_LEN (PKE_LABEL_LEN + 2 * PKE_NONCE_LEN)

int pke_crypto_init(void)
{
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;

	if (psa_crypto_init() != PSA_SUCCESS) {
		return -EIO;
	}

	psa_set_key_type(&attr, PSA_KEY_TYPE_HMAC);
	psa_set_key_bits(&attr, PSA_BYTES_TO_BITS(sizeof(shared_secret)));
	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_SIGN_MESSAGE | PSA_KEY_USAGE_VERIFY_MESSAGE);
	psa_set_key_algorithm(&attr, PSA_ALG_HMAC(PSA_ALG_SHA_256));

	if (psa_import_key(&attr, shared_secret, sizeof(shared_secret), &secret_key_id) !=
	    PSA_SUCCESS) {
		return -EIO;
	}
	return 0;
}

int pke_random(uint8_t *buf, size_t len)
{
	return psa_generate_random(buf, len) == PSA_SUCCESS ? 0 : -EIO;
}

static void proof_input(uint8_t out[PROOF_INPUT_LEN], const char *label,
			const uint8_t nonce_c[PKE_NONCE_LEN], const uint8_t nonce_f[PKE_NONCE_LEN])
{
	memcpy(out, label, PKE_LABEL_LEN);
	memcpy(out + PKE_LABEL_LEN, nonce_c, PKE_NONCE_LEN);
	memcpy(out + PKE_LABEL_LEN + PKE_NONCE_LEN, nonce_f, PKE_NONCE_LEN);
}

int pke_proof(const char *label, const uint8_t nonce_c[PKE_NONCE_LEN],
	      const uint8_t nonce_f[PKE_NONCE_LEN], uint8_t mac[PKE_MAC_LEN])
{
	uint8_t input[PROOF_INPUT_LEN];
	size_t mac_len;

	proof_input(input, label, nonce_c, nonce_f);
	if (psa_mac_compute(secret_key_id, PSA_ALG_HMAC(PSA_ALG_SHA_256), input, sizeof(input),
			    mac, PKE_MAC_LEN, &mac_len) != PSA_SUCCESS) {
		return -EIO;
	}
	return 0;
}

int pke_proof_verify(const char *label, const uint8_t nonce_c[PKE_NONCE_LEN],
		     const uint8_t nonce_f[PKE_NONCE_LEN], const uint8_t mac[PKE_MAC_LEN])
{
	uint8_t input[PROOF_INPUT_LEN];
	psa_status_t status;

	proof_input(input, label, nonce_c, nonce_f);
	status = psa_mac_verify(secret_key_id, PSA_ALG_HMAC(PSA_ALG_SHA_256), input, sizeof(input),
				mac, PKE_MAC_LEN);
	if (status == PSA_ERROR_INVALID_SIGNATURE) {
		return -EACCES;
	}
	return status == PSA_SUCCESS ? 0 : -EIO;
}

static int hkdf_sha256(const uint8_t *ikm, size_t ikm_len, const uint8_t *salt, size_t salt_len,
		       const char *info, uint8_t out[PKE_KEY_LEN])
{
	psa_key_derivation_operation_t op = PSA_KEY_DERIVATION_OPERATION_INIT;
	psa_status_t status;

	status = psa_key_derivation_setup(&op, PSA_ALG_HKDF(PSA_ALG_SHA_256));
	if (status == PSA_SUCCESS) {
		status = psa_key_derivation_input_bytes(&op, PSA_KEY_DERIVATION_INPUT_SALT, salt,
							salt_len);
	}
	if (status == PSA_SUCCESS) {
		status = psa_key_derivation_input_bytes(&op, PSA_KEY_DERIVATION_INPUT_SECRET, ikm,
							ikm_len);
	}
	if (status == PSA_SUCCESS) {
		status = psa_key_derivation_input_bytes(&op, PSA_KEY_DERIVATION_INPUT_INFO,
							(const uint8_t *)info, strlen(info));
	}
	if (status == PSA_SUCCESS) {
		status = psa_key_derivation_output_bytes(&op, out, PKE_KEY_LEN);
	}
	psa_key_derivation_abort(&op);
	return status == PSA_SUCCESS ? 0 : -EIO;
}

int pke_session_key(const uint8_t nonce_c[PKE_NONCE_LEN], const uint8_t nonce_f[PKE_NONCE_LEN],
		    uint8_t key[PKE_KEY_LEN])
{
	uint8_t salt[2 * PKE_NONCE_LEN];

	memcpy(salt, nonce_c, PKE_NONCE_LEN);
	memcpy(salt + PKE_NONCE_LEN, nonce_f, PKE_NONCE_LEN);
	return hkdf_sha256(shared_secret, sizeof(shared_secret), salt, sizeof(salt),
			   PKE_INFO_SESSION, key);
}

int pke_sts_key(const uint8_t session_key[PKE_KEY_LEN], const uint8_t nonce_r[PKE_NONCE_LEN],
		uint8_t sts_key[PKE_KEY_LEN])
{
	return hkdf_sha256(session_key, PKE_KEY_LEN, nonce_r, PKE_NONCE_LEN, PKE_INFO_STS, sts_key);
}

uint32_t pke_key_id(const uint8_t key[PKE_KEY_LEN])
{
	uint8_t hash[PSA_HASH_LENGTH(PSA_ALG_SHA_256)];
	size_t hash_len;

	if (psa_hash_compute(PSA_ALG_SHA_256, key, PKE_KEY_LEN, hash, sizeof(hash), &hash_len) !=
	    PSA_SUCCESS) {
		return 0;
	}
	return sys_get_be32(hash);
}
