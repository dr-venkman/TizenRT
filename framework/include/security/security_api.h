#ifndef __SECURITY_API_H__
#define  __SECURITY_API_H__

typedef struct _security_data {
	void *data;
	unsigned int length;
} security_data;

typedef enum {
	SECURITY_OK,

	SECURITY_ERROR,
	SECURITY_ALLOC_ERROR,
	SECURITY_INVALID_INPUT_PARAMS,
	SECURITY_INVALID_CERT_INDEX,
	SECURITY_INVALID_KEY_INDEX,
	SECURITY_INVALID_BUFFER_SIZE,
	SECURITY_MUTEX_INIT_ERROR,
	SECURITY_MUTEX_LOCK_ERROR,
	SECURITY_MUTEX_UNLOCK_ERROR,
	SECURITY_MUTEX_FREE_ERROR,

	SECURITY_WRITE_CERT_ERROR,
	SECURITY_READ_CERT_ERROR,
	SECURITY_GET_HASH_ERROR,
	SECURITY_GET_RANDOM_ERROR,
	SECURITY_ECDSA_SIGN_ERROR,
	SECURITY_ECDSA_VERIFY_ERROR,
	SECURITY_ECDH_COMPUTE_ERROR,

	SECURITY_NOT_SUPPORT,
} security_error;

typedef struct _security_csr {
	unsigned char 	issuer_country [128];
	unsigned char 	issuer_organization [128];
	unsigned char 	issuer_cn [128];
	unsigned char 	issuer_keyname [20];
	unsigned int 	issuer_algorithm;
	unsigned char 	subject_country [128];
	unsigned char 	subject_organization [128];
	unsigned char 	subject_cn [128];
	unsigned char 	subject_keyname [20];
	unsigned int 	subject_algorithm;
	unsigned int 	serial;
	unsigned int 	cert_years;
} security_csr;

typedef enum {
	RSAES_PKCS1_V1_5 = 0,
	RSAES_PKCS1_OAEP_MGF1_SHA1 = 1,
	RSAES_PKCS1_OAEP_MGF1_SHA224 = 2,
	RSAES_PKCS1_OAEP_MGF1_SHA256 = 3,
	RSAES_PKCS1_OAEP_MGF1_SHA384 = 4,
	RSAES_PKCS1_OAEP_MGF1_SHA512 = 5,
	RSASSA_PKCS1_V1_5_MD5 = 6,
	RSASSA_PKCS1_V1_5_SHA1 = 7,
	RSASSA_PKCS1_V1_5_SHA224 = 8,
	RSASSA_PKCS1_V1_5_SHA256 = 9,
	RSASSA_PKCS1_V1_5_SHA384 = 10,
	RSASSA_PKCS1_V1_5_SHA512 = 11,
	RSASSA_PKCS1_PSS_MGF1_SHA1 = 12,
	RSASSA_PKCS1_PSS_MGF1_SHA224 = 13,
	RSASSA_PKCS1_PSS_MGF1_SHA256 = 14,
	RSASSA_PKCS1_PSS_MGF1_SHA384 = 15,
	RSASSA_PKCS1_PSS_MGF1_SHA512 = 16,
} security_rsa_mode;

typedef enum {
	ECDSA_BRAINPOOL_P256R1 = 0,
	ECDSA_BRAINPOOL_P384R1 = 1,
	ECDSA_BRAINPOOL_P512R1 = 2,
	ECDSA_SEC_P256R1 = 3,
	ECDSA_SEC_P384R1 = 4,
	ECDSA_SEC_P521R1 = 5,
} security_ecdsa_curve;

typedef enum {
	AES_128,
	AES_192,
	AES_256,
	RSA_1024,
	RSA_2048,
	RSA_3072,
	RSA_4096,
	ECC_BRAINPOOL_P256R1,
	ECC_BRAINPOOL_P384R1,
	ECC_BRAINPOOL_P512R1,
	ECC_SEC_P256R1,
	ECC_SEC_P384R1,
	ECC_SEC_P521R1,
	HASH_MD5,
	HASH_SHA1,
	HASH_SHA224,
	HASH_SHA256,
	HASH_SHA384,
	HASH_SHA512,
	HMAC_SHA1,
	HMAC_SHA224,
	HMAC_SHA256,
	HMAC_SHA384,
	HMAC_SHA512,
	UNKNOWN_ALGO,
} security_algorithm;

typedef enum {
	AES_ECB_NOPAD = 0,
	AES_ECB_ISO9797_M1 = 1,
	AES_ECB_ISO9797_M2 = 2,
	AES_ECB_PKCS5 = 3,
	AES_ECB_PKCS7 = 4,
	AES_CBC_NOPAD = 5,
	AES_CBC_ISO9797_M1 = 6,
	AES_CBC_ISO9797_M2 = 7,
	AES_CBC_PKCS5 = 8,
	AES_CBC_PKCS7 = 9,
	AES_CTR = 10
} security_aes_mode;

typedef struct security_storage_file {
	char 	name [20];
	unsigned int 	attr;
} security_storage_file;

typedef security_storage_file* security_storage_list;

/**
 * Common
 */
int security_init(void);
int security_deinit(void);

/**
 * Authenticate
 */

int auth_generate_random(unsigned int size, security_data *random);
int auth_generate_certificate(const char *cert_name, security_csr *csr, security_data *cert);
int auth_set_certificate(const char *cert_name, security_data *cert);
int auth_get_certificate(const char *cert_name, security_data *cert);
int auth_remove_certificate(const char *cert_name);
int auth_get_rsa_signature(security_rsa_mode mode, const char *key_name, security_data *hash, security_data *sign);
int auth_verify_rsa_signature(security_rsa_mode mode, const char *key_name, security_data *hash, security_data *sign);
int auth_get_ecdsa_signature(security_ecdsa_curve curve, const char *key_name, security_data *hash, security_data *sign);
int auth_verify_ecdsa_signature(security_ecdsa_curve curve, const char *key_name, security_data *hash, security_data *sign);
int auth_get_hash(security_algorithm algo, security_data *data, security_data *hash);
int auth_get_hmac(security_algorithm algo, const char *key_name, security_data *data, security_data *hmac);
int auth_generate_dhparams(security_data *params, security_data *pub);
int auth_set_dhparams(security_data *params, security_data *pub);
int auth_compute_dhparams(security_data *pub, security_data *secret);
int auth_generate_ecdhkey(security_algorithm algo, security_data *pub);
int auth_compute_ecdhkey(security_data *pub, security_data *secret);

/**
 * Crypto
 */
int crypto_aes_encryption(security_aes_mode mode, const char *key_name, security_data *iv, security_data *input, security_data *output);
int crypto_aes_decryption(security_aes_mode mode, const char *key_name, security_data *iv, security_data *input, security_data *output);
int crypto_rsa_encryption(security_rsa_mode mode, const char *key_name, security_data *input, security_data *output);
int crypto_rsa_decryption(security_rsa_mode mode, const char *key_name, security_data *input, security_data *output);

/**
 * Secure Storage
 */
int ss_read_secure_storage(const char *name, unsigned int offset, security_data *data);
int ss_write_secure_storage(const char *name, unsigned int offsest, security_data *data);
int ss_delete_secure_storage(const char *name);
int ss_get_size_secure_storage(const char *name, unsigned int *size);
int ss_get_list_secure_storage(unsigned int *count, security_storage_list *list);

/**
 * Key Manager
 */
int keymgr_generate_key(security_algorithm algo, const char *key_name);
int keymgr_set_key(security_algorithm algo, const char *key_name, security_data *pubkey, security_data *prikey);
int keymgr_get_key(security_algorithm *algo, const char *key_name, security_data *pubkey);
int keymgr_remove_key(security_algorithm algo, const char *key_name);

#endif // __SECURITY_API_H__
