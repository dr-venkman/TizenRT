#include <tinyara/config.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <tinyara/seclink.h>

#ifdef SECLINK_PATH
#undef SECLINK_PATH
#endif
#define SECLINK_PATH "/dev/seclink"

#define SL_TAG "seclink"

#define SL_LOG printf

#define SL_ERR(fd)														\
	do {																\
		SL_LOG("[ERR:%s] %s %s:%d ret(%d) code(%s)\n",					\
			   SL_TAG, __FUNCTION__, __FILE__, __LINE__, fd, strerror(errno)); \
	} while(0)

#define SL_CALL(hnd, code, param)						\
	do {																\
		int res = ioctl(hnd->fd, code, (unsigned long)((uintptr_t)&param) ); \
		if(res < 0) {													\
			SL_ERR(res);												\
			return -1;													\
		}																\
	} while (0)


struct _seclink_s_ {
	int fd;
};

#define SL_CLOSE(fd)							\
	do{											\
		close(fd);								\
		fd = -1;								\
	} while (0)


#define SL_CHECK_VALID(hnd)									\
	do{														\
		if (!hnd || ((struct _seclink_s_ *)hnd)->fd <= 0) {	\
			return -1;										\
		}													\
	} while (0)

/*  Common */
int sl_init(sl_ctx *hnd)
{
	int fd = open(SECLINK_PATH, O_RDWR);
	if (fd < 0) {
		SL_ERR(fd);
		return -1;
	}

	struct _seclink_s_ *handle = (struct _seclink_s_ *)malloc(sizeof(struct _seclink_s_));
	if (!handle) {
		SL_ERR(fd);
		close(fd);
		return -1;
	}

	handle->fd = fd;
	*hnd = handle;

	return 0;
}

int sl_deinit(sl_ctx hnd)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	SL_CLOSE(sl->fd);

	free(sl);

	return 0;
}



/*  key manager */
int sl_set_key(sl_ctx hnd, hal_key_type mode, uint32_t key_idx, hal_data *key, hal_data *prikey)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	struct seclink_key_info info = {mode, key_idx, key, prikey};
	struct seclink_req req = {.req_type.key = &info, 0};

	SL_CALL(sl, SECLINK_HAL_SETKEY, req);

	return req.res;
}

int sl_get_key(sl_ctx hnd, hal_key_type mode, uint32_t key_idx, _OUT_ hal_data *key)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	struct seclink_key_info info = {mode, key_idx, key, NULL};
	struct seclink_req req = {.req_type.key = &info, 0};

	SL_CALL(sl, SECLINK_HAL_GETKEY, req);

	return req.res;
}

int sl_remove_key(sl_ctx hnd, hal_key_type mode, uint32_t key_idx)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	struct seclink_key_info info = {mode, key_idx, NULL, NULL};
	struct seclink_req req = {.req_type.key = &info, 0};

	SL_CALL(sl, SECLINK_HAL_REMOVEKEY, req);

	return req.res;
}

int sl_generate_key(sl_ctx hnd, hal_key_type mode, uint32_t key_idx)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	struct seclink_key_info info = {mode, key_idx, NULL, NULL};
	struct seclink_req req = {.req_type.key = &info, 0};

	SL_CALL(sl, SECLINK_HAL_GENERATEKEY, req);

	return req.res;
}


/*  Authenticate */
int sl_generate_random(sl_ctx hnd, uint32_t len, _OUT_ hal_data *random)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	struct seclink_auth_info info = {.auth_type.random_len = len, -1, random, .auth_data.data = NULL};
	struct seclink_req req = {.req_type.auth = &info, 0};

	SL_CALL(sl, SECLINK_HAL_GENERATERANDOM, info);

	return req.res;
}

int sl_get_hash(sl_ctx hnd, hal_hash_type mode, hal_data *input, _OUT_ hal_data *hash)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	struct seclink_auth_info info = {.auth_type.hash_type = mode, -1, input, .auth_data.data = hash};
	struct seclink_req req = {.req_type.auth = &info, 0};

	SL_CALL(sl, SECLINK_HAL_GETHASH, req);

	return req.res;
}

int sl_get_hmac(sl_ctx hnd, hal_hmac_type mode, hal_data *input, uint32_t key_idx, _OUT_ hal_data *hmac)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	struct seclink_auth_info info = {.auth_type.hmac_type = mode, key_idx, input, .auth_data.data = hmac};
	struct seclink_req req = {.req_type.auth = &info, 0};

	SL_CALL(sl, SECLINK_HAL_GETHMAC, req);

	return req.res;
}

int sl_rsa_sign_md(sl_ctx hnd, hal_rsa_mode mode, hal_data *hash, uint32_t key_idx, _OUT_ hal_data *sign)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	struct seclink_auth_info info = {.auth_type.rsa_type = mode, key_idx, hash, .auth_data.data = sign};
	struct seclink_req req = {.req_type.auth = &info, 0};

	SL_CALL(sl, SECLINK_HAL_RSASIGNMD, req);

	return req.res;
}

int sl_rsa_verify_md(sl_ctx hnd, hal_rsa_mode mode, hal_data *hash, hal_data *sign, uint32_t key_idx)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	struct seclink_auth_info info = {.auth_type.rsa_type = mode, key_idx, hash, .auth_data.data = sign};
	struct seclink_req req = {.req_type.auth = &info, 0};

	SL_CALL(sl, SECLINK_HAL_RSAVERIFYMD, req);

	return req.res;
}

int sl_ecdsa_sign_md(sl_ctx hnd, hal_ecdsa_mode mode, hal_data *hash, uint32_t key_idx, _OUT_ hal_data *sign)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	struct seclink_auth_info info = {.auth_type.ecdsa_type = mode, key_idx, hash, .auth_data.data = sign};
	struct seclink_req req = {.req_type.auth = &info, 0};

	SL_CALL(sl, SECLINK_HAL_ECDSASIGNMD, req);

	return req.res;
}

int sl_ecdsa_verify_md(sl_ctx hnd, hal_ecdsa_mode mode, hal_data *hash, hal_data *sign, uint32_t key_idx)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	struct seclink_auth_info info = {.auth_type.ecdsa_type = mode, key_idx, hash, .auth_data.data = sign};
	struct seclink_req req = {.req_type.auth = &info, 0};

	SL_CALL(sl, SECLINK_HAL_ECDSAVERIFYMD, req);

	return req.res;
}

int sl_dh_generate_param(sl_ctx hnd, uint32_t dh_idx, _INOUT_ hal_dh_data *dh_param)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	struct seclink_auth_info info = {.auth_type.random_len = 0, -1, NULL, .auth_data.dh_param = dh_param};
	struct seclink_req req = {.req_type.auth = &info, 0};

	SL_CALL(sl, SECLINK_HAL_DHGENERATEPARAM, req);

	return req.res;
}

int sl_dh_compute_shared_secret(sl_ctx hnd, hal_dh_data *dh_param, uint32_t dh_idx, _OUT_ hal_data *shared_secret)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	struct seclink_auth_info info = {.auth_type.random_len = 0, dh_idx, shared_secret, .auth_data.dh_param = dh_param};
	struct seclink_req req = {.req_type.auth = &info, 0};

	SL_CALL(sl, SECLINK_HAL_DHCOMPUTESHAREDSECRET, req);

	return req.res;
}

int sl_ecdh_compute_shared_secret(sl_ctx hnd, hal_ecdh_data *ecdh_mode, uint32_t key_idx, _OUT_ hal_data *shared_secret)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	struct seclink_auth_info info = {.auth_type.random_len = 0, key_idx, shared_secret, .auth_data.ecdh_param = ecdh_mode};
	struct seclink_req req = {.req_type.auth = &info, 0};

	SL_CALL(sl, SECLINK_HAL_ECDHCOMPUTESHAREDSECRET, req);

	return req.res;
}

int sl_set_certificate(sl_ctx hnd, uint32_t cert_idx, hal_data *cert_in)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	struct seclink_auth_info info = {.auth_type.random_len = 0, cert_idx, cert_in, .auth_data.data = NULL};
	struct seclink_req req = {.req_type.auth = &info, 0};

	SL_CALL(sl, SECLINK_HAL_SETCERTIFICATE, req);

	return req.res;
}

int sl_get_certificate(sl_ctx hnd, uint32_t cert_idx, _OUT_ hal_data *cert_out)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	struct seclink_auth_info info = {.auth_type.random_len = 0, cert_idx, cert_out, .auth_data.data = NULL};
	struct seclink_req req = {.req_type.auth = &info, 0};

	SL_CALL(sl, SECLINK_HAL_GETCERTIFICATE, req);

	return req.res;
}

int sl_remove_certificate(sl_ctx hnd, uint32_t cert_idx)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	struct seclink_auth_info info = {.auth_type.random_len = 0, cert_idx, 0, .auth_data.data = NULL};
	struct seclink_req req = {.req_type.auth = &info, 0};

	SL_CALL(sl, SECLINK_HAL_REMOVECERTIFICATE, req);

	return req.res;
}

int sl_get_factorykey_data(sl_ctx hnd, uint32_t key_idx, hal_data *data)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	struct seclink_auth_info info = {.auth_type.random_len = 0, key_idx, data, .auth_data.data = NULL};
	struct seclink_req req = {.req_type.auth = &info, 0};

	SL_CALL(sl, SECLINK_HAL_GETFACTORYKEY, req);

	return req.res;
}


/*  Crypto */
int sl_aes_encrypt(sl_ctx hnd, hal_data *dec_data, hal_aes_param *aes_param, uint32_t key_idx, _OUT_ hal_data *enc_data)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	struct seclink_crypto_info info = {key_idx, dec_data, enc_data, aes_param};
	struct seclink_req req = {.req_type.crypto = &info, 0};

	SL_CALL(sl, SECLINK_HAL_AESENCRYPT, req);

	return req.res;
}

int sl_aes_decrypt(sl_ctx hnd, hal_data *enc_data, hal_aes_param *aes_param, uint32_t key_idx, _OUT_ hal_data *dec_data)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	struct seclink_crypto_info info = {key_idx, enc_data, dec_data, aes_param};
	struct seclink_req req = {.req_type.crypto = &info, 0};

	SL_CALL(sl, SECLINK_HAL_AESDECRYPT, req);

	return req.res;
}

int sl_rsa_encrypt(sl_ctx hnd, hal_data *dec_data, uint32_t key_idx, _OUT_ hal_data *enc_data)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	struct seclink_crypto_info info = {key_idx, dec_data, enc_data, NULL};
	struct seclink_req req = {.req_type.crypto = &info, 0};

	SL_CALL(sl, SECLINK_HAL_RSADECRYPT, req);

	return req.res;
}

int sl_rsa_decrypt(sl_ctx hnd, hal_data *enc_data, uint32_t key_idx, _OUT_ hal_data *dec_data)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	struct seclink_crypto_info info = {key_idx, enc_data, dec_data, NULL};
	struct seclink_req req = {.req_type.crypto = &info, 0};

	SL_CALL(sl, SECLINK_HAL_RSADECRYPT, req);

	return req.res;
}


/*  Secure Storage */
int sl_write_storage(sl_ctx hnd, uint32_t ss_idx, hal_data *data)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	struct seclink_ss_info info = {ss_idx, data};
	struct seclink_req req = {.req_type.ss = &info, 0};

	SL_CALL(sl, SECLINK_HAL_WRITESTORAGE, req);

	return req.res;
}

int sl_read_storage(sl_ctx hnd, uint32_t ss_idx, _OUT_ hal_data *data)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	struct seclink_ss_info info = {ss_idx, data};
	struct seclink_req req = {.req_type.ss = &info, 0};

	SL_CALL(sl, SECLINK_HAL_READSTORAGE, req);

	return req.res;
}

int sl_delete_storage(sl_ctx hnd, uint32_t ss_idx)
{
	SL_CHECK_VALID(hnd);

	struct _seclink_s_ *sl = (struct _seclink_s_ *)hnd;
	struct seclink_ss_info info = {ss_idx, NULL};
	struct seclink_req req = {.req_type.ss = &info, 0};

	SL_CALL(sl, SECLINK_HAL_DELETESTORAGE, req);

	return req.res;
}
