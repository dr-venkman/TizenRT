#include <tinyara/config.h>

#include <stdio.h>
#include <tinyara/seclink.h>
#include "seclink_drv_req.h"

int hd_handle_crypto_request(int cmd, unsigned long arg)
{
	struct seclink_req *req = (struct seclink_req *)arg;
	if (!req) {
		return -1;
	}

	struct seclink_crypto_info *info = (struct seclink_crypto_info *)req->req_type.crypto;
	if (!info) {
		return -1;
	}

	printf("crypto request cmd(%x)\n", cmd);

	switch(cmd) {
	case SECLINK_HAL_AESENCRYPT:
		req->res = hal_aes_encrypt(info->input, info->aes_param, info->key_idx, info->output);
		break;
	case SECLINK_HAL_AESDECRYPT:
		req->res = hal_aes_decrypt(info->input, info->aes_param, info->key_idx, info->output);
		break;
	case SECLINK_HAL_RSAENCRYPT:
		req->res = hal_rsa_encrypt(info->input, info->key_idx, info->output);
		break;
	case SECLINK_HAL_RSADECRYPT:
		req->res = hal_rsa_decrypt(info->input, info->key_idx, info->output);
		break;
	}

	return 0;
}
