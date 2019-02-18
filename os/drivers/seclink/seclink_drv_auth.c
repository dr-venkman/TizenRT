#include <tinyara/config.h>

#include <stdio.h>
#include <tinyara/seclink.h>
#include "seclink_drv_req.h"

int hd_handle_auth_reqeust(int cmd, unsigned long arg)
{
	struct seclink_req *req = (struct seclink_req *)arg;
	if (!req) {
		return -1;
	}

	struct seclink_auth_info *info = req->req_type.auth;
	if (!info) {
		return -1;
	}

	switch(cmd) {
	case SECLINK_HAL_GENERATERANDOM:
		req->res = hal_generate_random(info->auth_type.random_len, info->data);
		break;
	case SECLINK_HAL_GETHASH:
		req->res = hal_get_hash(info->auth_type.hash_type, info->data, info->auth_data.data);
		break;
	case SECLINK_HAL_GETHMAC:
		req->res = hal_get_hmac(info->auth_type.hmac_type, info->data, info->key_idx, info->auth_data.data);
		break;
	case SECLINK_HAL_RSASIGNMD:
		req->res = hal_rsa_sign_md(info->auth_type.rsa_type, info->data, info->key_idx, info->auth_data.data);
		break;
	case SECLINK_HAL_RSAVERIFYMD:
		req->res = hal_rsa_verify_md(info->auth_type.rsa_type, info->data, info->auth_data.data, info->key_idx);
		break;
	case SECLINK_HAL_ECDSASIGNMD:
		req->res = hal_ecdsa_sign_md(info->auth_type.ecdsa_type, info->data, info->key_idx, info->auth_data.data);
		break;
	case SECLINK_HAL_ECDSAVERIFYMD:
		req->res = hal_ecdsa_verify_md(info->auth_type.ecdsa_type, info->data, info->auth_data.data, info->key_idx);
		break;
	case SECLINK_HAL_DHGENERATEPARAM:
		req->res = hal_dh_generate_param(info->key_idx, info->auth_data.dh_param);
		break;
	case SECLINK_HAL_DHCOMPUTESHAREDSECRET:
		req->res = hal_dh_compute_shared_secret(info->auth_data.dh_param, info->key_idx, info->data);
		break;
	case SECLINK_HAL_ECDHCOMPUTESHAREDSECRET:
		req->res = hal_ecdh_compute_shared_secret(info->auth_data.ecdh_param, info->key_idx, info->data);
		break;
	case SECLINK_HAL_SETCERTIFICATE:
		req->res = hal_set_certificate(info->key_idx, info->data);
		break;
	case SECLINK_HAL_GETCERTIFICATE:
		req->res = hal_get_certificate(info->key_idx, info->data);
		break;
	case SECLINK_HAL_REMOVECERTIFICATE:
		req->res = hal_remove_certificate(info->key_idx);
		break;
	case SECLINK_HAL_GETFACTORYKEY:
		req->res = hal_get_factorykey_data(info->key_idx, info->data);
		break;
	}
	return 0;
}
