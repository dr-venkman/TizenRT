/****************************************************************************
 *
 * Copyright 2019 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/
#include <tinyara/config.h>
#include <stdlib.h>
#include <security_api.h>
#include "security_common.h"
#include "security_utils.h"

int _handle_generate_random(unsigned int size, security_data *random)
{
	SECURITY_ENTRY;

	/*  generate fake data */
	char *data = (char *)malloc(sizeof(size));
	if (!random->data) {
		SECURITY_ERR;
		random->length = 0;
		SECURITY_RETURN(SECURITY_ERROR);
	}
	for (int i = 0 ; i < size; i++) {
		data[i] = i;
	}
	random->data = data;
	random->length = size;

	SECURITY_RETURN(SECURITY_OK);
}

int _handle_generate_certificate(const char *cert_name, security_csr *csr, security_data *cert)
{
	return 0;
}

int _handle_set_certificate(const char *cert_name, security_data cert)
{
	return 0;
}

int _handle_get_certificate(const char *cert_name, security_data *cert)
{
	return 0;
}

int _handle_remove_certificate(const char *cert_name)
{
	return 0;
}

int _handle_get_rsa_signature(security_rsa_mode mode, const char *key_name, security_data hash, security_data *sign)
{
	return 0;
}

int _handle_verify_rsa_signature(security_rsa_mode mode, const char *key_name, security_data hash, security_data sign)
{
	return 0;
}

int _handle_get_ecdsa_signature(security_ecdsa_curve curve, const char *key_name, security_data hash, security_data *sign)
{
	return 0;
}

int _handle_verify_ecdsa_signature(security_ecdsa_curve curve, const char *key_name, security_data hash, security_data sign)
{
	return 0;
}

int _handle_get_hash(security_algorithm algo, security_data data, security_data *hash)
{
	return 0;
}

int _handle_get_hmac(security_algorithm algo, const char *key_name, security_data data, security_data *hmac)
{
	return 0;
}

int _handle_generate_dhparams(security_data *params, security_data *pub)
{
	return 0;
}

int _handle_set_dhparams(security_data params, security_data *pub)
{
	return 0;
}

int _handle_compute_dhparams(security_data pub, security_data *secret)
{
	return 0;
}

int _handle_generate_ecdhkey(security_algorithm algo, security_data *pub)
{
	return 0;
}

int _handle_compute_ecdhkey(security_data pub, security_data *secret)
{
	return 0;
}

/*
 * public
 */
int handle_auth_msg(security_auth_request req, security_param *data)
{
	int res = SECURITY_OK;
	switch (req) {
	case SECURITY_AUTH_GEN_RANDOM:
		res = _handle_generate_random(data->size, data->first);
		break;
	case SECURITY_AUTH_GEN_CERT:
		break;
	case SECURITY_AUTH_SET_CERT:
		break;
	case SECURITY_AUTH_GET_CERT:
		break;
	case SECURITY_AUTH_REM_CERT:
		break;
	case SECURITY_AUTH_GET_RSASIG:
		break;
	case SECURITY_AUTH_VER_RSASIG:
		break;
	case SECURITY_AUTH_GET_ECDSASIG:
		break;
	case SECURITY_AUTH_VER_ECDSASIG:
		break;
	case SECURITY_AUTH_GET_HASH:
		break;
	case SECURITY_AUTH_GET_HMAC:
		break;
	case SECURITY_AUTH_GEN_DHPARAM:
		break;
	case SECURITY_AUTH_SET_DHPARAM:
		break;
	case SECURITY_AUTH_COM_DHPARAM:
		break;
	case SECURITY_AUTH_GEN_ECDHKEY:
		break;
	case SECURITY_AUTH_COM_ECDHKEY:
		break;
	default:
		SECURITY_ERR;
	}

	return res;
}
