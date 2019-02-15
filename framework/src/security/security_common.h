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

#ifndef _SECURITY_COMMON_H__
#define _SECURITY_COMMON_H__

#include <tinyara/config.h>

/* #include <security_api.h> */

/* typedef enum { */
/* 	SECURITY_KEYMGR, */
/* 	SECURITY_AUTHENTICATE, */
/* 	SECURITY_CRYPTO, */
/* 	SECURITY_SECURESTORAGE, */
/* 	SECURITY_COMMON, */
/* } security_type; */

/* typedef enum { */
/* 	SECURITY_KEYMGR_DUMMY, */
/* } security_keymgr_request; */

/* typedef enum { */
/* 	SECURITY_AUTH_GEN_RANDOM, */

/* 	SECURITY_AUTH_GEN_CERT, */
/* 	SECURITY_AUTH_SET_CERT, */
/* 	SECURITY_AUTH_GET_CERT, */
/* 	SECURITY_AUTH_REM_CERT, */

/* 	SECURITY_AUTH_GET_RSASIG, */
/* 	SECURITY_AUTH_VER_RSASIG, */

/* 	SECURITY_AUTH_GET_ECDSASIG, */
/* 	SECURITY_AUTH_VER_ECDSASIG, */

/* 	SECURITY_AUTH_GET_HASH, */
/* 	SECURITY_AUTH_GET_HMAC, */

/* 	SECURITY_AUTH_GEN_DHPARAM, */
/* 	SECURITY_AUTH_SET_DHPARAM, */
/* 	SECURITY_AUTH_COM_DHPARAM, */

/* 	SECURITY_AUTH_GEN_ECDHKEY, */
/* 	SECURITY_AUTH_COM_ECDHKEY, */
/* } security_auth_request; */

/* typedef enum { */
/* 	SECURITY_CRYPTO_DUMMY, */
/* } security_crypto_request; */

/* typedef enum { */
/* 	SECURITY_SS_DUMMY, */
/* } security_ss_request; */

/* typedef enum { */
/* 	SECURITY_COMMON_DUMMY, */
/* } security_common_request; */

/* typedef union { */
/* 	security_keymgr_request key_req; */
/* 	security_auth_request auth_req; */
/* 	security_crypto_request crypto_req; */
/* 	security_ss_request ss_req; */
/* 	security_common_request comm_req; */
/* } security_method; */

/* typedef struct _security_param { */
/* 	/\*  random *\/ */
/* 	int size; */

/* 	/\*  certificate *\/ */
/* 	security_csr *csr; */

/* 	/\*  rsa signature *\/ */
/* 	security_rsa_mode rsa_mode; */

/* 	/\*  ecdsa signature *\/ */
/* 	security_ecdsa_curve curve; */

/* 	/\*  common *\/ */
/* 	char *name; */
/* 	security_algorithm algo; */
/* 	security_data *first; */
/* 	security_data *second; */

/* } security_param; */

/* int handle_keymgr_msg(security_keymgr_request req, security_param *data); */
/* int handle_auth_msg(security_auth_request req, security_param *data); */
/* int handle_crypto_msg(security_crypto_request req, security_param *data); */
/* int handle_ss_msg(security_ss_request req, security_param *data); */

#endif // _SECURITY_COMMON_H__
