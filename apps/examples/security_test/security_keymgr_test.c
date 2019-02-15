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
#include <stdio.h>
#include <security/security_api.h>
#include "security_test_utils.h"

#if 0
#define HMACSHA256_KEY "hmacsha256key"
#define AES128_KEY "aes128key"
#define AES128SET_KEY "aes128setkey"
#define RSA1024_KEY "rsa1024key"
#else
#define HMACSHA256_KEY "ss/01"
#define AES128_KEY "ss/02"
#define AES128SET_KEY "ss/03"
#define RSA1024_KEY "ss/04"
#endif

void
test_keymanager(void)
{
	security_data hash_gen_key;
    security_data aes_gen_key;
    security_data rsa_gen_key;
    security_data get_key;
    security_data aes_set_key;

    aes_set_key.data = "1234567890123456";
    aes_set_key.length = 16;

    printf("  . See Initialize ...");
    fflush(stdout);

    if (0 != security_init()) {
        printf("Fail\n  ! security_init\n");
        return;
	}
    printf("ok\n");

    printf("  . SEE Generate Key : HMAC_SHA256 ...\n");
    fflush(stdout);

    if (0 != keymgr_generate_key(HMAC_SHA256, HMACSHA256_KEY)) {
        printf("Fail\n  ! keymgr_generate_key\n");
        goto exit;
    }
    printf("ok\n");

    printf("  . SEE Generate Key : AES128 ...\n");
    fflush(stdout);

    if (0 != keymgr_generate_key(AES_128, AES128_KEY)) {
        printf("Fail\n  ! keymgr_generate_key\n");
        goto exit;
    }
    printf("ok\n");

    printf("  . SEE Generate Key : RSA1024 ...\n");
    fflush(stdout);

    if (0 != keymgr_generate_key(RSA_1024, RSA1024_KEY)) {
        printf("Fail\n  ! keymgr_generate_key\n");
        goto exit;
    }
    printf("ok\n");
    // PrintBuffer("RSA1024 Public key", rsa_gen_key.data, rsa_gen_key.length);

    printf("  . SEE Get Publickey ...\n");
    fflush(stdout);

	security_algorithm key_type = UNKNOWN_ALGO;
    if (0 != keymgr_get_key(&key_type, RSA1024_KEY, &get_key)) {
        printf("Fail\n  ! keymgr_get_pubkey\n");
        goto exit;
    }
    printf("ok\n");
    PrintBuffer("RSA1024 Public key", get_key.data, get_key.length);

    printf("  . SEE Set Key : AES128 ...\n");
    fflush(stdout);

    if (0 != keymgr_set_key(AES_128, AES128SET_KEY, &aes_set_key, NULL)) {
        printf("Fail\n  ! keymgr_set_key\n");
        goto exit;
    }
    printf("ok\n");

    printf("  . SEE Remove Key : HMAC_SHA256 ...\n");
    fflush(stdout);

    if (0 != keymgr_remove_key(HMAC_SHA256, HMACSHA256_KEY)) {
        printf("Fail\n  ! keymgr_remove_key\n");
        goto exit;
    }
    printf("ok\n");

    printf("  . SEE Remove Key : AES128 ...\n");
    fflush(stdout);

    if (0 != keymgr_remove_key(AES_128, AES128_KEY)) {
        printf("Fail\n  ! keymgr_remove_key\n");
        goto exit;
    }
    printf("ok\n");

    printf("  . SEE Remove Key : RSA1024 ...\n");
    fflush(stdout);

    if (0 != keymgr_remove_key(RSA_1024, RSA1024_KEY)) {
        printf("Fail\n  ! keymgr_remove_key\n");
        goto exit;
    }
    printf("ok\n");

    printf("  . SEE Remove Key : SET_AES128 ...\n");
    fflush(stdout);

    if (0 != keymgr_remove_key(AES_128, AES128SET_KEY)) {
        printf("Fail\n  ! keymgr_remove_key\n");
        goto exit;
    }
    printf("ok\n");

exit:
	free_security_data(&hash_gen_key);
	free_security_data(&aes_gen_key);
	free_security_data(&rsa_gen_key);
	free_security_data(&get_key);

	printf("  . See Deinitialize ...\n");
	security_deinit();
	printf("ok\n");
	fflush(stdout);

    return;
}
