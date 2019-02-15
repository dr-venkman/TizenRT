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
/*
 * Test Crypto API
 */

#include <tinyara/config.h>

#include <stdio.h>
#include <security/security_api.h>
#include "security_test_utils.h"

/*  security api doesn't support key name now. it'll be supported later
 * currently ss/01 will be convert to slot 1.
 */
#if 0
#define AES128_KEY "aes128key"
#define RSA1024_KEY "rsa1024key"
#else
#define AES128_KEY "ss/01"
#define RSA1024_KEY "ss/02"
#endif

void
test_crypto(void)
{
	security_data iv;
    security_data input;
    security_data aes_gen_key;
    security_data aes_enc_data;
    security_data aes_dec_data;
    security_data rsa_gen_key;
    security_data rsa_enc_data;
    security_data rsa_dec_data;

    iv.data="1234567890123456";
    iv.length = 16;

    input.data="1234567890123456";
    input.length=16;

    printf("  . See Initialize ...\n");
    fflush(stdout);
    if (0 != security_init()) {
        printf("Fail\n  ! security_init\n");
		return;
    }
    printf("ok\n");

    printf("  . SEE Generate Key : AES128 ...\n");
    fflush(stdout);
    if (0 != keymgr_generate_key(AES_128, AES128_KEY)) {
        printf("Fail\n  ! keymgr_generate_key\n");
        goto exit;
    }
    printf("ok\n");

	printf("  . SEE Get Key");

    printf("  . SEE AES Encryption ...\n");
    fflush(stdout);
    if (0 != crypto_aes_encryption(AES_ECB_NOPAD, AES128_KEY, &iv, &input, &aes_enc_data)) {
        printf("Fail\n  ! crypto_aes_encryption\n");
        goto exit;
    }
    printf("ok\n");
    PrintBuffer("Input", input.data, input.length);
    PrintBuffer("Enc Data", aes_enc_data.data, aes_enc_data.length);

    printf("  . SEE AES Decryption ...\n");
    fflush(stdout);
    if (0 != crypto_aes_decryption(AES_ECB_NOPAD, AES128_KEY, &iv, &aes_enc_data, &aes_dec_data )) {
        printf("Fail\n  ! security_aes_decryption\n");
        goto exit;
    }
    printf("ok\n");
    PrintBuffer("Enc Data", aes_enc_data.data, aes_enc_data.length);
    PrintBuffer("Dec Data", aes_dec_data.data, aes_dec_data.length);

    printf("  . SEE Remove Key : AES128 ...\n");
    fflush(stdout);
    if (0 != keymgr_remove_key(AES_128, AES128_KEY)) {
        printf("Fail\n  ! keymgr_remove_key\n");
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

    printf("  . SEE RSA Encryption ...\n");
    fflush(stdout);
    if (0 != crypto_rsa_encryption(RSAES_PKCS1_V1_5, RSA1024_KEY, &input, &rsa_enc_data)) {
        printf("Fail\n  ! crypto_rsa_encryption\n");
        goto exit;
    }
    printf("ok\n");
    PrintBuffer("Input", input.data, input.length);
    PrintBuffer("Enc Data", rsa_enc_data.data, rsa_enc_data.length);

    printf("  . SEE RSA Decryption ...\n");
    fflush(stdout);
    if (0 != crypto_rsa_decryption(RSAES_PKCS1_V1_5, RSA1024_KEY, &rsa_enc_data, &rsa_dec_data )) {
        printf("Fail\n  ! crypto_rsa_decryption\n");
        goto exit;
    }
    printf("ok\n");
    PrintBuffer("Dec Data", rsa_dec_data.data, rsa_dec_data.length);

    printf("  . SEE Remove Key : RSA1024 ...\n");
    fflush(stdout);

    if (0 != keymgr_remove_key(RSA_1024, RSA1024_KEY)) {
        printf("Fail\n  ! keymgr_remove_key\n");
        goto exit;
    }
    printf("ok\n");

exit:
	free_security_data(&aes_gen_key);
	free_security_data(&aes_enc_data);
	free_security_data(&aes_dec_data);
	free_security_data(&rsa_gen_key);
	free_security_data(&rsa_enc_data);
	free_security_data(&rsa_dec_data);

	printf("  . See Deinitialize ...\n");
	security_deinit();
	printf("ok\n");
	fflush(stdout);

    return;
}
