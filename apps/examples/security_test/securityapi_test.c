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
#include <stdlib.h>

#include <tinyara/seclink.h>
#include <security/security_api.h>

/**
 * Debugging
 */
#define SAT_ERR														\
	do {																\
		printf("[ERR] %s\t%s:%d\n", __FUNCTION__, __FILE__, __LINE__);	\
	} while (0)

#define SAT_CALL(func)                     \
    do {                                        \
        int sectest_res = func;                         \
        if (sectest_res < 0) {                          \
            SAT_ERR;                        \
        }                                       \
    } while (0)

#define SECTEST_KEY_IDX 1

extern void test_keymanager(void);
extern void test_crypto(void);
extern void test_securestorage(void);
extern void test_authenticate(void);

int security_api_test(void)
{
	test_keymanager();
	test_crypto();
	test_securestorage();
	test_authenticate();

	return 0;
}

#if 0
void selftest(void)
{
	char *pubkey_data = {"public_sssssssssssstttttttttt"};
	security_data_s pubkey;
	pubkey.data = pubkey_data;
	pubkey.length = sizeof(pubkey_data);

	char *prikey_data = {"private_sssssssssssstttttttttt"};
	security_data_s prikey;
	prikey.data = prikey_data;
	prikey.length = sizeof(prikey_data);

	/* char *hash_data = {"hash_sssssssssssstttttttttt"}; */
	/* security_data_s hash; */
	/* hash.data = hash_data; */
	/* hash.length = sizeof(hash_data); */

	/* char *hmac_data = {"hash_sssssssssssstttttttttt"}; */
	/* security_data_s hmac; */
	/* hmac.data = hmac_data; */
	/* hmac.length = sizeof(hmac_data); */

	/* char *rsahash_data = {"rsahash_sssssssssssstttttttttt"}; */
	/* security_data_s rsahash; */
	/* rsahash.data = rsahash_data; */
	/* rsahash.length = sizeof(rsahash_data); */

	/* char *sign_data = {"sign_sssssssssssstttttttttt"}; */
	/* security_data_s sign; */
	/* sign.data = sign_data; */
	/* sign.length = sizeof(sign_data); */

	/* char *ecdsar_data = {"ecdsa_r_sssssssssssstttttttttt"}; */
	/* security_data_s ecdsar; */
	/* ecdsar.data = ecdsar_data; */
	/* ecdsar.length = sizeof(ecdsar_data); */

	/* char *ecdsas_data = {"ecdsa_s_sssssssssssstttttttttt"}; */
	/* security_data_s ecdsas; */
	/* ecdsas.data = ecdsas_data; */
	/* ecdsas.length = sizeof(ecdsas_data); */

	/* char *dhg_data = {"dh_G_sssssssssssstttttttttt"}; */
	/* security_data_s dhg; */
	/* dhg.data = dhg_data; */
	/* dhg.length = sizeof(dhg_data); */

	/* char *dhp_data = {"dh_P_sssssssssssstttttttttt"}; */
	/* security_data_s dhp; */
	/* dhp.data = dhp_data; */
	/* dhp.length = sizeof(dhp_data); */

	/* char *dhp_data = {"dh_P_sssssssssssstttttttttt"};char *cert_data = {"certificate_sssssssssssstttttttttt"}; */
	/* security_data_s cert; */
	/* cert.data = cert_data; */
	/* cert.length = sizeof(cert_data); */

	char *dec_data = {"decrypted_sssssssssssstttttttttt"};
	security_data_s dec;
	dec.data = dec_data;
	dec.length = sizeof(dec_data);

	char *enc_data = {"encrypted_sssssssssssstttttttttt"};
	security_data_s enc;
	enc.data = enc_data;
	enc.length = sizeof(enc_data);

	char *iv_data = {"iv_sssssssssssstttttttttt"};
	security_data_s iv;
	iv.data = iv_data;
	iv.length = sizeof(iv_data);

	char *ss_data = {"securestorage_sssssssssssstttttttttt"};
	security_data_s ss;
	ss.data = ss_data;
	ss.length = sizeof(ss_data);

	security_data_s input;
	security_data output;

	 /* Initialize handle */
	SAT_CALL(security_init());

    /* Key manager */
	#define SAT_KEY_PATH "ss/01"

	SAT_CALL(keymgr_generate_key(AES_128, SAT_KEY_PATH, &input));

	SAT_CALL(keymgr_set_key(AES_192, SAT_KEY_PATH, &pubkey, &prikey));

	SAT_CALL(keymgr_get_key(AES_192, SAT_KEY_PATH, &input));

	SAT_CALL(keymgr_remove_key(AES_192, SAT_KEY_PATH));

	/* Crypto */
	security_data output;
	security_aes_mode aes_mode = AES_ECB_ISO9797_M1;
	SAT_CALL(crypto_aes_encryption(aes_mode, SAT_KEY_PATH, &iv, &enc_data, &output));
	sat_free(&output);

	SAT_CALL(crypto_aes_decryption(aes_mode, SAT_KEY_PATH, &iv, &dec_data, &output));
	sat_free(&output);

	security_rsa_mode rsa_mode = RSAES_PKCS1_OAEP_MGF1_SHA384;
	SAT_CALL(crypto_rsa_encryption(rsa_mode, SAT_KEY_PATH, &enc_data, &output));
	sat_free(&output);

	SAT_CALL(crypto_rsa_decryption(rsa_mode, SAT_KEY_PATH, &dec_data, &output));
	sat_free(&output);

	/* Secure Storage */
	#define SAT_SS_PATH "ss/02"

	SAT_CALL(ss_write_secure_storage(SAT_SS_PATH, 0, &ss));

	SAT_CALL(ss_read_secure_storage(SAT_SS_PATH, 0, sizeof(ss_data), &output));

	SAT_CALL(ss_delete_secure_storage(SAT_SS_PATH));

	// difference between those.
	uint32_t ss_size = 0;
	SAT_CALL(ss_get_size_secure_storage(SAT_SS_PATH, &ss_size));

	uint32_t num_ss_slot = 0;

	SAT_CALL(ss_get_list_secure_storage( ));


	/* /\* */
	/*  * Authenticate */
	/*  *\/ */
	/* #define SECTEST_RANDOM_LEN 32 */
	/* SAT_CALL(sl_generate_random( )); */

	/* SAT_CALL(sl_get_hash( )); */

	/* SAT_CALL(sl_get_hmac( )); */

	/* hal_rsa_mode rsa_mode = {HAL_RSASSA_PKCS1_PSS_MGF1, HAL_HASH_SHA256}; */
	/* SAT_CALL(sl_rsa_sign_md( )); */

	/* SAT_CALL(sl_rsa_verify_md( )); */

	/* hal_ecdsa_mode ecdsa_mode = {HAL_ECDSA_BRAINPOOL_P512R1, HAL_HASH_SHA224, &ecdsar, &ecdsas}; */
	/* SAT_CALL(sl_ecdsa_sign_md( )); */

	/* SAT_CALL(sl_ecdsa_verify_md( )); */

	/* hal_dh_data dh_mode = {HAL_DH_1024, &dhg, &dhp, NULL}; */
	/* SAT_CALL(sl_dh_generate_param( )); */

	/* SAT_CALL(sl_dh_compute_shared_secret( )); */

	/* hal_ecdh_data ecdh_mode = {HAL_ECDSA_BRAINPOOL_P512R1, NULL, NULL}; */
	/* SAT_CALL(sl_ecdh_compute_shared_secret( )); */

	/* SAT_CALL(sl_set_certificate( )); */

	/* security_data cert_get; */
	/* SAT_CALL(sl_get_certificate( )); */

	/* SAT_CALL(sl_remove_certificate( )); */

	/* security_data fackey; */
	/* SAT_CALL(sl_get_factorykey_data( )); */


	/*  Terminate handle */
	SAT_CALL(security_deinit());

	return 0;
}
#endif
