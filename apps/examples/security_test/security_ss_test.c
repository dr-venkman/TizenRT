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

/*
 * Test Secure Storage APIs
 */

void
test_securestorage(void)
{
	unsigned int storage_size;
    unsigned int count;
    int i;
    security_data input;
    security_data output;
    security_storage_list list;

    input.data="1234567890123456";
    input.length = 16;

    printf("  . See Initialize ...\n");
    fflush(stdout);
    if (0 != security_init()) {
        printf("Fail\n  ! security_init\n");
        return;
	}
    printf("ok\n");

    printf("  . SEE Write Secure Storage ...\n");
    fflush(stdout);
    if (0 != ss_write_secure_storage("storage_test", 0, &input)) {
        printf("Fail\n  ! ss_write_secure_storage\n");
        goto exit;
    }
    printf("ok\n");
    PrintBuffer("input", input.data, input.length);

    printf("  . SEE Get Size of Secure Storage ...\n");
    fflush(stdout);
    if (0 != ss_get_size_secure_storage("storage_test", &storage_size)) {
        printf("Fail\n  ! ss_get_size_secure_storage\n");
        goto exit;
    }
    printf("ok\n");
    printf("storage_test size : %d\n", storage_size);

    printf("  . SEE Read Secure Storage ...\n");
    fflush(stdout);
    if (0 != ss_read_secure_storage("storage_test", 0, &output)) {
        printf("Fail\n  ! ss_read_secure_storage\n");
        goto exit;
    }
    printf("ok\n");
    PrintBuffer("storage_test", output.data, output.length);

    printf("  . SEE Get Secure Storage List ...\n");
    fflush(stdout);
    if (0 != ss_get_list_secure_storage(&count, &list)) {
        printf("Fail\n  ! ss_get_list_secure_storage\n");
        goto exit;
    }
    printf("ok\n");
    printf("[%20s] [%8s]\n", "FILE NAME", "FILE ATTR");
    for (i = 0; i < count; i++) {
        printf("[%20s] [%08x]\n", list[i].name, list[i].attr);
    }

    printf("  . SEE Delete secure storage ...\n");
    fflush(stdout);
    if (0 != ss_delete_secure_storage("storage_test")) {
        printf("Fail\n  ! ss_delete_secure_storage\n");
        goto exit;
    }
    printf("ok\n");

exit:
	free_security_data(&output);
	if (count > 0)
		free(list);

	printf("  . See Deinitialize ...\n");
	security_deinit();
	printf("ok\n");
	fflush(stdout);

    return;
}

