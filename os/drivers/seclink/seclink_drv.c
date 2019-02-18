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

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <tinyara/config.h>

#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <tinyara/fs/fs.h>
#include <tinyara/testcase_drv.h>
#include <tinyara/sched.h>
#include <tinyara/seclink.h>
#include <tinyara/seclink_drv.h>
#include "seclink_drv_req.h"

#define SECLINK_PATH "/dev/seclink"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int seclink_open(FAR struct file *filep);
static int seclink_close(FAR struct file *filep);
static ssize_t seclink_read(FAR struct file *filep, FAR char *buffer, size_t len);
static ssize_t seclink_write(FAR struct file *filep, FAR const char *buffer, size_t len);
static int seclink_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_seclink_fops = {
	seclink_open,                                                   /* open */
	seclink_close,                                                   /* close */
	seclink_read,
	seclink_write,
	0,                                                   /* seek */
	seclink_ioctl                                        /* ioctl */
#ifndef CONFIG_DISABLE_POLL
	, 0                                                  /* poll */
#endif
};

int seclink_open(FAR struct file *filep)
{
	printf("-->%s\n", __FUNCTION__);
	return 0;
}

int seclink_close(FAR struct file *filep)
{
	printf("-->%s\n", __FUNCTION__);
	return 0;
}

ssize_t seclink_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
	printf("-->%s\n", __FUNCTION__);
	return 0;
}
ssize_t seclink_write(FAR struct file *filep, FAR const char *buffer, size_t len)
{
	printf("-->%s\n", __FUNCTION__);
	return 0;
}

int seclink_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
	printf("-->%s (%d)(%x)\n", __FUNCTION__, cmd, arg);

	if (cmd == SECLINK_HAL_INIT || cmd == SECLINK_HAL_DEINIT) {
		hd_handle_common_request(cmd, arg);
	} else if (cmd & SECLINK_AUTH) {
		hd_handle_auth_reqeust(cmd, arg);
	} else if (cmd & SECLINK_KEY) {
		hd_handle_key_request(cmd, arg);
	} else if (cmd & SECLINK_SS) {
		hd_handle_ss_request(cmd, arg);
	} else if (cmd & SECLINK_CRYPTO) {
		hd_handle_crypto_request(cmd, arg);
	}

	return 0;
}

int seclink_register(void *priv)
{
	vdbg("Registering %s\n", SECLINK_PATH);

	return register_driver(SECLINK_PATH, &g_seclink_fops, 0666, priv);
}
