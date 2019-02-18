#include <tinyara/config.h>

#include <stdio.h>
#include <tinyara/seclink.h>
#include "seclink_drv_req.h"


int hd_handle_key_request(int cmd, unsigned long arg)
{
	struct seclink_req *req = (struct seclink_req *)arg;
	if (!req) {
		return -1;
	}

	struct seclink_key_info *info = (struct seclink_key_info *)req->req_type.key;
	if (!info) {
		return -1;
	}

	printf("secure storage request cmd(%x) (%d)\n", cmd, info->mode);

	switch (cmd) {
	case SECLINK_HAL_SETKEY:
		req->res = hal_set_key(info->mode, info->key_idx, info->key, info->prikey);
		break;
	case SECLINK_HAL_GETKEY:
		req->res = hal_get_key(info->mode, info->key_idx, info->key);
		break;
	case SECLINK_HAL_REMOVEKEY:
		req->res = hal_remove_key(info->mode, info->key_idx);
		break;
	case SECLINK_HAL_GENERATEKEY:
		req->res = hal_generate_key(info->mode, info->key_idx);
		break;
	}

	return 0;
}
