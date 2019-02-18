#include <tinyara/config.h>

#include <stdio.h>
#include <tinyara/seclink.h>
#include "seclink_drv_req.h"

int hd_handle_ss_request(int cmd, unsigned long arg)
{
	struct seclink_req *req = (struct seclink_req *)arg;
	if (!req) {
		return -1;
	}

	struct seclink_ss_info *info = req->req_type.ss;
	if (!info) {
		return -1;
	}

	switch(cmd) {
	case SECLINK_HAL_WRITESTORAGE:
		req->res = hal_write_storage(info->key_idx, info->data);
		break;
	case SECLINK_HAL_READSTORAGE:
		req->res = hal_read_storage(info->key_idx, info->data);
		break;
	case SECLINK_HAL_DELETESTORAGE:
		req->res = hal_delete_storage(info->key_idx);
		break;
	}

	return 0;
}
