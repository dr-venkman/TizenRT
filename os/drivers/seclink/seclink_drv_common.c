#include <tinyara/config.h>

#include <stdio.h>
#include <tinyara/seclink.h>
#include "seclink_drv_req.h"


int hd_handle_common_request(int cmd, unsigned long arg)
{
	struct seclink_req *req = (struct seclink_req *)arg;
	if (!req) {
		return -1;
	}

	printf("general request cmd(%x)\n", cmd);

	switch (cmd) {
	case SECLINK_HAL_INIT:
        req->res = hal_init();
		break;
	case SECLINK_HAL_DEINIT:
		req->res = hal_deinit();
		break;
	}

	return 0;
}
