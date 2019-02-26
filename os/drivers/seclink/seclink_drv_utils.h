#ifndef _SECLINK_DRIVER_UTILS_H__
#define _SECLINK_DRIVER_UTILS_H__

#define SECLINK_PATH "/dev/seclink"

#define SLDRV_LOG printf

#define SLDRV_TAG "[SECLINK_DRV]"

#define SLDRV_ERR(fd)													\
	do {																\
		SLDRV_LOG(SL_TAG"[ERR:%s] %s %s:%d ret(%d) code(%s)\n",			\
				  SL_TAG, __FUNCTION__, __FILE__, __LINE__, fd, strerror(errno)); \
	} while(0)

/* #define SLDRV_CALL(hnd, code, param)										\ */
/* 	do {																\ */
/* 		int res = ioctl(hnd->fd, code, (unsigned long)((uintptr_t)&param) ); \ */
/* 		if(res < 0) {													\ */
/* 			SLDRV_ERR(res);												\ */
/* 			return -1;													\ */
/* 		}																\ */
/* 	} while (0) */

#define SLDRV_ENTER														\
	do {																\
		SLDRV_LOG(SLDRV_TAG"%s\t%s:%d\n", __FUNCTION__, __FILE__, __LINE__); \
	} while (0)


#endif // _SECLINK_DRIVER_UTILS_H__
