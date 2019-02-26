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
#include <stdio.h>
#include <pthread.h>

#define SECURITY_LOG printf

#ifdef LINUX
#define getpid pthread_self
#define SECURITY_ENTRY SECURITY_LOG("T%lx [SM]--> %s\t%s:%d\n", getpid(), __FUNCTION__, __FILE__, __LINE__)
#define SECURITY_OUT SECURITY_LOG("T%lx [SM]<-- %s\t%s:%d\n", getpid(), __FUNCTION__, __FILE__, __LINE__)
#define SECURITY_ERR SECURITY_LOG("T%lx [SM]<-- %s\t%s:%d\n", getpid(), __FUNCTION__, __FILE__, __LINE__)
#else
#define SECURITY_ENTRY SECURITY_LOG("T%lu [SM]--> %s\t%s:%d\n", getpid(), __FUNCTION__, __FILE__, __LINE__)
#define SECURITY_OUT SECURITY_LOG("T%lu [SM]<-- %s\t%s:%d\n", getpid(), __FUNCTION__, __FILE__, __LINE__)
#define SECURITY_ERR SECURITY_LOG("T%lu [SM]<-- %s\t%s:%d\n", getpid(), __FUNCTION__, __FILE__, __LINE__)
#endif

/*  SECURITY_LOCK protects   */
#define SECURITY_RETURN(res)						\
	do {										\
		if (res != SECURITY_OK) {					\
			SECURITY_ERR;							\
		} else {								\
			SECURITY_OUT;							\
		}										\
		return res;								\
	} while (0)

#define SECURITY_CALL(func, res, ret)				\
	do {										\
		if (func != res) {						\
			SECURITY_RETURN(ret);					\
		}										\
	} while (0)

