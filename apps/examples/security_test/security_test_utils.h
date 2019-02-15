#ifndef _SECURITY_TEST_UTILS_H__
#define _SECURITY_TEST_UTILS_H__

#include <stdint.h>
#include <security/security_api.h>

void free_security_data(security_data *data);
void PrintBuffer(const char *header, unsigned char* buffer, uint32_t len);

#endif // _SECURITY_TEST_UTILS_H__
