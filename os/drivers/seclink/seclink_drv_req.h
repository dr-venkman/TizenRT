#ifndef __SECLINK_DRV_REQ_H__
#define __SECLINK_DRV_REQ_H__

int hd_handle_common_request(int cmd, unsigned long arg);
int hd_handle_auth_reqeust(int cmd, unsigned long arg);
int hd_handle_key_request(int cmd, unsigned long arg);
int hd_handle_ss_request(int cmd, unsigned long arg);
int hd_handle_crypto_request(int cmd, unsigned long arg);

#endif // __SECLINK_DRV_REQ_H__

