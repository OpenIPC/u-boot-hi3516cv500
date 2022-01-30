/*
 * Copyright (c) Hisilicon Technologies Co., Ltd. 2017-2020. All rights reserved.
 * Description   : head files for drv klad.
 * Author        : Hisilicon multimedia software group
 * Create        : 2017-10-20
 */

#ifndef DRV_KLAD_H
#define DRV_KLAD_H

#include "hi_types.h"
#include "common.h"
#include "drv_cipher_ioctl.h"

extern hi_void *g_efuse_otp_reg_base;

hi_s32 drv_klad_init(hi_void);
hi_void drv_klad_deinit(hi_void);

hi_s32 drv_cipher_klad_load_key(hi_u32 chn_id,
                                hi_cipher_ca_type root_key,
                                hi_cipher_klad_target klad_target,
                                hi_u8 *data_input,
                                hi_u32 key_len);

hi_s32 drv_cipher_klad_encrypt_key(hi_cipher_ca_type root_key,
                                   hi_cipher_klad_target klad_target,
                                   hi_u32 *clean_key,
                                   hi_u32 *encrypt_key);

#endif