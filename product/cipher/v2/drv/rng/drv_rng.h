/*
 * Copyright (c) Hisilicon Technologies Co., Ltd. 2017-2020. All rights reserved.
 * Description   : head file for drv rng
 * Author        : Hisilicon multimedia software group
 * Create        : 2017-10-20
 */

#ifndef __DRV_RNG_H__
#define __DRV_RNG_H__

/* add include here */
#include "hi_drv_cipher.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Macro Definition ******************************/
hi_u32 drv_cipher_rand(hi_void);
hi_s32 drv_rng_init(hi_void);
hi_void drv_rng_deinit(hi_void);

#ifdef __cplusplus
}
#endif
#endif /* __DRV_CIPHER_H__ */
