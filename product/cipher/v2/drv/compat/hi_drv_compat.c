/*
 * Copyright (c) Hisilicon Technologies Co., Ltd. 2017-2020. All rights reserved.
 * Description   : drivers for cipher compat.
 * Author        : Hisilicon multimedia software group
 * Create        : 2017-10-20
 */

#include "cipher_adapt.h"

hi_s32 hi_drv_compat_init(void)
{
    hi_s32 ret;

    ret = drv_klad_init();
    if (ret != HI_SUCCESS) {
         return ret;
    }

    return HI_SUCCESS;
}

hi_s32 hi_drv_compat_deinit(void)
{
    drv_klad_deinit();

    return HI_SUCCESS;
}

