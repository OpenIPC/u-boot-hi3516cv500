/*
 * Copyright (c) Hisilicon Technologies Co., Ltd. 2017-2020. All rights reserved.
 * Description   : mpi rng api.
 * Author        : Hisilicon multimedia software group
 * Create        : 2017-10-20
 */

#include "cipher_osal.h"

hi_s32 mpi_cipher_get_random_number(hi_u32 *random_number, hi_u32 time_out_us)
{
    hi_s32 ret;
    cipher_rng_s ci_rng;

    inlet_var_is_null_return(random_number);

    ci_rng.ci_rng = 0;
    ci_rng.time_out_us = time_out_us;

    ret = cipher_ioctl(g_cipher_dev_fd, CMD_CIPHER_GETRANDOMNUMBER, &ci_rng);

    if (ret == HI_SUCCESS)
        *random_number = ci_rng.ci_rng;

    return ret;
}

hi_s32 hi_mpi_cipher_get_random_number(hi_u32 *random_number)
{
    check_cipher_not_open_return();

    return mpi_cipher_get_random_number(random_number, 0);
}

