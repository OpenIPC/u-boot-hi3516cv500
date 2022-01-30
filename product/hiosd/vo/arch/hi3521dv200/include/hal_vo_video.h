/*
 * Copyright (C) Hisilicon Technologies Co., Ltd. 2012-2019. All rights reserved.
 * Description: hal layer header file.
 * Author: Hisilicon multimedia software group
 * Create: 2012/06/28
 */

#ifndef __HAL_VO_VIDEO_H__
#define __HAL_VO_VIDEO_H__

#include "hal_vo_video_comm.h"
hi_void vo_hal_layer_set_cvfir_cfg(hal_disp_layer layer, hi_u32 vratio, vdp_v1_cvfir_cfg *cfg,
    vo_zme_comm_pq_cfg *pq_cfg);
hi_bool hal_cbm_get_cbm1_mixer_layer_id(hi_vo_layer layer, hi_u8 *layer_id);
hi_bool hal_cbm_get_cbm2_mixer_layer_id(hi_vo_layer layer, hi_u8 *layer_id);
hi_bool hal_cbm_get_cbm3_mixer_layer_id(hi_vo_layer layer, hi_u8 *layer_id);
#endif /* end of __HAL_VO_VIDEO_H__ */
