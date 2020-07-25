/*
 * NUTTX input and output
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * NUTTX input and output: definitions and structures
 */

#ifndef AVDEVICE_NUTTX_H
#define AVDEVICE_NUTTX_H

#include <nuttx/config.h>
#include <nuttx/audio/audio.h>

#include "libavutil/log.h"
#include "libavdevice/timefilter.h"

typedef struct NuttxBuffer {
    struct ap_buffer_s *abuffer;
    bool               inqueue;
} NuttxBuffer;

typedef struct NuttxPriv {
    NuttxBuffer *buffers;

    char        mqname[16];   ///< message queue name
    mqd_t       mq;           ///< message queue
    int         fd;           ///< nuttx device fd

    int         periods;      ///< buffer pereids
    int         period_bytes; ///< preferred size for reads and writes, in bytes

    int         frame_size;   ///< bytes per sample * channels
    uint32_t    sample_rate;
    uint32_t    channels;
    uint32_t    bps;
    bool        playback;

    uint8_t     *buffer;
    int         buffer_pos;

    int64_t     timestamp;    ///< current timestamp, without latency applied.
    int         last_period;
    TimeFilter *timefilter;

    int *sample_fmts;           ///< list of accepted sample formats, terminated by AV_SAMPLE_FMT_NONE
    int sample_fmts_size;
    uint64_t *channel_layouts;  ///< list of accepted channel layouts, terminated by -1
    int channel_layouts_size;
    int *sample_rates;          ///< list of accepted sample rates, terminated by -1
    int sample_rates_size;
} NuttxPriv;

av_cold int ff_nuttx_get_capabilities(NuttxPriv *priv, const char *device,
                                      bool playback);
av_cold bool ff_nuttx_check_support(enum AVCodecID codec_id);
av_cold int ff_nuttx_open(NuttxPriv *priv, const char *device);
av_cold int ff_nuttx_close(NuttxPriv *priv);

int ff_nuttx_get_buffer(NuttxPriv *priv, struct ap_buffer_s **abuffer);
int ff_nuttx_write_period(NuttxPriv *priv, uint8_t *buf, int size);
int ff_nuttx_read_period(NuttxPriv *priv, uint8_t *buf, int *size);

#endif /* AVDEVICE_NUTTX_H */
