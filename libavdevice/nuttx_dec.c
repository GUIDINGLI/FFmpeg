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
 * NUTTX input and output: input
 *
 * This avdevice encoder can play audio to an NUTTX device.
 *
 * The playback period is set to the lower value available for the device,
 * which gives a low latency suitable for real-time playback.
 */

#include "libavutil/internal.h"
#include "libavutil/mathematics.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"

#include "libavformat/internal.h"

#include "avdevice.h"
#include "nuttx.h"

static av_cold int nuttx_read_header(AVFormatContext *s1)
{
    NuttxPriv *priv = s1->priv_data;
    enum AVCodecID codec_id;
    AVStream *st;
    int ret;

    st = avformat_new_stream(s1, NULL);
    if (!st) {
        av_log(s1, AV_LOG_ERROR, "Cannot add stream\n");
        return AVERROR(ENOMEM);
    }

    codec_id = s1->audio_codec_id;
    if (!ff_nuttx_check_support(codec_id))
        codec_id = AV_CODEC_ID_PCM_S16LE;

    if (!priv->sample_rate)
        priv->sample_rate = 48000;

    if (!priv->channels)
        priv->channels = 1;

    priv->bps = av_get_bits_per_sample(codec_id);
    priv->frame_size = priv->bps / 8 * priv->channels;
    priv->playback   = false;

    ret = ff_nuttx_open(s1->priv_data, s1->url);
    if (ret < 0)
        return ret;

    /* take real parameters */
    st->codecpar->codec_type  = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id    = codec_id;
    st->codecpar->sample_rate = priv->sample_rate;
    st->codecpar->channels    = priv->channels;
    st->codecpar->frame_size  = priv->frame_size;
    avpriv_set_pts_info(st, 64, 1, 1000000);  /* 64 bits pts in us */

    /* microseconds instead of seconds, MHz instead of Hz */
    priv->timefilter = ff_timefilter_new(1000000.0 / priv->sample_rate,
                                priv->period_bytes / priv->frame_size,
                                1.5E-6);
    if (!priv->timefilter)
        goto fail;

    return 0;

fail:
    ff_nuttx_close(priv);
    return AVERROR(EIO);
}

static int nuttx_read_packet(AVFormatContext *s1, AVPacket *pkt)
{
    NuttxPriv *priv  = s1->priv_data;
    int64_t dts;
    int size;
    int ret;

    if (av_new_packet(pkt, priv->period_bytes) < 0)
        return AVERROR(EIO);

    size = priv->period_bytes;
    ret = ff_nuttx_read_period(priv, pkt->data, &size);
    if (ret) {
        av_log(s1, AV_LOG_ERROR, "%s, error ret %d\n", __func__, ret);
        return ret;
    }

    dts = av_gettime();
    pkt->pts = ff_timefilter_update(priv->timefilter, dts, priv->last_period);
    priv->last_period = size / priv->frame_size;

    pkt->size = size;

    return 0;
}

static int nuttx_read_close(AVFormatContext *s1)
{
    return ff_nuttx_close(s1->priv_data);
}

static const AVOption options[] = {
    { "periods",      "", offsetof(NuttxPriv, periods),      AV_OPT_TYPE_INT, {.i64 = 4},     1, INT_MAX, AV_OPT_FLAG_DECODING_PARAM },
    { "period_bytes", "", offsetof(NuttxPriv, period_bytes), AV_OPT_TYPE_INT, {.i64 = 8192},  1, INT_MAX, AV_OPT_FLAG_DECODING_PARAM },
    { "sample_rate",  "", offsetof(NuttxPriv, sample_rate),  AV_OPT_TYPE_INT, {.i64 = 48000}, 1, INT_MAX, AV_OPT_FLAG_DECODING_PARAM },
    { "channels",     "", offsetof(NuttxPriv, channels),     AV_OPT_TYPE_INT, {.i64 = 2},     1, INT_MAX, AV_OPT_FLAG_DECODING_PARAM },
    { NULL },
};

static const AVClass nuttx_demuxer_class = {
    .class_name     = "NUTTX indev",
    .item_name      = av_default_item_name,
    .option         = options,
    .version        = LIBAVUTIL_VERSION_INT,
    .category       = AV_CLASS_CATEGORY_DEVICE_AUDIO_INPUT,
};

AVInputFormat ff_nuttx_demuxer = {
    .name           = "nuttx",
    .long_name      = NULL_IF_CONFIG_SMALL("NUTTX audio input"),
    .priv_data_size = sizeof(NuttxPriv),
    .read_header    = nuttx_read_header,
    .read_packet    = nuttx_read_packet,
    .read_close     = nuttx_read_close,
    .flags          = AVFMT_NOFILE,
    .priv_class     = &nuttx_demuxer_class,
};
