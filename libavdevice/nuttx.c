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
 * NUTTX input and output: common code
 */

#include <libavcodec/codec_id.h>
#include <libavutil/samplefmt.h>

#include <fcntl.h>

#include "nuttx.h"

#define MAX_RETRY       500
#define MAX_NSAMPLERATE 16

static int ff_nuttx_samplerate_convert(NuttxPriv *priv, uint32_t samplerate)
{
    int i = 0;

    if (samplerate <= 0 || samplerate >= (AUDIO_SAMP_RATE_192K << 1))
        return -EINVAL;

    priv->sample_rates_size = MAX_NSAMPLERATE * sizeof(int);
    priv->sample_rates      = av_mallocz(priv->sample_rates_size);
    if (!priv->sample_rates)
        return -ENOMEM;

    while (samplerate) {
        if (i >= MAX_NSAMPLERATE)
            break;

        if (samplerate & AUDIO_SAMP_RATE_8K) {
            samplerate &= (uint32_t)~AUDIO_SAMP_RATE_8K;
            priv->sample_rates[i++] = 8000;
        } else if (samplerate & AUDIO_SAMP_RATE_11K) {
            samplerate &= ~AUDIO_SAMP_RATE_11K;
            priv->sample_rates[i++] = 11025;
        } else if (samplerate & AUDIO_SAMP_RATE_16K) {
            samplerate &= ~AUDIO_SAMP_RATE_16K;
            priv->sample_rates[i++] = 16000;
        } else if (samplerate & AUDIO_SAMP_RATE_22K) {
            samplerate &= ~AUDIO_SAMP_RATE_22K;
            priv->sample_rates[i++] = 22050;
        } else if (samplerate & AUDIO_SAMP_RATE_32K) {
            samplerate &= ~AUDIO_SAMP_RATE_32K;
            priv->sample_rates[i++] = 32000;
        } else if (samplerate & AUDIO_SAMP_RATE_44K) {
            samplerate &= ~AUDIO_SAMP_RATE_44K;
            priv->sample_rates[i++] = 44100;
        } else if (samplerate & AUDIO_SAMP_RATE_48K) {
            samplerate &= ~AUDIO_SAMP_RATE_48K;
            priv->sample_rates[i++] = 48000;
        } else if (samplerate & AUDIO_SAMP_RATE_96K) {
            samplerate &= ~AUDIO_SAMP_RATE_96K;
            priv->sample_rates[i++] = 96000;
        } else if (samplerate & AUDIO_SAMP_RATE_128K) {
            samplerate &= ~AUDIO_SAMP_RATE_128K;
            priv->sample_rates[i++] = 128000;
        } else if (samplerate & AUDIO_SAMP_RATE_160K) {
            samplerate &= ~AUDIO_SAMP_RATE_160K;
            priv->sample_rates[i++] = 160000;
        } else if (samplerate & AUDIO_SAMP_RATE_172K) {
            samplerate &= ~AUDIO_SAMP_RATE_172K;
            priv->sample_rates[i++] = 172000;
        } else if (samplerate & AUDIO_SAMP_RATE_192K) {
            samplerate &= ~AUDIO_SAMP_RATE_192K;
            priv->sample_rates[i++] = 192000;
        }
    }

    return 0;
}

av_cold int ff_nuttx_get_capabilities(NuttxPriv *priv, const char *device,
                                      bool playback)
{
    struct audio_caps_s caps;
    int channel, samplerate;
    int ret;
    int fd;

    fd = open(device, O_RDWR | O_CLOEXEC);
    if (fd < 0)
        return AVERROR(ENOSYS);

    caps.ac_len = sizeof(caps);
    caps.ac_type = playback ? AUDIO_TYPE_OUTPUT : AUDIO_TYPE_INPUT;
    caps.ac_subtype = AUDIO_TYPE_QUERY;

    ret = ioctl(fd, AUDIOIOC_GETCAPS, (unsigned long)&caps);
    if (ret != caps.ac_len)
        goto out;

    channel    = caps.ac_channels;
    samplerate = caps.ac_controls.b[0];

    priv->sample_fmts_size     = 2 * sizeof(int);
    priv->sample_fmts          = av_mallocz(priv->sample_fmts_size);
    priv->channel_layouts_size = 2 * sizeof(uint64_t);
    priv->channel_layouts      = av_mallocz(priv->channel_layouts_size);

    if (!priv->sample_fmts || !priv->channel_layouts) {
        ret = -ENOMEM;
        goto out;
    }

    priv->sample_fmts[0]     = AV_SAMPLE_FMT_S16;
    priv->sample_fmts[1]     = AV_SAMPLE_FMT_NONE;
    priv->channel_layouts[0] = av_get_default_channel_layout(channel);
    priv->channel_layouts[1] = 0;

    ret = ff_nuttx_samplerate_convert(priv, samplerate);
out:
    close(fd);
    if (ret < 0) {
        if (priv->sample_fmts) av_free(priv->sample_fmts);
        if (priv->sample_rates) av_free(priv->sample_rates);
        if (priv->channel_layouts) av_free(priv->channel_layouts);
    }
    return ret;
}

av_cold bool ff_nuttx_check_support(enum AVCodecID codec_id)
{
    switch (codec_id)
    {
        case AV_CODEC_ID_PCM_S8:
        case AV_CODEC_ID_PCM_S16LE:
        case AV_CODEC_ID_PCM_S24LE:
        case AV_CODEC_ID_PCM_S32LE:
            return true;
        default:
            return false;
    }
}

av_cold int ff_nuttx_open(NuttxPriv *priv, const char *device)
{
    struct audio_caps_desc_s caps_desc = {0};
    struct audio_buf_desc_s buf_desc;
    struct ap_buffer_info_s buf_info;
    int ret;
    int x;

    struct mq_attr attr = {
        .mq_maxmsg  = 8,
        .mq_msgsize = sizeof(struct audio_msg_s),
    };

    /* open device */
    priv->fd = open(device, O_RDWR | O_CLOEXEC);
    if (priv->fd < 0) {
        return AVERROR(ENOSYS);
    }

    /* configure */
    ret = ioctl(priv->fd, AUDIOIOC_RESERVE, 0);
    if (ret < 0) {
        goto out;
    }

    caps_desc.caps.ac_len            = sizeof(struct audio_caps_s);
    caps_desc.caps.ac_type           = priv->playback ?
                                       AUDIO_TYPE_OUTPUT : AUDIO_TYPE_INPUT;
    caps_desc.caps.ac_channels       = priv->channels;
    caps_desc.caps.ac_controls.hw[0] = priv->sample_rate;
    caps_desc.caps.ac_controls.b[3]  = priv->sample_rate >> 16;
    caps_desc.caps.ac_controls.b[2]  = priv->bps;
    ret = ioctl(priv->fd, AUDIOIOC_CONFIGURE, (unsigned long)&caps_desc);
    if (ret < 0) {
        goto out;
    }

    /* create message queue */
    snprintf(priv->mqname, sizeof(priv->mqname), "/tmp/%0lx",
            (unsigned long)((uintptr_t)priv));
    priv->mq = mq_open(priv->mqname, O_RDWR | O_CREAT, 0644, &attr);
    if (priv->mq < 0) {
        goto out;
    }

    ret = ioctl(priv->fd, AUDIOIOC_REGISTERMQ, (unsigned long)priv->mq);
    if (ret < 0) {
        goto out;
    }

    /* create audio buffers */
    if (priv->periods && priv->period_bytes) {
        /* try to set BUFINFO and don't care the returns */
        buf_info.nbuffers    = priv->periods;
        buf_info.buffer_size = priv->period_bytes;
        ioctl(priv->fd, AUDIOIOC_SETBUFFERINFO, (unsigned long)&buf_info);
    }

    ret = ioctl(priv->fd, AUDIOIOC_GETBUFFERINFO, (unsigned long)&buf_info);
    if (!ret) {
        priv->periods      = buf_info.nbuffers;
        priv->period_bytes = buf_info.buffer_size;
    } else {
        priv->periods      = CONFIG_AUDIO_NUM_BUFFERS;
        priv->period_bytes = CONFIG_AUDIO_BUFFER_NUMBYTES;
    }

    priv->buffers = (NuttxBuffer *)malloc(priv->periods * sizeof(NuttxBuffer));
    if (!priv->buffers) {
        ret = -ENOMEM;
        goto out;
    }

    for (x = 0; x < priv->periods; x++) {
        priv->buffers[x].abuffer = NULL;
        priv->buffers[x].inqueue = false;
    }

    for (x = 0; x < priv->periods; x++) {
        buf_desc.numbytes  = priv->period_bytes;
        buf_desc.u.pbuffer = &priv->buffers[x].abuffer;
        ret = ioctl(priv->fd, AUDIOIOC_ALLOCBUFFER, (unsigned long)&buf_desc);
        if (ret != sizeof(buf_desc)) {
            goto out;
        }
    }

    /* create local buffer */
    priv->buffer = malloc(priv->period_bytes);
    if (!priv->buffer) {
        ret = -ENOMEM;
        goto out;
    }

    /* enqueuebuffer if capture */
    if (!priv->playback) {
        for (x = 0; x < priv->periods; x++) {
            buf_desc.u.buffer = priv->buffers[x].abuffer;
            ret = ioctl(priv->fd, AUDIOIOC_ENQUEUEBUFFER, (unsigned long)&buf_desc);
            if (ret < 0) {
                goto out;
            }

            priv->buffers[x].inqueue = true;
        }
    }

    /* start audio */
    ret = ioctl(priv->fd, AUDIOIOC_START, 0);

out:
    if (ret < 0)
        ff_nuttx_close(priv);
    return ret;
}

av_cold int ff_nuttx_close(NuttxPriv *priv)
{
    struct audio_msg_s msg;
    int i;

    if (priv->fd < 0)
        return 0;

    while (1) {
        struct mq_attr stat;
        int ret;

        ret = mq_getattr(priv->mq, &stat);
        if (ret < 0)
            return ret;

        if (!stat.mq_curmsgs)
            break;

        mq_receive(priv->mq, (char *)&msg, sizeof(msg), NULL);
    }

    ioctl(priv->fd, AUDIOIOC_STOP, 0);

    while (1) {
        mq_receive(priv->mq, (char *)&msg, sizeof(msg), NULL);
        if (msg.msg_id == AUDIO_MSG_COMPLETE) {
            break;
        }
    }

    if (priv->mq) {

        ioctl(priv->fd, AUDIOIOC_UNREGISTERMQ, NULL);

        mq_close(priv->mq);
        priv->mq = NULL;

        mq_unlink(priv->mqname);
    }

    ioctl(priv->fd, AUDIOIOC_RELEASE, 0);

    if (priv->buffers) {
        for (i = 0; i < priv->periods; i++) {
            struct audio_buf_desc_s buf_desc;

            buf_desc.u.buffer = priv->buffers[i].abuffer;
            ioctl(priv->fd, AUDIOIOC_FREEBUFFER, (unsigned long)&buf_desc);
        }

        free(priv->buffers);
        priv->buffers = NULL;
    }

    if (priv->buffer) {
        free(priv->buffer);
        priv->buffer = NULL;
        priv->buffer_pos = 0;
    }

    close(priv->fd);
    priv->fd = -1;

    return 0;
}

int ff_nuttx_get_buffer(NuttxPriv *priv, struct ap_buffer_s **abuffer)
{
    struct audio_msg_s msg;
    struct mq_attr stat;
    ssize_t size;
    int i, ret;

    *abuffer = NULL;

    for (i = 0; i < priv->periods; i++) {
        if (!priv->buffers[i].inqueue) {
            priv->buffers[i].inqueue = true;
            *abuffer = priv->buffers[i].abuffer;
            return 0;
        }
    }

    ret = mq_getattr(priv->mq, &stat);
    if (ret < 0)
        return ret;

    if (!stat.mq_curmsgs)
        return -ENOBUFS;

    size = mq_receive(priv->mq, (char *)&msg, sizeof(msg), NULL);
    if (size != sizeof(msg))
        return -EIO;

    if (msg.msg_id == AUDIO_MSG_DEQUEUE) {
        *abuffer = msg.u.ptr;
    } else if (msg.msg_id == AUDIO_MSG_COMPLETE) {
    } else {
        return -EIO;
    }

    return 0;
}

int ff_nuttx_write_period(NuttxPriv *priv, uint8_t *buf, int size)
{
    struct audio_buf_desc_s desc;
    struct ap_buffer_s *abuffer;
    int i = 0;
    int ret;

    while (1) {
        ret = ff_nuttx_get_buffer(priv, &abuffer);
        if (ret == -ENOBUFS) {
            if (i++ > MAX_RETRY)
                goto out;
            usleep(10 * 1000);
        }
        else if (ret)
            goto out;
        else
            break;
    }

    memcpy(abuffer->samp, buf, size);
    abuffer->nbytes = size;

    desc.u.buffer = abuffer;
    ret = ioctl(priv->fd, AUDIOIOC_ENQUEUEBUFFER, (unsigned long)&desc);
out:
    return ret;
}

int ff_nuttx_read_period(NuttxPriv *priv, uint8_t *buf, int *size)
{
    struct audio_buf_desc_s desc;
    struct ap_buffer_s *abuffer;
    int i = 0;
    int ret;

    if (*size < priv->period_bytes)
        return -EIO;

    while (1) {
        ret = ff_nuttx_get_buffer(priv, &abuffer);
        if (ret == -ENOBUFS) {
            if (i++ > MAX_RETRY)
                goto out;
            usleep(10 * 1000);
        }
        else if (ret)
            goto out;
        else
            break;
    }

    if (*size < abuffer->nbytes) {
        ret = -EIO;
        goto out;
    }

    memcpy(buf, abuffer->samp, abuffer->nbytes);
    *size = abuffer->nbytes;

    abuffer->nbytes = 0;
    desc.u.buffer = abuffer;
    ret = ioctl(priv->fd, AUDIOIOC_ENQUEUEBUFFER, (unsigned long)&desc);
out:
    return ret;
}
