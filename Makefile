############################################################################
# apps/external/ffmpeg/Makefile
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
#
############################################################################

-include $(APPDIR)/Make.defs

SBINDIR   := $(BINDIR)
SINCDIR   := $(INCDIR)
SCFLAGS   := $(CFLAGS)
SCXXFLAGS := $(CXXFLAGS)
SAR       := $(AR)
-include ffbuild/config.mak
BINDIR    := $(SBINDIR)
INCDIR    := $(SINCDIR)
CFLAGS    := $(SCFLAGS)
CXXFLAGS  := $(SCXXFLAGS)
AR        := $(SAR)

CSRCS     :=
ASRCS     :=
SRC_PATH  := .

CFG_ARCH  := $(CONFIG_ARCH)
ifeq ($(CONFIG_ARCH), avr)
  CFG_ARCH = avr32
endif

ifneq ($(CONFIG_ARCH),sim)
  CFG_CMDS += --arch=$(CFG_ARCH) --target-os=none
  CFG_CMDS += --enable-cross-compile --cross-prefix=$(CROSSDEV)
endif

ifneq ($(CONFIG_LIB_MBEDTLS),)
  CFG_CMDS += --enable-mbedtls
endif

FFLIBS-$(CONFIG_AVCODEC)    += avcodec
FFLIBS-$(CONFIG_AVDEVICE)   += avdevice
FFLIBS-$(CONFIG_AVFILTER)   += avfilter
FFLIBS-$(CONFIG_AVFORMAT)   += avformat
FFLIBS-$(CONFIG_AVRESAMPLE) += avresample
FFLIBS-yes                  += avutil
FFLIBS-$(CONFIG_POSTPROC)   += postproc
FFLIBS-$(CONFIG_SWRESAMPLE) += swresample
FFLIBS-$(CONFIG_SWSCALE)    += swscale

SUBDIR_VARS := CLEANFILES FFLIBS HOSTPROGS TESTPROGS TOOLS               \
               HEADERS ARCH_HEADERS BUILT_HEADERS SKIPHEADERS            \
               ARMV5TE-OBJS ARMV6-OBJS ARMV8-OBJS VFP-OBJS NEON-OBJS     \
               ALTIVEC-OBJS VSX-OBJS MMX-OBJS X86ASM-OBJS                \
               MIPSFPU-OBJS MIPSDSPR2-OBJS MIPSDSP-OBJS MSA-OBJS         \
               MMI-OBJS OBJS SLIBOBJS HOSTOBJS TESTOBJS

define DOSUBDIR
  $(foreach V,$(SUBDIR_VARS), $(V) := $(V)-yes :=)
  include $(1)/Makefile
  -include $(1)/$$(ARCH)/Makefile
  include ffbuild/arch.mak
  SOBJS := $$(sort $$(addprefix $(1)/, $$(OBJS) $$(OBJS-yes)))
  CSRCS += $$(wildcard $$(patsubst %.o,%.c, $$(SOBJS)))
  ASRCS += $$(wildcard $$(patsubst %.o,%.S, $$(SOBJS)))
endef

$(foreach D,$(FFLIBS-yes),$(eval $(call DOSUBDIR,lib$(D))))

PRIORITY  = $(CONFIG_LIB_FFMPEG_TOOLS_PRIORITY)
STACKSIZE = $(CONFIG_LIB_FFMPEG_TOOLS_STACKSIZE)
MODULE    = $(CONFIG_LIB_FFMPEG)
CSRCS    += fftools/cmdutils.c

ifeq ($(CONFIG_FFMPEG), yes)
  PROGNAME += ffmpeg
  MAINSRC  += fftools/ffmpeg.c
  CSRCS    += fftools/ffmpeg_opt.c fftools/ffmpeg_filter.c fftools/ffmpeg_hw.c
endif

ifeq ($(CONFIG_FFPROBE), yes)
  PROGNAME += ffprobe
  MAINSRC  += fftools/ffprobe.c
endif

CFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(CURDIR)}
CFLAGS += -w -Os -DHAVE_AV_CONFIG_H

libavutil/ffversion.h .version:
	$(Q)ffbuild/version.sh . libavutil/ffversion.h $(EXTRA_VERSION)

config.h:
	$(Q) echo "FFMPEG configure... $(CONFIG_ARCH)"
	$(Q)./configure             --disable-all                   \
		--disable-everything    --disable-autodetect            \
		--disable-amf           --disable-audiotoolbox          \
		--disable-cuda-llvm     --disable-cuvid                 \
		--disable-d3d11va       --disable-doc                   \
		--disable-dwt           --disable-dxva2                 \
		--disable-dxva2         --disable-error-resilience      \
		--disable-faan          --disable-ffnvcodec             \
		--disable-htmlpages     --disable-iconv                 \
		--disable-lsp           --disable-lzo                   \
		--disable-manpages      --disable-pic                   \
		--disable-pixelutils    --disable-podpages              \
		--disable-nvdec         --disable-nvenc                 \
		--disable-txtpages      --disable-v4l2-m2m              \
		--disable-vaapi         --disable-vdpau                 \
		--disable-videotoolbox  --disable-x86asm                \
		--disable-xop                                           \
		--enable-version3       --enable-small                  \
		--extra-cflags="$(CFLAGS)" --extra-ldflags="$(CFLAGS)"  \
		--ld=echo               $(CFG_CMDS)                     \
		"$(CONFIG_LIB_FFMPEG_CONFIGURATION)"

context:: config.h libavutil/ffversion.h

distclean::
	rm -f .version config.h ffbuild/.config ffbuild/config*
	rm -f libavutil/avconfig.h libavutil/ffversion.h
	find ./ -name "*_list.c" | xargs rm -f

include $(APPDIR)/Application.mk
