###
# 


if PPM_SUPPORT
  PPM_PROGS = pfsinppm pfsoutppm
endif

if TIFF_SUPPORT
  TIFF_PROGS = pfsintiff pfsouttiff
endif

if JPEGHDR_SUPPORT
  JPEGHDR_PROGS = pfsinjpeghdr pfsoutjpeghdr
endif

if OPENEXR_SUPPORT
  OPENEXR_PROGS = pfsinexr pfsoutexr
endif

if IMAGEMAGICK_SUPPORT
  IMAGEMAGICK_PROGS = pfsinimgmagick pfsoutimgmagick
endif

if GDAL_SUPPORT
  GDAL_PROGS = pfsingdal
endif

bin_PROGRAMS = pfsinrgbe pfsoutrgbe \
               pfsinpfm pfsoutpfm \
               $(PPM_PROGS) \
               $(TIFF_PROGS) \
	       $(JPEGHDR_PROGS) \
               $(OPENEXR_PROGS) \
               $(IMAGEMAGICK_PROGS) \
	       $(GDAL_PROGS)

bin_SCRIPTS  = pfsin pfsout pfsoutffmpeg pfsinmulti pfsindcraw

man_MANS = pfsin.1 pfsout.1 pfsinppm.1 pfsinexr.1 pfsinrgbe.1 \
           pfsintiff.1 pfsoutppm.1 pfsoutexr.1 pfsoutffmpeg.1 \
           pfsinpfm.1 pfsoutpfm.1 pfsinmulti.1 pfsinimgmagick.1 \
           pfsoutimgmagick.1 pfsinjpeghdr.1 pfsoutjpeghdr.1 \
	   pfsindcraw.1 pfsingdal.1


EXTRA_DIST = $(man_MANS) $(bin_SCRIPTS)

LIBS += ../pfs/libpfs-1.2.la
INCLUDES = -I${srcdir}/../pfs

#Commented out because it does not work with conditional build
#noinst_LIBRARIES = libpfsfileformat.a
#libpfsfileformat_a_SOURCES = rgbeio.cpp ppmio.cpp hdrtiffio.cpp 

if PPM_SUPPORT
pfsinppm_SOURCES = pfsinppm.cpp ppmio.cpp ppmio.h
pfsoutppm_SOURCES = pfsoutppm.cpp ppmio.cpp ppmio.h
pfsinppm_LDADD = $(NETPBM_LIBS)
pfsoutppm_LDADD = $(NETPBM_LIBS)
INCLUDES += $(NETPBM_CFLAGS)
endif

if TIFF_SUPPORT
pfsintiff_SOURCES = pfsintiff.cpp hdrtiffio.cpp hdrtiffio.h
pfsouttiff_SOURCES = pfsouttiff.cpp hdrtiffio.cpp hdrtiffio.h
pfsintiff_LDADD = $(TIFF_LIBS)
pfsouttiff_LDADD = $(TIFF_LIBS)
endif

if JPEGHDR_SUPPORT
pfsinjpeghdr_SOURCES = pfsinjpeghdr.cpp 
pfsoutjpeghdr_SOURCES = pfsoutjpeghdr.cpp 
pfsinjpeghdr_LDADD = $(JPEGHDR_LIBS)
pfsoutjpeghdr_LDADD = $(JPEGHDR_LIBS)
endif

#pfsreadseq_SOURCES = pfsreadseq.cpp
#pfsreadseq_LDADD = ../pfs/libpfs-1.0.la

if OPENEXR_SUPPORT
pfsinexr_SOURCES = pfsinexr.cpp exrio.cpp exrio.h
pfsoutexr_SOURCES = pfsoutexr.cpp exrio.cpp exrio.h
pfsoutexr_LDADD = $(OPENEXR_LIBS)
pfsinexr_LDADD = $(OPENEXR_LIBS)
INCLUDES += $(OPENEXR_CFLAGS)
endif

if IMAGEMAGICK_SUPPORT
pfsinimgmagick_SOURCES = pfsinimgmagick.cpp
pfsinimgmagick_LDADD = $(IMAGEMAGICK_LIBS)
pfsoutimgmagick_SOURCES = pfsoutimgmagick.cpp
pfsoutimgmagick_LDADD = $(IMAGEMAGICK_LIBS)
INCLUDES += $(IMAGEMAGICK_CFLAGS)
endif

if GDAL_SUPPORT
pfsingdal_SOURCES = pfsingdal.cpp
pfsingdal_LDADD = $(GDAL_LIBS)
endif

pfsinrgbe_SOURCES = pfsinrgbe.cpp rgbeio.cpp rgbeio.h
pfsoutrgbe_SOURCES = pfsoutrgbe.cpp rgbeio.cpp rgbeio.h

pfsinpfm_SOURCES = pfsinpfm.cpp
pfsoutpfm_SOURCES = pfsoutpfm.cpp

install-data-hook:
	cd $(DESTDIR)$(man1dir); \
	ln -f -s pfsoutppm.1 pfsouttiff.1; \
	ln -f -s pfsoutppm.1 pfsoutrgbe.1;

uninstall-hook:
	cd $(DESTDIR)$(man1dir); \
	rm pfsouttiff.1 pfsoutrgbe.1
