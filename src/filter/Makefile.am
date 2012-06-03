###
bin_PROGRAMS = pfsgamma pfsclamp pfstag pfssize pfsextractchannels pfspanoramic \
	pfsrotate pfsflip pfscut pfspad pfscat pfsabsolute pfsdisplayfunction

man_MANS = pfsgamma.1 pfsclamp.1 pfstag.1 pfssize.1 pfsextractchannels.1 \
	pfspanoramic.1 pfsrotate.1 pfsflip.1 pfscut.1 pfspad.1 pfscat.1 \
	pfsabsolute.1 pfsdisplayfunction.1

EXTRA_DIST = $(man_MANS)

pfsclamp_SOURCES = pfsclamp.cpp

pfstag_SOURCES = pfstag.cpp

pfsgamma_SOURCES = pfsgamma.cpp

pfsdisplayfunction_SOURCES = pfsdisplayfunction.cpp display_function.cpp display_function.h

pfssize_SOURCES = pfssize.cpp

pfspanoramic_SOURCES = pfspanoramic.cpp

pfsextractchannels_SOURCES = pfsextractchannels.cpp

pfsrotate_SOURCES = pfsrotate.cpp

pfsflip_SOURCES = pfsflip.cpp

pfscut_SOURCES = pfscut.cpp

pfspad_SOURCES = pfspad.cpp

pfscat_SOURCES = pfscat.cpp

pfsabsolute_SOURCES = pfsabsolute.cpp

LIBS += ../pfs/libpfs-1.2.la
INCLUDES = -I${srcdir}/../pfs

