###
bin_PROGRAMS = pfsglview

man_MANS = pfsglview.1

EXTRA_DIST = $(man_MANS)

pfsglview_SOURCES = pfsglview.cpp histogram.cpp histogram.h picture_io.cpp picture_io.h rmglwin.cpp rmglwin.h winstat.cpp winstat.h glenv.h

LIBS += ../pfs/libpfs-1.2.la $(GL_LIBS)
INCLUDES = -I${srcdir}/../pfs $(GL_CFLAGS)

