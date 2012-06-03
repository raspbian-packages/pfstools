###
# 
bin_PROGRAMS = pfsouthdrhtml
pfsouthdrhtml_SOURCES = pfsouthdrhtml.cpp hdrhtml.cpp hdrhtml.h

man_MANS = pfsouthdrhtml.1

dist_pkgdata_DATA = hdrhtml_c_b2.csv hdrhtml_t_b2.csv hdrhtml_c_b3.csv hdrhtml_t_b3.csv \
                    hdrhtml_c_b4.csv hdrhtml_t_b4.csv hdrhtml_c_b5.csv hdrhtml_t_b5.csv

nobase_dist_pkgdata_DATA = hdrhtml_default_templ/hdrhtml_page_templ.html hdrhtml_default_templ/hdrhtml_image_templ.html \
      hdrhtml_hdrlabs_templ/hdrhtml_assets/hdr_viewer.css hdrhtml_hdrlabs_templ/hdrhtml_assets/*.png \
      hdrhtml_hdrlabs_templ/hdrhtml_assets/*.gif hdrhtml_hdrlabs_templ/hdrhtml_assets/*.js hdrhtml_hdrlabs_templ/*.html

EXTRA_DIST = $(man_MANS)

LIBS += ../pfs/libpfs-1.2.la $(IMAGEMAGICK_LIBS)
INCLUDES = -I${srcdir}/../pfs $(IMAGEMAGICK_CFLAGS)

