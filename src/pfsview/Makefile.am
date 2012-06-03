bin_PROGRAMS = pfsview

bin_SCRIPTS  = pfsv

man_MANS = pfsview.1 pfsv.1

EXTRA_DIST = $(bin_SCRIPTS) $(man_MANS) mac_os/Info.plist mac_os/PkgInfo mac_os/pfsview_icon_mac.icns icons/appicon.png icons/appicon.svg pfsview.qrc

pfsview_MOC = main.moc.cpp pfsview_widget.moc.cpp luminancerange_widget.moc.cpp

pfsview_SOURCES = main.cpp main.h \
	pfsview_widget.cpp pfsview_widget.h \
	luminancerange_widget.cpp luminancerange_widget.h \
	histogram.cpp histogram.h resources.cpp

nodist_pfsview_SOURCES = $(pfsview_MOC)

pfsview_LDADD = $(QT_LIBS)

BUILT_SOURCES = $(pfsview_MOC)

LIBS += ../pfs/libpfs-1.2.la
#LIBS = 

INCLUDES = $(QT_CFLAGS) -I$(top_srcdir)/src/pfs/

CLEANFILES = $(BUILT_SOURCES)

%.moc.cpp: %.h
	$(MOC) -o $@ $<

resources.cpp: pfsview.qrc
	rcc -o $@ $<

if MAC_OS
install-data-hook:
	install -d "/Applications/pfsview.app/Contents"
	install -d "/Applications/pfsview.app/Contents/MacOS"
	install -d "/Applications/pfsview.app/Contents/Resources"
	install "mac_os/PkgInfo" "/Applications/pfsview.app/Contents"
	install "mac_os/Info.plist" "/Applications/pfsview.app/Contents"
	install "mac_os/pfsview_icon_mac.icns" "/Applications/pfsview.app/Contents/Resources"
	mv $(DESTDIR)$(bindir)/pfsview /Applications/pfsview.app/Contents/MacOS/
endif
