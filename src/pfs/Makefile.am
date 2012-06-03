###

lib_LTLIBRARIES = libpfs-1.2.la
noinst_LIBRARIES = libpfs.a

# removes automake conflict
libpfs_a_CXXFLAGS = $(AM_CXXFLAGS) -fPIC
libpfs_a_SOURCES = pfs.cpp pfsutils.cpp colorspace.cpp

libpfs_1_2_la_SOURCES = pfs.cpp pfsutils.cpp colorspace.cpp
library_includedir=$(includedir)/pfs-1.2
library_include_HEADERS = pfs.h array2d.h
#INCLUDES = 
pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = pfs.pc

