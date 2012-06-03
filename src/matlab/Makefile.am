mexdir = $(MEX_DIR)
mdir = $(MEX_DIR)
mex_SCRIPTS = pfsclose.@MEX_EXT@ pfsopen.@MEX_EXT@ pfspopen.@MEX_EXT@ pfsput.@MEX_EXT@ pfsget.@MEX_EXT@ pfs_transform_colorspace.@MEX_EXT@ pfspclose.@MEX_EXT@

mex_ccfiles = $(mex_SCRIPTS:.@MEX_EXT@=.cpp) mex_utils.cpp mex_utils.h \
              compatibility.h

m_SCRIPTS = pfsclose.m pfsopen.m pfspopen.m pfsput.m pfsget.m pfs_transform_colorspace.m \
            pfspclose.m pfs_shell.m pfs_read_luminance.m pfs_read_rgb.m pfs_read_xyz.m \
            pfs_read_image.m pfs_write_rgb.m pfs_write_image.m pfs_write_luminance.m \
            pfs_write_xyz.m pfsview.m pfs_test_shell.m Contents.m

EXTRA_DIST = Makefile.win32 $(mex_ccfiles) $(m_SCRIPTS)

CCFILES = $(mex_SCRIPTS:.@MEX_EXT@=.cpp) 

.cpp.@MEX_EXT@:
	$(MEX) -I${srcdir}/../pfs -D_MATLAB_VER $^

pfsclose.@MEX_EXT@: pfsclose.cpp mex_utils.cpp
pfsopen.@MEX_EXT@: pfsopen.cpp mex_utils.cpp
pfspclose.@MEX_EXT@: pfspclose.cpp mex_utils.cpp
pfspopen.@MEX_EXT@: pfspopen.cpp mex_utils.cpp
pfsput.@MEX_EXT@: pfsput.cpp mex_utils.cpp ${srcdir}/../pfs/pfs.cpp
pfsget.@MEX_EXT@: pfsget.cpp mex_utils.cpp ${srcdir}/../pfs/pfs.cpp
pfs_transform_colorspace.@MEX_EXT@: pfs_transform_colorspace.cpp mex_utils.cpp ${srcdir}/../pfs/colorspace.cpp

clean:
	rm -f *.@MEX_EXT@

.PHONY: clean

