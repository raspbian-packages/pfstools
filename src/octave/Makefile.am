octdir = $(OCTAVE_OCT_DIR)
mdir = $(OCTAVE_M_DIR)

linked_oct_files = pfsread.oct pfswrite.oct pfstransform_colorspace.oct \
		pfsopen.oct pfsget.oct pfsput.oct pfsclose.oct

h_files = octave_util.h
m_oct_files = pfsview.m pfsview_rgb.m pfsview_list.m \
              pfs_read_rgb.m pfs_read_xyz.m pfs_read_luminance.m \
              pfs_write_rgb.m pfssize.m pfs_open_frames.m pfs_close_frames.m

help_files = pfsopen.help pfsclose.help pfsget.help pfsput.help \
             pfsread.help pfswrite.help pfstransform_colorspace.help

help_h_files = $(help_files:.help=_help.h) 

oct_SCRIPTS = $(linked_oct_files) 
m_SCRIPTS = $(m_oct_files)

bin_SCRIPTS = pfsoctavelum pfsoctavergb pfsstat
man_MANS = pfsoctavelum.1 pfsoctavergb.1 pfsstat.1

CCFILES = \
  $(linked_oct_files:.oct=.cpp) 

EXTRA_DIST = $(m_oct_files) $(CCFILES) $(h_files) \
	$(help_files) $(help_h_files) $(man_MANS) $(bin_SCRIPTS)

%.oct: %.cpp %_help.h
	echo $(find %.help.h, $(help_files))
	$(MKOCTFILE) -v -s -I. -I$(top_srcdir)/src/pfs/ $(LIBS) -L../pfs/ -lpfs $< -o $@


%_help.h: %.help
	awk ' { print "\t\"" $$0 "\\n\""; } END { print ";" } ' <$< >$@

clean:
	rm -f *.oct *.o *core

.PHONY: clean
