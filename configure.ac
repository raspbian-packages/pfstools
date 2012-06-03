dnl
dnl Initialization part
dnl
AC_INIT([pfstools],[1.8.5])
AM_INIT_AUTOMAKE
AC_CONFIG_HEADERS([config.h])

AC_LANG(C++)			# perform tests in C++ language

AC_PROG_CXX
LT_INIT
AC_PROG_LIBTOOL                               
AC_CONFIG_MACRO_DIR([m4])

dnl
dnl Templates for autoheader defines
dnl
AH_TEMPLATE([DEBUG],
	    [Define as 1 when compiling in debug mode ])

dnl
dnl Internal Debug
dnl
AC_ARG_ENABLE(debug,
		AS_HELP_STRING([--enable-debug],[Enable debug symbols and asserts]),
		 [if test "$enable_debug" = "yes" ; then
                     temp_CXXFLAGS=`echo $CXXFLAGS | sed "s/-O./ /; s/-g//"`
		     CXXFLAGS="-g $temp_CXXFLAGS"
		     AC_DEFINE([DEBUG])
		  fi],
                  [enable_debug="no"
                     temp_CXXFLAGS=`echo $CXXFLAGS | sed "s/-O./ /; s/-g//"`
   	             CXXFLAGS="-O3 $temp_CXXFLAGS"
                     AC_DEFINE(NDEBUG, 1, [Disable assertion checks])
                  ])


###########################################################
# system detection and configuration
                  
dnl Compile NetPBM code?
AC_ARG_ENABLE(netpbm,
		AS_HELP_STRING([--disable-netpbm],[Do not compile programs that use NETPBM (NetPBM) library]),
		 [if test "$enable_netpbm" = "no" ; then
		     do_netpbm="no"
		  else
		     if test "$enable_netpbm" = "yes" ; then
			do_netpbm="yes"
		     fi
		  fi],
		 [do_netpbm="yes"])

dnl Compile OpenEXR code?
AC_ARG_ENABLE(openexr,
		AS_HELP_STRING([--disable-openexr],[Do not compile programs that use OpenEXR library]),
		 [if test "$enable_openexr" = "no" ; then
		     do_openexr="no"
		  else
		     if test "$enable_openexr" = "yes" ; then
			do_openexr="yes"
		     fi
		  fi],
		 [do_openexr="yes"])

dnl Compile TIFF code?
AC_ARG_ENABLE(tiff,
		AS_HELP_STRING([--disable-tiff],[Do not compile programs that use TIFF library]),
		 [if test "$enable_tiff" = "no" ; then
		     do_tiff="no"
		  else
		     if test "$enable_tiff" = "yes" ; then
			do_tiff="yes"
		     fi
		  fi],
		 [do_tiff="yes"])

dnl Compile QT code?
AC_ARG_ENABLE(qt,
		AS_HELP_STRING([--disable-qt],[Do not compile programs that use QT library]),
		 [if test "$enable_qt" = "no" ; then
		     do_qt="no"
		  else
		     if test "$enable_qt" = "yes" ; then
			do_qt="yes"
		     fi
		  fi],
		 [do_qt="yes"])

dnl Compile JPEG-HDR code?
AC_ARG_ENABLE(jpeghdr,
		AS_HELP_STRING([--disable-jpeghdr],[Do not compile programs that use JPEGHDR library]),
		 [if test "$enable_jpeghdr" = "no" ; then
		     do_jpeghdr="no"
		  else
		     if test "$enable_jpeghdr" = "yes" ; then
			do_jpeghdr="yes"
		     fi
		  fi],
		 [do_jpeghdr="yes"])
                                                                                      
dnl Compile ImageMagic++ code?
AC_ARG_ENABLE(imagemagick,
		AS_HELP_STRING([--disable-imagemagick],[Do not compile programs that use ImageMagick library]),
		 [if test "$enable_imagemagick" = "no" ; then
		     do_imagemagick="no"
		  else
		     if test "$enable_imagemagick" = "yes" ; then
			do_imagemagick="yes"
		     fi
		  fi],
		 [do_imagemagick="yes"])

dnl Compile Octave code?
AC_ARG_ENABLE(octave,
		AS_HELP_STRING([--disable-octave],[Do not compile octave interface functions]),
		 [if test "$enable_octave" = "no" ; then
		     do_octave="no"
		  else
		     if test "$enable_octave" = "yes" ; then
			do_octave="yes"
		     fi
		  fi],
		 [do_octave="yes"])

dnl Compile OpenGL code?
AC_ARG_ENABLE(opengl,
		AS_HELP_STRING([--disable-opengl],[Do not compile opengl code (pfsglview)]),
		 [if test "$enable_opengl" = "no" ; then
		     do_opengl="no"
		  else
		     if test "$enable_opengl" = "yes" ; then
			do_opengl="yes"
		     fi
		  fi],
		 [do_opengl="yes"])


dnl Compile Matlab code?
AC_ARG_ENABLE(matlab,
		AS_HELP_STRING([--disable-matlab],[Do not compile matlab files]),
		 [if test "$enable_matlab" = "no" ; then
		     do_matlab="no"
		  else
		     if test "$enable_matlab" = "yes" ; then
			do_matlab="yes"
		     fi
		  fi],
		 [do_matlab="yes"])

dnl Compile GDAL code?
AC_ARG_ENABLE(gdal,
		AS_HELP_STRING([--disable-gdal],[Do not compile programs that use GDAL library]),
		 [if test "$enable_gdal" = "no" ; then
		     do_gdal="no"
		  else
		     if test "$enable_gdal" = "yes" ; then
			do_gdal="yes"
		     fi
		  fi],
		 [do_gdal="yes"])
                                                                    
                  
                  
dnl
dnl Checking for libraries
dnl

        
if test "$do_netpbm" = "yes"; then

dnl Check for libnetpbm-10 (Debian)
AC_CHECK_LIB(netpbm, pm_init,
   [AC_DEFINE( HAVE_LIBNETPPM,1,
              [Define this if you have the libnetpbm libraries])
	      NETPBM_LIBS="-lnetpbm"
              NETPBM_CFLAGS=""
              netpbm_support="yes"],[netpbm_support="no"])

if test "$netpbm_support" = "no"; then 
dnl Check for libnetpbm (sf.net)
  AC_CHECK_LIB(ppm, ppm_init,
    [AC_DEFINE( HAVE_LIBNETPPM,1,
	        [Define this if you have the libppm libraries])
	        NETPBM_LIBS="-lppm"
                NETPBM_CFLAGS=""
                netpbm_support="yes"],[netpbm_support="no"],)
fi

if test "$netpbm_support" = "yes"; then 
dnl Search for the header files
   AC_MSG_CHECKING([for NetPBM header files])
   CPATH_NO_COLON=`echo $CPATH | sed -e "s/:/ /g"`
   for DIR in ${prefix}/include /usr/local/include/ /usr/include /opt/local/include $CPATH_NO_COLON; do
       test -d "$DIR" && NETPBM_PPMH=`find $DIR -name ppm.h`
       if test -n "$NETPBM_PPMH"; then
          NETPBM_CFLAGS="-I ${NETPBM_PPMH%%ppm.h}"
          AC_MSG_RESULT([yes])
          break
       fi
   done
   if test -z "$NETPBM_CFLAGS"; then
      netpbm_support="no"
      AC_MSG_RESULT([no])
   fi      
fi   

else
  netpbm_support="no"
fi
        
dnl Check for ImageMagick++
if test "$do_imagemagick" = "yes"; then
PKG_CHECK_MODULES(IMAGEMAGICK, ImageMagick++ >= 6.0,
  [AC_DEFINE( HAVE_LIBIMAGEMAGICK, 1, [ImageMagick library used for reading/writing images.])
  imagemagick_support="yes"],[
  imagemagick_support="no"])
else
  imagemagick_support="no"
fi

if test "$do_tiff" = "yes"; then                        
AC_CHECK_LIB(tiff, TIFFOpen,
  [AC_DEFINE( HAVE_LIBTIFF,1,
	      [Define this if you have the libtiff libraries])
	      TIFF_LIBS="-ltiff"
              tiff_support="yes"],[tiff_support="no"],)
else
  tiff_support="no"
fi

if test "$do_jpeghdr" = "yes"; then                        
AC_CHECK_LIB(jpeghdr, jpeghdr_CreateDecompress,
  [AC_DEFINE( HAVE_LIBJPEGHDR,1,
	      [Define this if you have the libjpeghdr libraries])
	      JPEGHDR_LIBS="-lhdr -ljpeghdr -ljpeg -lm"
              jpeghdr_support="yes"],[jpeghdr_support="no"], -lhdr -ljpeg -lm)
else
  jpeghdr_support="no"
fi
        
AC_ARG_WITH(exrdir,
	[  --with-openexrdir       path where exr include files are installed],
	openexr_incpath=$withval)

if test "$do_openexr" = "yes"; then                        
        
#Check first using pkg-config
PKG_CHECK_MODULES(OPENEXR, OpenEXR >= 1.0,
  [AC_DEFINE( HAVE_LIBOPENEXR, 1, [OpenEXR libraries, optional, used in io routines.])
  openexr_support="yes"],[
  
  AC_MSG_WARN( $OPENEXR_PKG_ERRORS )

dnl Locate OpenEXR includes
   AC_MSG_CHECKING([for OpenEXR header files])
   CPATH_NO_COLON=`echo $CPATH | sed -e "s/:/ /g"`
   for DIR in ${prefix}/include /usr/local/include /usr/include /opt/local/include $CPATH_NO_COLON; do
       test -d "$DIR" && openexr_incpath=`find $DIR -name ImfRgbaFile.h`
       if test -n "$openexr_incpath"; then
          openexr_incpath="${openexr_incpath%%ImfRgbaFile.h}"
          AC_MSG_RESULT([yes])
          break
       fi
   done
   if test -z "$openexr_incpath"; then
      openexr_support="no"
      AC_MSG_RESULT([no])
   fi      
  
  #if not found                    
TMP_LIBS="$LIBS"
TMP_CXXFLAGS="$CXXFLAGS"
OPENEXR_LIBS="-lImath -lIlmImf -lHalf -lIex -lz"
OPENEXR_CFLAGS="-I$openexr_incpath"
LIBS="$OPENEXR_LIBS $LIBS"
CXXFLAGS="$CXXFLAGS $OPENEXR_CFLAGS"
AC_LINK_IFELSE([AC_LANG_PROGRAM([[#include <ImfRgbaFile.h>
using namespace Imf;
using namespace Imath;]], [[RgbaInputFile file("bzdet");]])],[AC_DEFINE( HAVE_LIBOPENEXR, 1, [OpenEXR libraries, optional, used in io routines.])
  AC_MSG_RESULT([yes]); openexr_support="yes"],[AC_MSG_RESULT([no]); openexr_support="no"])
LIBS=$TMP_LIBS
CXXFLAGS=$TMP_CXXFLAGS

]) #end of PKG_CHECK_MODULES

else
  openexr_support="no"
fi

                             
## QT library check
AC_ARG_WITH([qtdir],     [  --with-qtdir=DIR        Qt installation directory [default=$QTDIR]], QTDIR=$withval)
AC_ARG_WITH([qtinclude], [  --with-qtinclude=DIR    Qt include directory (overwrites qtdir)], QTINCLUDE=$withval)
AC_ARG_WITH([qtlibs],    [  --with-qtlibs=DIR       Qt library directory (overwrites qtdir)], QTLIBS=$withval)


if test "$do_qt" = "yes"; then

##If no QTDIR specified, try pkg-config
#if test x"$QTDIR" = x && test x"$QTINCLUDE" = x && test x"$QTLIBS" = x ; then

   PKG_CHECK_MODULES(QT, QtGui,
        [AC_DEFINE( HAVE_LIBQT, 1, [QT libraries, optional, used in pfsview.])
       QT_SUPPORT="yes"
     ],[
       #if pkg-config fails
	AC_MSG_WARN( $QT_PKG_ERRORS )
       pfs_CHECK_QT
#	QT_SUPPORT="no"
     ]) #end of PKG_CHECK_MODULES
#else
  ## Check QT using the specified QTDIR
#  pfs_CHECK_QT
#fi

## Look for MOC
if test -z "$QTDIR"; then
  MOC="moc-qt4"
else
  MOC="$QTDIR/bin/moc"
fi
AC_ARG_WITH(moc,
	[  --with-moc              Qt meta object compiler],
	[ MOC=$withval ])

AC_PATH_PROG(MOC, $MOC)
test -z "$MOC" && ( AC_MSG_WARN([no QT meta object compiler (moc) found in the path]); QT_SUPPORT="no" )     

else
  QT_SUPPORT="no"
fi

if test "$do_opengl" = "yes"; then             
  
# Checks for OpenGL under given operating system,
# adds operating system specific flags and libs.
case "$host_os" in
     [[cC]][[yY]][[gG]][[wW]][[iI]][[nN]]|mingw32|mks|cygwin)
        AC_DEFINE( WIN32, 1, [Windows compilation.])
        AC_MSG_CHECKING([for OpenGL])
        AC_COMPILE_IFELSE(
          AC_LANG_PROGRAM([[
                #include <windows.h>
                #include <GL/gl.h>
                ]],[[glEnd();]]),
          [AC_DEFINE( HAVE_LIBOPENGL, 1, [OpenGL libraries are required.])
           GL_LIBS="-L/usr/local/lib -lglut32 -lglu32 -lopengl32 -lgdi32"
           GL_CFLAGS="-I/usr/local/include -DCYGWIN -DWIN32 -D_WIN32 -DGLEW_STATIC"
           gl_support="yes"
           AC_MSG_RESULT([yes])],
          [AC_MSG_WARN( [OpenGL test failed. pfsglview will not be compiled] )
           gl_support="no"
          ])
        ;;

     darwin*)
           AC_DEFINE( HAVE_LIBOPENGL, 1, [OpenGL libraries are required.])
           GL_LIBS="$LIBS -framework GLUT -framework OpenGL -framework Cocoa"
           gl_support="yes"
        ;;

     *)
        AC_CHECK_LIB(GL, glDisable,
          [
           AC_CHECK_LIB(glut, glutInit, 
             AC_DEFINE( HAVE_LIBOPENGL, 1, [OpenGL libraries are required.])
             GL_LIBS="$LIBS -lglut -lGLU -lGL -lpthread"
             gl_support="yes", 
             AC_MSG_WARN( [GLUT library not found. pfsglview will not be compiled] )
             gl_support="no" )
           ],
          [AC_MSG_WARN( [OpenGL test failed. pfsglview will not be compiled] )
           gl_support="no"]
           ,)
        ;;
esac

else
  gl_support="no"
fi

dnl Is Mac OS - needed for pfsview
case "$host_os" in
     darwin*)
     	   AM_CONDITIONAL(MAC_OS, true)
	   ;;
     *)
     	   AM_CONDITIONAL(MAC_OS, false)
	   ;;
esac


dnl Check for MATLAB

MEX=mex
MEX_EXT=""

AC_ARG_WITH(mex,
	[  --with-mex              MATLAB compiler command],
	MEX=$withval)

if test "${prefix}" = "NONE"; then  
   MEX_DIR=`eval echo "/usr/local/share/${PACKAGE}/pfstools_matlab"`
else
   first_pass=`eval echo "${datadir}/${PACKAGE}/pfstools_matlab"`
   MEX_DIR=`eval echo "$first_pass"`
fi

dnl AC_DEFINE_UNQUOTED( [PCG_DATADIR], "$PCG_DATADIR", [Directory where data files are located.])
        
AC_ARG_WITH(mex-dir,
	[  --with-mex-dir          path for MATLAB files],
	MEX_DIR=$withval)
                
matlab_support="no";    
if test "$do_matlab" = "yes"; then

   AC_PATH_PROG(MEX, $MEX)   
   if test -n "$MEX" && $MEX -v 2>/dev/null | grep MATLAB >/dev/null; then
       MATLAB_ROOT=`matlab -e 2>/dev/null | grep '^MATLAB=' | sed -e 's/MATLAB=//'`;
       MEX_EXT=`"$MATLAB_ROOT/bin/mexext" 2>/dev/null`;
       if test -z "$MEX_EXT"; then
        AC_MSG_WARN([unable to determine mex extension, defaulting to mexglx])
	MEX_EXT="mexglx";
       fi
       matlab_support="yes";
   else
       if test -n "$MEX"; then
        AC_MSG_WARN([mex script does not seem to come from matlab (could be Octave mex script). Pass the path to matlab mex using --with-mex option.])
       else
        AC_MSG_WARN([no matlab mex found in the path])
       fi
   fi 
fi

dnl Check for GDAL
if test "$do_gdal" = "yes"; then                        
AC_CHECK_LIB(gdal, GDALAllRegister,
  [AC_DEFINE( HAVE_LIBGDAL,1,
	      [Define this if you have the GDAL library])
	      GDAL_LIBS="-lgdal"
              gdal_support="yes"],[gdal_support="no"],)
else
  gdal_support="no"
fi

# Add define pointing to data directory (for lookup tables, etc.)
if test "${prefix}" = "NONE"; then  
   PKG_DATADIR=`eval echo "/usr/local/share/${PACKAGE}"`
else
   first_pass=`eval echo "${datadir}/${PACKAGE}"`
   PKG_DATADIR=`eval echo "$first_pass"`
fi
AC_DEFINE_UNQUOTED( [PKG_DATADIR], "$PKG_DATADIR", [Directory where data files are located.])
AC_SUBST(PKG_DATADIR)                                  


dnl
dnl  Check for bash executable
dnl

AC_ARG_WITH(bash,
	[  --with-bash             location of the bash shell, needed for some pfs* scripts],
	[ BASH_PATH=$withval ])
        
test -z "$BASH_PATH" && BASH_PATH="bash"
                
AC_PATH_PROG(BASH_PATH, $BASH_PATH)
test -z "$BASH_PATH" && ( AC_MSG_WARN([no bash shell found in the path, some programs may fail to run]); BASH_PATH="/bin/bash" )     
                  
dnl
dnl Autoheader defines
dnl
AH_BOTTOM([/* Output stream for debug messages. */
#ifdef DEBUG
#define DEBUG_STR cerr
#else
#define DEBUG_STR if(1); else cerr
#endif
/* Output stream for verbose messages */        
#define VERBOSE_STR if(verbose) std::cerr << PROG_NAME << ": "        
])


## TODO: check by compiling a simple program if it is necessary
AH_BOTTOM([/* On some systems iostream, string etc are in 'std' namespace */
using namespace std; ])


##AC_PREFIX_DEFAULT("$HOME/local")
              
AC_SUBST(NETPBM_LIBS)
AC_SUBST(NETPBM_CFLAGS)
AC_SUBST(TIFF_LIBS)
AC_SUBST(OPENEXR_LIBS)
AC_SUBST(OPENEXR_CFLAGS)
AC_SUBST(IMAGEMAGICK_LIBS)
AC_SUBST(IMAGEMAGICK_CFLAGS)
AC_SUBST(JPEGHDR_LIBS)
AC_SUBST(GDAL_LIBS)
AC_SUBST(QT_LIBS)
AC_SUBST(QT_CFLAGS)
AC_SUBST(GL_LIBS)
AC_SUBST(GL_CFLAGS)
AC_SUBST(MOC)
AC_SUBST(CFLAGS)
AC_SUBST(PACKAGE_VERSION)
AC_SUBST(MEX)
AC_SUBST(MEX_DIR)
AC_SUBST(MEX_EXT)
AC_SUBST(BASH_PATH)
#QT_PATH=$qtpath
#AC_SUBST(QT_PATH)

if test "$do_octave" = "yes"; then
#Check for octave
#oct_version="2.1.35"
oct_version=""
AC_ARG_WITH(oct-version,
	[  --with-oct-version      Octave version, default empty, example: '2.1.35'],
	[ oct_version=$withval ])

if test -n "$oct_version"; then oct_version="-${oct_version}"; fi

OCTAVE_OCT_DIR=`octave-config${oct_version} --oct-site-dir`
OCTAVE_OCT_DIR=${OCTAVE_OCT_DIR:+${OCTAVE_OCT_DIR}/pfstools}
OCTAVE_M_DIR=`octave-config${oct_version} --m-site-dir`
OCTAVE_M_DIR=${OCTAVE_M_DIR:+${OCTAVE_M_DIR}/pfstools}

AC_ARG_WITH(octave-dir,
	[  --with-octave-dir       where Octave's .oct and .m files should be installed],
	[ OCTAVE_OCT_DIR=$withval; OCTAVE_M_DIR=$withval ])

AC_ARG_WITH(octave-m-dir,
	[  --with-octave-m-dir     where Octave's .m files should be installed],
	[ OCTAVE_M_DIR=$withval ])

AC_ARG_WITH(octave-oct-dir,
	[  --with-octave-oct-dir   where Octave's .oct files should be installed],
	[ OCTAVE_OCT_DIR=$withval ])

test -z "$OCTAVE_OCT_DIR" && AC_MSG_WARN([no path for .oct files specified])
test -z "$OCTAVE_M_DIR" && AC_MSG_WARN([no path for .m files specified])


MKOCTFILE="mkoctfile${oct_version}"
AC_ARG_WITH(mkoctfile,
	[  --with-mkoctfile        mkoctfile command],
	[ MKOCTFILE=$withval ])

AC_PATH_PROG(MKOCTFILE, $MKOCTFILE)
test -z "$MKOCTFILE" && AC_MSG_WARN([no mkoctfile found in the path])

if test -z "$MKOCTFILE" || test -z "$OCTAVE_OCT_DIR" || test -z "$OCTAVE_M_DIR"; then
   octave_support="no"
   AC_MSG_WARN([Octave support disabled])
else
   octave_support="yes"
fi        
else # do_octave == yes
   octave_support="no"
fi      
          
AC_PATH_PROG(PDFLATEX, [pdflatex])
test -z "$PDFLATEX" && AC_MSG_WARN([pdflatex not found in the path])
          
AM_CONDITIONAL(OCTAVE_SUPPORT, test "$octave_support" = "yes" )
AM_CONDITIONAL(PPM_SUPPORT, test "$netpbm_support" = "yes" )
AM_CONDITIONAL(OPENEXR_SUPPORT, test "$openexr_support" = "yes" )
AM_CONDITIONAL(TIFF_SUPPORT, test "$tiff_support" = "yes" )
AM_CONDITIONAL(JPEGHDR_SUPPORT, test "$jpeghdr_support" = "yes" )
AM_CONDITIONAL(IMAGEMAGICK_SUPPORT, test "$imagemagick_support" = "yes" )
AM_CONDITIONAL(QT_SUPPORT, test "$QT_SUPPORT" = "yes" )                             
AM_CONDITIONAL(PDFLATEX_SUPPORT, test -n "$PDFLATEX" )                             
AM_CONDITIONAL(GL_SUPPORT, test "$gl_support" = "yes" )
AM_CONDITIONAL(MATLAB_SUPPORT, test "$matlab_support" = "yes" )
AM_CONDITIONAL(GDAL_SUPPORT, test "$gdal_support" = "yes" )

AC_SUBST(MKOCTFILE)
AC_SUBST(OCTAVE_M_DIR)
AC_SUBST(OCTAVE_OCT_DIR)              

AC_CONFIG_FILES([ Makefile ])
AC_CONFIG_FILES([ src/Makefile ])
AC_CONFIG_FILES([ src/pfs/Makefile ])
AC_CONFIG_FILES([ src/pfs/pfs.pc ])
AC_CONFIG_FILES([ src/fileformat/Makefile ])
AC_CONFIG_FILES([ src/filter/Makefile ])
AC_CONFIG_FILES([ src/octave/Makefile ])
AC_CONFIG_FILES([ src/pfsview/Makefile ])
AC_CONFIG_FILES([ src/pfsglview/Makefile ])
AC_CONFIG_FILES([ src/matlab/Makefile ])
AC_CONFIG_FILES([ doc/Makefile ])
AC_CONFIG_FILES([ src/fileformat/pfsin ])
AC_CONFIG_FILES([ src/fileformat/pfsout ])
AC_CONFIG_FILES([ src/fileformat/pfsindcraw ])
AC_CONFIG_FILES([ src/fileformat/pfsinmulti ])
AC_CONFIG_FILES([ src/pfsview/pfsv ])
AC_CONFIG_FILES([ src/hdrhtml/Makefile ])
AC_OUTPUT


AC_MSG_RESULT([
pfstools is now configured for ${host}

  Source directory:     $srcdir
  Installation prefix:  $prefix
  C++ compiler:         $CXX $XTRA_CXXFLAGS $WARN_CXXFLAGS $CXXFLAGS
  Data directory:       $PKG_DATADIR

  Bash shell:           $BASH_PATH
  
  Octave                $octave_support
    octave-m-dir:       $OCTAVE_M_DIR
    octave-oct-dir:     $OCTAVE_OCT_DIR
    mkoctfile:          $MKOCTFILE

  Matlab                $matlab_support
    matlab-dir:         $MEX_DIR
    mex:                $MEX
    
  NetPBM                $netpbm_support])
if test "x$netpbm_support" = "xyes" ; then
AC_MSG_RESULT([    LIBS:               $NETPBM_LIBS])
AC_MSG_RESULT([    CFLAGS:             $NETPBM_CFLAGS])
else
AC_MSG_RESULT([      The following command(s) WILL NOT be compiled: pfsinppm pfsoutppm])
fi  
AC_MSG_RESULT([  TIFF                  $tiff_support])
if test "x$tiff_support" = "xyes" ; then
AC_MSG_RESULT([    LIBS:               $TIFF_LIBS])
else
AC_MSG_RESULT([      The following command(s) WILL NOT be compiled: pfsintiff pfsouttiff])
fi
AC_MSG_RESULT([  OpenEXR               $openexr_support])
if test "x$openexr_support" = "xyes" ; then
AC_MSG_RESULT([    CFLAGS:             $OPENEXR_CFLAGS
    LIBS:               $OPENEXR_LIBS])
else
AC_MSG_RESULT([      The following command(s) WILL NOT be compiled: pfsinexr pfsoutexr])
fi
AC_MSG_RESULT([  ImageMagick++         $imagemagick_support])
if test "x$imagemagick_support" = "xyes" ; then
AC_MSG_RESULT([    CFLAGS:             $IMAGEMAGICK_CFLAGS
    LIBS:               $IMAGEMAGICK_LIBS])
else
AC_MSG_RESULT([      The following command(s) WILL NOT be compiled: pfsinimgmagick pfsoutimgmagick pfsouthdrhtml])
fi    
AC_MSG_RESULT([  JPEG-HDR              $jpeghdr_support])
if test "x$jpeghdr_support" = "xyes" ; then
AC_MSG_RESULT([    LIBS:               $JPEGHDR_LIBS])       
else
AC_MSG_RESULT([      The following command(s) WILL NOT be compiled: pfsinjpeghdr pfsoutjpeghdr])
fi
AC_MSG_RESULT([  GDAL                  $gdal_support])
if test "x$gdal_support" = "xyes" ; then
AC_MSG_RESULT([    LIBS:               $GDAL_LIBS
  ])
else
AC_MSG_RESULT([      The following command(s) WILL NOT be compiled: pfsingdal])
fi
AC_MSG_RESULT([  QT                    $QT_SUPPORT])
if test "x$QT_SUPPORT" = "xyes" ; then
AC_MSG_RESULT([    QTDIR:              $QTDIR
    CFLAGS:             $QT_CFLAGS
    LIBS:               $QT_LIBS
    MOC:                $MOC
])
else
AC_MSG_RESULT([      The following command(s) WILL NOT be compiled: pfsview pfsv])
fi
AC_MSG_RESULT([  OpenGL                $gl_support])
if test "x$gl_support" = "xyes" ; then
AC_MSG_RESULT([    CFLAGS:             $GL_CFLAGS
    LIBS:               $GL_LIBS
    ])   
else
AC_MSG_RESULT([      The following command(s) WILL NOT be compiled: pfsglview])
fi
AC_MSG_RESULT([  Debug mode            $enable_debug

$HINTS  
])

if test "$QT_SUPPORT" = "no"; then 
AC_MSG_RESULT([
* To enable QT support, try specifying either --with-qtdir, or a triple --with-qtinclude, --with-qtlibs, --with-moc.
])
fi
