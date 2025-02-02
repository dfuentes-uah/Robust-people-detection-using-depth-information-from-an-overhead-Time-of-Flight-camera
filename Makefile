##############################################################################
#
# $Id: Makefile,v 1.7 2016/10/19 00:01:16 macias Exp $
#
##############################################################################

include ../makefile/Makefile.main

# Doxygen project name and brief description (will appear in the documentation title)
DOXYGEN_PROJECT_NAME      = Test application for readPMD3sFilesLib
DOXYGEN_BRIEF_DESCRIPTION = This application tests the readPMD3sFilesLib library

# Target related macros: actual exec app, and source files for libs
# and config (command line options) source files
TARGET         = tofCounting

LIBSRCS        = extractVector.cpp\
                 saveResult.cpp\
                 tiempo.cpp\
                 histogramasec.cpp\
                 clasificador.cpp\
                 geom_filters.cpp\
                 maxDetector.cpp\
                 direccion.cpp\
                 secciones.cpp\
                 DetectorTOF.cpp\
                 readWriteVector.cpp\
                 Funciones_opencv.cpp\
                 SVM.cpp\
                 ../readPMD3sFilesLibTest/readPMD3sLibTestFunction.c \
                 ../readPMD3sFilesLib/calcMaxMin.c \
                 ../readPMD3sFilesLib/readPMD3sFilesLib.c \
                 ../displayPMD3sFilesLib/displayPMD3sFilesLib.c \
                 ../tofCountingUtils/tofCountingUtils.c \
                 ../../far-field/sourceLocationResultsLib/sourcelocationresultslib.c \
                 ../../far-field/handleAudioFileLib/handleAudioFileLib.c

CFGSRCS        = ../commandLine/cfgViewTofStreams.c  \
                 ../commandLine/cfgDirGTFiles.c \
                 ../commandLine/cfgDirResultFiles.c \
                 ../commandLine/cfgCameraTypeFlags.c \
                 ../commandLine/cfgUseStreamFlags.c  \
                 ../commandLine/cfgUseStreamFlagsZorXyz.c \
								 ../commandLine/cfgResultFormat.c \
								 ../commandLine/cfgGtFormat.c \
								 ../commandLine/cfgTofCorrectGT.c \
								 ../commandLine/cfgTofCountingScoringThresholds.c \
								 ../commandLine/cfgDoNotDisplay.c  
#								 ../commandLine/cfgDirResultFiles.c 
#								 ../commandLine/cfgDirGTFiles.c \


# Include here main source code for your application
SOURCES =tofCounting.c $(LIBSRCS) $(CFGSRCS)

# Extra compiler flags to use
# This example shows support for the opencv default installation in your 
# system. This is usually opencv version 1
EXTRA_CFLAGS = -I../../far-field/ `pkg-config --cflags opencv` -I/usr/include/opencv -I/usr/include/opencv2 `pkg-config --cflags sndfile` -I../../ispaceDemos/ `pkg-config --cflags gsl`
# If you are developing in the geintra cluster, opencv 2 has been also
# installed, and you may use this other CFLAGS definition (see also the
# alternative EXTRALDFLAGS below:
# EXTRA_CFLAGS = `pkg-config --cflags opencv2`
# If you are using glib-2.0, you should add
# EXTRA_CFLAGS = `pkg-config --cflags glib-2.0`
#
# For libsndfile add `pkg-config --cflags sndfile`
#
# For fftw3 add `pkg-config --cflags fftw3`
#
# For GSL add `pkg-config --cflags gsl`
#
# For OpenNI add `pkg-config --cflags openni-dev`
#
# For openGL & friends add `pkg-config --cflags gl` `pkg-config --cflags glu`
#
# For libfreenect, add `pkg-config --cflags libfreenect` (YOU NEED TO ADD THE /usr/local/lib64/pkgconfig (OR WHATEVER) directory to PKG_CONFIG_PATH
#
# For cuda5-5 (only in supported machines) add -I/usr/local/cuda/include

# Extra linker flags to use (for opencv version 1)
EXTRALDFLAGS = -lm -lgeintra `pkg-config --libs opencv` `pkg-config --libs sndfile` `pkg-config --libs gsl` -lboost_system
# Extra linker flags to use (for opencv version 2 in the geintra cluster)
# EXTRALDFLAGS = -lm -lgeintra `pkg-config --libs opencv2`
# If you are using glib-2.0, you should add
# EXTRA_LDFLAGS = `pkg-config --libs glib-2.0`
#
# For libsndfile add `pkg-config --libs sndfile`
#
# For fftw3 add `pkg-config --libs fftw3`
#
# For GSL add `pkg-config --libs gsl`
#
# For OpenNI add `pkg-config --libs openni-dev`
#
# For openGL & friends add `pkg-config --libs gl` `pkg-config --libs glu` -lglut
#
# For libfreenect, add `pkg-config --libs libfreenect` (YOU NEED TO ADD THE /usr/local/lib64/pkgconfig (OR WHATEVER) directory to PKG_CONFIG_PATH)
#
# For cuda5-5 (only in supported machines) add -L/usr/local/cuda/lib64 -L/usr/local/cuda/lib

DEBUG = 1

# Set GENERATE_LIB=LIB if you want to generate a library instead of an application
GENERATE_LIB=

include ../makefile/Makefile.targets

##############################################################################
#
# $Log: Makefile,v $
# Revision 1.7  2016/10/19 00:01:16  macias
# Now support for other directories (z16 and destination .result). Still hard coded
#
# Revision 1.6  2016/07/13 16:16:46  macias
# Beautify
#
# Revision 1.5  2016/07/06 15:29:13  david.fuentes
# Summary: Reconstruction of Raquel Pca Alg.Added SVM with training
# Author: david.fuentes
#
# Revision 1.4  2016/06/30 08:55:43  david.fuentes
# *** empty log message ***
#
# Revision 1.3  2016/02/17 16:55:14  david.fuentes
# Getting ready to run for paper
#
# Revision 1.2  2015/11/09 14:37:04  david.fuentes
# *** empty log message ***
#
# Revision 1.1  2015/07/07 15:33:54  david.fuentes
# First commit
#
# Revision 1.10  2014/12/02 14:48:31  macias
# Segregated z/z16 and xyz stream types, to allow using them in tofScoring
# (so that scoring can be done in metric units if z or xyz are provided).
#
# Revision 1.9  2014/11/27 01:34:12  macias
# + Now readPMD3sFilesLibTest has command line arguments :-)
#
# Revision 1.8  2014/11/26 23:38:30  macias
# + Segregated calcMaxMin functionality in separate files
#
# Revision 1.7  2014/11/14 02:37:56  macias
# Trying to make readPMD3sFilesLib independent of the display (to avoid
# opencv dependencies)... wip
#
# Revision 1.6  2014/11/04 15:44:28  macias
# Routinary commit... back to working in mowgli
#
# Revision 1.5  2014/10/02 11:16:06  macias
# Now makefile support pcl
#
# Revision 1.4  2014/09/25 22:05:06  macias
# Now support for reading GT files
#
# Revision 1.3  2014/09/17 16:23:42  david.fuentes
# Fixed several bugs
#
# Revision 1.2  2014/07/29 23:47:07  macias
# + Function to read PMD3s data is ready and test function is also able to display sequence on screen
#
# Revision 1.1  2014/07/29 11:38:34  david.fuentes
# Start implementing readPMD3sFilesLib
# Fixed problems in locatingHM for PMDS3
#
# Revision 1.13  2014/04/23 23:01:17  macias
# Added cuda compilation issues
#
# Revision 1.12  2013/07/01 09:12:27  macias
# Added libfreenect references to use pkg-config .pc file
#
# Revision 1.11  2012-11-15 15:14:19  david.casillas
# Added pkg-config for openNI, gl, glu & glut
#
# Revision 1.10  2011-07-23 22:15:39  macias
# Added pkg-config for fftw3 and gsl
#
# Revision 1.9  2011-07-15 08:45:14  jesus.martinez
# Added pkg-config sample for libsndfile
#
# Revision 1.8  2011-07-14 08:55:40  macias
# + Added emacs variables to set makefile mode
# + SKELETON makefiles: Improved text in variables for doxygen definitions
#
# Revision 1.7  2011-07-14 08:54:53  macias
# + Added emacs variables to set makefile mode
# + SKELETON makefiles: Improved text in variables for doxygen definitions
#
# Revision 1.6  2011-07-11 17:08:12  macias
# Improved doxygen target
# + Not it automatically generates info for dependencies of the main
#   target
# + TODO: Remove files outside the current directory
# + Added $(TARGET).doxygen removal in make clean
# + Added $(TARGET).doxygen dependencies (same than target)
#
# Revision 1.5  2011-05-25 09:22:27  macias
# Added pkg support for glib-2.0 (due to compilation problems in Servidor)
#
# Revision 1.4  2011-04-01 20:20:05  macias
# Added additional info for opencv2 support
#
# Revision 1.3  2010-04-30 13:05:02  macias
# Fixed wrong placement of opencv pkg cflags
#
# Revision 1.2  2010-04-30 13:02:52  macias
# Added pkg flags for opencv
#
# Revision 1.1  2010-04-29 16:16:46  macias
# Added skeleton Makefile to generate app or static lib
#
# Revision 1.2  2009-10-31 23:58:19  macias
# Removed comment and makefile uses now EXTRA_CFLAGS and GENERATE_LIB
#
# Revision 1.1  2009-10-30 12:34:45  macias
# First release of GEINTRA hello world program.
#
##############################################################################

###########################################################################
# Local Variables:
# mode:makefile
# End:
###########################################################################
