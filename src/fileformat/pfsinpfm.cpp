/**
 * @brief Read files in PFM format
 * 
 * This file is a part of PFSTOOLS package.
 * ---------------------------------------------------------------------- 
 * Copyright (C) 2003,2004 Rafal Mantiuk and Grzegorz Krawczyk
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * ---------------------------------------------------------------------- 
 * 
 * @author Rafal Mantiuk, <mantiuk@mpi-sb.mpg.de>
 *
 * The format description was based on:
 * http://netpbm.sourceforge.net/doc/pfm.html
 *
 * $Id: pfsinpfm.cpp,v 1.1 2005/06/15 13:36:54 rafm Exp $
 */

#include <config.h>

#include <iostream>

#include <stdio.h>
#include <stdint.h>
#include <getopt.h>
#include <stdlib.h>
#include <math.h>

#include <pfs.h>

#define PROG_NAME "pfsinpfm"

struct PFMHeader
{
  int width, height;
  bool grayscale;
  float scale;
  bool need_swap;
};


static inline uint32_t pfs_bswap_32(uint32_t x)
{
  x= ((x<<8)&0xFF00FF00) | ((x>>8)&0x00FF00FF);
  x= (x>>16) | (x<<16);
  return x;
}

static float swap_if_needed(float x, bool swap_needed )
{
  if( swap_needed ) {
    uint32_t *xi = (uint32_t*)&x;
    uint32_t yi = pfs_bswap_32( *xi );
    float *y = (float*)&yi;    
    return *y;
  } else
    return x;
}


static bool isBigEndian()
{
  int x = 1;
  char *y = (char*)&x;
  return y[0] == 0;
}


PFMHeader readPFMHeader( FILE *fh )
{
  PFMHeader header;
  char headerID[2];
  int read;
    
  read = fscanf( fh, "%c%c\n%d %d\n%f", &headerID[0], &headerID[1],
    &header.width, &header.height,
    &header.scale );

  if( read != 5 )
    throw pfs::Exception( "Wrong file header" );

  if( !memcmp( "PF", headerID, 2 ) )
    header.grayscale = false;
  else if( !memcmp( "Pf", headerID, 2 ) )
    header.grayscale = true;
  else
    throw pfs::Exception( "Wrong PFM image type" );

  read = fread( headerID, 1, 1, fh );  // Read a single EOL character

  if( read != 1 || headerID[0] != 0x0a ) 
    throw pfs::Exception( "Wrong file header" );

  // Check if swapping bytes because of endianness is required
  header.need_swap = (isBigEndian() ^ (header.scale > 0));
  
  return header;
}

void readPFMFileColor( FILE *fh, PFMHeader &header, float *R, float *G, float *B )
{
  const int lineSize = header.width*3;
  float *line = new float[lineSize];

  // PFM images are stored in rows bottom to top
  for( int l = header.height-1; l >= 0; l-- ) {    
    int read = fread( line, sizeof( float ), lineSize, fh );
    if( read != lineSize )
      throw pfs::Exception( "Unexpected EOF" );
    for( int x = 0; x < header.width; x++ ) {
      const int lineOffset = l*header.width;
      R[lineOffset+x] = swap_if_needed( line[x*3+0], header.need_swap );
      G[lineOffset+x] = swap_if_needed( line[x*3+1], header.need_swap );
      B[lineOffset+x] = swap_if_needed( line[x*3+2], header.need_swap );      
    }
  }
  delete[] line;

  if( fabs(header.scale) != 1 ) {
    const float scaleFactor = fabs( header.scale );
    for( int i = 0; i < header.width*header.height; i++ ) {
      R[i] *= scaleFactor;
      G[i] *= scaleFactor;
      B[i] *= scaleFactor;
    }    
  }
  
}

void readPFMFileGrayscale( FILE *fh, PFMHeader &header, float *Y )
{
  // PFM images are stored in rows bottom to top
  for( int l = header.height-1; l >= 0; l-- ) {    
    int read = fread( Y + header.width*l,
      sizeof( float ), header.width, fh );

    if( header.need_swap ) {
      const int lineOffset = l*header.width;
      for( int x = 0; x < header.width; x++ ) {
        Y[lineOffset+x] = swap_if_needed( Y[lineOffset+x], 1 );
      }
    }
    
    if( read != header.width )
      throw pfs::Exception( "Unexpected EOF" );
  }
  if( fabs(header.scale) != 1 ) {
    const float scaleFactor = fabs( header.scale );
    for( int i = 0; i < header.width*header.height; i++ ) {
      Y[i] *= scaleFactor;
    }    
  }
}

// ======== Main

class QuietException 
{
};

void printHelp()
{
  fprintf( stderr, PROG_NAME " [--verbose] [--help]\n"
    "See man page for more information.\n" );
}


void readFrames( int argc, char* argv[] )
{
  pfs::DOMIO pfsio;

  bool verbose = false;

  // Parse command line parameters
  static struct option cmdLineOptions[] = {
    { "help", no_argument, NULL, 'h' },
    { "verbose", no_argument, NULL, 'v' },
    { "linear", no_argument, NULL, 'l' },
    { NULL, 0, NULL, 0 }
  };
  static const char optstring[] = "hvl";
    
  pfs::FrameFileIterator it( argc, argv, "rb", NULL, stdin,
    optstring, cmdLineOptions );
    
  int optionIndex = 0;
  while( 1 ) {
    int c = getopt_long (argc, argv, optstring, cmdLineOptions, &optionIndex);
    if( c == -1 ) break;
    switch( c ) {
    case 'h':
      printHelp();
      throw QuietException();
    case 'v':
      verbose = true;
      break;
    case 'l':
      std::cerr << PROG_NAME << " warning: linearize option ignored for an HDR input!"
                << std::endl;
      break;
    case '?':
      throw QuietException();
    case ':':
      throw QuietException();
    }
  }

 
  while( true )
  {
    pfs::FrameFile ff = it.getNextFrameFile();
    if( ff.fh == NULL ) break; // No more frames

    PFMHeader header = readPFMHeader( ff.fh );

    VERBOSE_STR << "reading file '" << ff.fileName << "'" << std::endl;
    VERBOSE_STR << "PFM file endianness: " << (header.scale > 0 ? "Big" : "Little" ) << std::endl;    
    
    pfs::Frame *frame = pfsio.createFrame( header.width, header.height );

    if( header.grayscale ) {
      pfs::Channel *Y;
      Y = frame->createChannel( "Y" );
      readPFMFileGrayscale( ff.fh, header, Y->getRawData() );      
    } else {
      pfs::Channel *X, *Y, *Z;
      frame->createXYZChannels( X, Y, Z );
      readPFMFileColor( ff.fh, header, X->getRawData(), Y->getRawData(),
        Z->getRawData() );
      pfs::transformColorSpace( pfs::CS_RGB, X, Y, Z, pfs::CS_XYZ, X, Y, Z );
    }

    it.closeFrameFile( ff );
    
    frame->getTags()->setString("LUMINANCE", "RELATIVE");

    const char *fileNameTag = strcmp( "-", ff.fileName )==0 ? "stdin" : ff.fileName;
    frame->getTags()->setString( "FILE_NAME", fileNameTag );

    pfsio.writeFrame( frame, stdout );
    pfsio.freeFrame( frame );
  }
}


int main( int argc, char* argv[] )
{
  try {
    readFrames( argc, argv );
  }
  catch( pfs::Exception ex ) {
    fprintf( stderr, PROG_NAME " error: %s\n", ex.getMessage() );
    return EXIT_FAILURE;
  }
  catch( QuietException  ex ) {
    return EXIT_FAILURE;
  }
  
  return EXIT_SUCCESS;
}
