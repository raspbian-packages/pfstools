/**
az @brief Read RAW Yuv files, commonly used for video compression
 * 
 * This file is a part of PFSTOOLS package.
 * ---------------------------------------------------------------------- 
 * Copyright (C) 2017 Rafal Mantiuk
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
 * $Id: pfsinrgbe.cpp,v 1.5 2014/06/17 21:57:08 rafm Exp $
 */

#include <config.h>
#include <string.h>

#include <cstdlib>

#include <algorithm>
#include <iostream>
#include <vector>

#include <getopt.h>
#include <pfs.h>
#include <pfsutils.cpp>
#include <climits>

#define PROG_NAME "pfsinyuv"

char charsToRemove[7] = {'b', 'i', 't', 's', 'f', 'p', 's'};

template<typename T>
inline T clamp( T val, T min, T max )
{
  if( val < min )
    return min;
  if( val > max )
    return max;
  return val;  
}


template<typename T>
bool read_yuv_channel( FILE *fh, int width, int height, int stride, pfs::Array2D *dest, float gain, float offset, float min, float max )
{
  T *line_buf = new T[width];
    
  for( int y = 0; y < height; y++ ) {      size_t read = fread( line_buf, sizeof( T ), width, fh );
  if( y == 0 && read == 0 ) { // End of file reached
    delete [] line_buf;
    return false;
  }
      
    if( read != width ) {
      delete [] line_buf;
      throw pfs::Exception( "Error when reading Yuv file" );
    }
    for( int x = 0; x < width; x++ ) {    
      float v_float = (float)line_buf[x]*gain - offset;        
      (*dest)(x<<stride, y<<stride) = clamp( v_float, min, max );
    }
    
  }    
  delete [] line_buf;
  return true;
}

void removeCharsFromString(std::string &str, char* charsToRemove ) {
  for ( unsigned int i = 0; i < strlen(charsToRemove); ++i ) {
        str.erase( remove(str.begin(), str.end(), charsToRemove[i]), str.end() );
   }
}

int parseIntField(std::string i){
  //parses an integer field. Removes pesky characters like the b in 10b or the p in 1080p
  removeCharsFromString(i, charsToRemove);
  return std::atoi(i.data());
}

//more readable helper method for strings, case insensitive
bool contains(std::string s1, std::string s2){
  
  std::transform(s1.begin(), s1.end(), s1.begin(), ::tolower);
  std::transform(s2.begin(), s2.end(), s2.begin(), ::tolower);
  
  return s1.find(s2) != std::string::npos || s1 == s2;
}


//returns a vector of strings tokenized 
std::vector<std::string> split(std::string s, std::string delim){
  return std::vector<std::string>();
}

bool parseFileName(const char * fileNameIn, int * width,  int * height, int * bitdepth, pfs::ColorSpace * colorspace, bool * upscale_420, int * fps){
  try{
    /* Parses the filename, puts the result into fields pointed to by the arguments
     returns true if all went well, false if there's a malformed filename
     string should be <prefix>_<width>x<height(p)>_<fps>_<bitdepth(b)>_<colorspace>_<chroma_format>
     Regex would be nice*/  
    std::string fileName(fileNameIn); 
    const std::string delimiter = "_";
    size_t pos = 0;
    std::string token;  
    std::vector<std::string> tokens;

    while ((pos = fileName.find(delimiter)) != std::string::npos) {
        token = fileName.substr(0, pos);
        tokens.push_back(token);
        fileName.erase(0, pos + delimiter.length());
    }
    tokens.push_back(fileName);

    for (std::vector<string>::iterator tok = tokens.begin() + 1; tok != tokens.end(); ++tok) {
      
      if (contains(*tok, "x")){
        sscanf(tok->c_str(), "%dx%d", width, height);
      }
      else if(*tok == "10" || *tok == "8" || *tok == "12" || tok->at(tok->length() - 1) == 'b' || contains(*tok, "bit")){ //last character is a b
        *bitdepth = parseIntField(*tok); 
      }
      else if(*tok == "24" || *tok == "25" || *tok == "50" || *tok == "60" || contains(*tok, "fps")){
        *fps = parseIntField(*tok);
      }
      else if (contains(*tok, "pq") || (contains(*tok, "2020") && !contains(*tok, "hlg"))){
        *colorspace = pfs::CS_PQYCbCr2020;
      }
      else if (contains(*tok, "bt") || contains(*tok, "709")){
        *colorspace = pfs::CS_YCbCr709;
      }
      else if (contains(*tok, "hlg")){
        *colorspace = pfs::CS_HLGYCbCr2020;
      }
      else if(contains(*tok, "444") || contains(*tok, "420")){
        *upscale_420 = contains(*tok, "420");
      }
    }
    return true;
  }
  catch(...){
    return false;
  }
}

void upscale_420to444( pfs::Array2D *ch )
{
  const int height = ch->getRows();
  const int width = ch->getCols();
  
  // For even rows
    for( int y = 0; y < height; y += 2 ) {
    float v_l = (*ch)(0,y);
    float v_r;
    int x;    
    for( x = 1; x < (width-1); x += 2 ) {     
      v_r = (*ch)(x+1,y);
      // Interpolate between pixels on the left and right
      (*ch)(x,y) = (v_l + v_r)/2.f;
      v_l = v_r;
    }
    if( x < width )
      (*ch)(x,y) = v_r;         
  }

  // For odd rows
  int y;
    for( y = 1; y < (height-1); y += 2 ) {
    for( int x = 0; x < width; x++ ) {          
      (*ch)(x,y) = ((*ch)(x,y-1) + (*ch)(x,y+1))/2.f;     
    }   
  }
  // Last row
  if( y < height ) {
    for( int x = 0; x < width; x++ ) {          
      (*ch)(x,y) = (*ch)(x,y-1);      
    }   
  }   
}


class YUVReader
{
  FILE *fh;
  int width, height;
  int bit_depth;
  bool subsampling_420;
  unsigned long int fileSize;
  
  enum ColorSpace { REC709, REC2020 };
  ColorSpace color_space;      
  
public:
  YUVReader( FILE *fh, int width, int height, int bit_depth, bool subsampling_420 ) : fh(fh), width( width ), height( height ), bit_depth( bit_depth ), subsampling_420( subsampling_420 )
  {
    //initialize filesize
    fseek(fh, 0L, SEEK_END);
    fileSize = ftell(fh);
    //go back from where we came
    fseek(fh, 0L, SEEK_SET);
  }
  
  int getWidth() const 
    {
      return width;
    }
  
  int getHeight() const
    {
      return height;
    }

  unsigned long int getFileSize() const
    {
      return fileSize;
    }
  
  /*
  Advances a number of frames through the file without reading them */ 
//  bool advanceFrames(int frameNo){
//    int divisor = subsampling_420 ? 4 : 1; //this is the integer divisor of the green and blue channels (they are 4 times smaller if 4:2:0 subsampling is enabled) 
//    long int offset = frameNo * (/*Red:*/ width * height * sizeof(STORE_FORMAT) + /*Green, Blue:*/ 2*width*height*sizeof(STORE_FORMAT)/divisor);
//    return fseek(fh, offset, SEEK_CUR);
//  }

  /* See to a given frame */
  bool seekToFrame(int frameNo){
    unsigned long int offset = getFileOffsetFromFrame(frameNo);
    if(offset > fileSize){
      throw pfs::Exception("Seeking past EOF, is your given frame range within the range of the input?");
    }
    else{
      return fseek(fh, offset, SEEK_SET);
    }
  }

  unsigned long int getFileOffsetFromFrame(int frameNo){
    unsigned long int divisor = subsampling_420 ? 4 : 1; //this is the integer divisor of the green and blue channels (they are 4 times smaller if 4:2:0 subsampling is enabled)

    if( frameNo <= 0 )
      throw pfs::Exception( "Invalid frame index. Frame are indexed from 1" );

    unsigned long int storeFormatSize = bit_depth <= 8 ? sizeof(unsigned char) : sizeof(unsigned short);
    unsigned long int offset = (frameNo-1) * (/*Luma:*/ width * height * storeFormatSize + /*CrCb:*/ 2*width*height*storeFormatSize/divisor);
    return offset;
  }
  
  /**
   * Read a single frame from Yuv video file and store in 3 RGB channels of type pfs::Array. 
   * Integer values are converted into floating point values.
   * 
   * @return TRUE if the frame has been loaded successfully, FALSE if end-of-file. 
   * Exception is thrown if there is an issue reading a full frame.
   */

  bool readImage( pfs::Array2D *R, pfs::Array2D *G, pfs::Array2D *B ){  
    if(bit_depth <= 8){
      return readImageTyped<unsigned char>(R, G, B);
    }
    else{
      return readImageTyped<unsigned short>(R, G, B);
    }
  }

  private: 
    template<typename T>
    bool readImageTyped( pfs::Array2D *R, pfs::Array2D *G, pfs::Array2D *B)
    {

      { // Read Y
      const float offset = 16.f/219.f;
      const float gain = 1.f/( (float)(1<<(bit_depth-8))*219.f);    
      if( !read_yuv_channel<T>( fh, width, height, 0, R, gain, offset, 0.f, 1.f ) )  {
        return false;
      }   
      }
      
      { // Read Cb, Cr
        const float offset = 128.f/224.f;
        const float gain = 1.f/( (float)(1<<(bit_depth-8))*224.f);
        unsigned int chroma_width = width;
        unsigned int chroma_height = height;
        unsigned int chroma_stride = 0;
        if (subsampling_420){
          chroma_width = width/2;
          chroma_height = height/2;
          chroma_stride = 1;  
        } 

        if( !read_yuv_channel<T>( fh, chroma_width, chroma_height, chroma_stride, G, gain, offset, -0.5f, 0.5f ) ) 
        throw pfs::Exception( "EOF reached when reading the Cb portion of a frame" );

        if( !read_yuv_channel<T>( fh, chroma_width, chroma_height, chroma_stride, B, gain, offset, -0.5f, 0.5f ) ) 
        throw pfs::Exception( "EOF reached when reading the Cr portion of a frame" );
        if(subsampling_420){
          upscale_420to444( B ); 
          upscale_420to444( G );
        }
      }
    return true;    
    }
};


class QuietException 
{
};

void printHelp()
{
  std::cerr << PROG_NAME " [--verbose] [--quiet] [--width] [--height] [--colorspace] [--no-guess] [--chroma-format] [--bit-depth] [--frames] [--help]" << std::endl
            << "See man page for more information." << std::endl;
}


void readFrames( int argc, char* argv[] )
{
  pfs::DOMIO pfsio;

  bool verbose = false;
  bool quiet = false;
  bool opt_noguess = false;
  int opt_width = -1;
  int opt_height = -1;
  int opt_bitdepth = -1;
  const char * opt_chroma_subsampling = NULL;
  int opt_frame_min = -1;
  int opt_frame_max = -1;
  int opt_frame_stride = 0;
  int opt_fps = -1;
  char * filename;
  pfs::ColorSpace opt_colorspace = pfs::CS_INVALID;
  
  // Parse command line parameters
  static struct option cmdLineOptions[] = {
    { "help", no_argument, NULL, 'p' },
    { "verbose", no_argument, NULL, 'v' },
    { "width", required_argument, NULL, 'w' },
    { "height", required_argument, NULL, 'h' },
    { "bit-depth", required_argument, NULL, 'b' },
    { "colorspace", required_argument, NULL, 'c' },           
    { "quiet", no_argument, NULL, 'q' },
    { "no-guess", no_argument, NULL, 'n'},
    { "fps", required_argument, NULL, 'f'},
    { "chroma-format", required_argument, NULL, 's'},
    { "frames", required_argument, NULL, 'r'},
    { NULL, 0, NULL, 0 }
  };
  static const char optstring[] = "-pw:h:vqb:c:f:nr:s:";

  int optionIndex = 0;
  while( 1 ) {
    int c = getopt_long (argc, argv, optstring, cmdLineOptions, &optionIndex);
    if(c == -1){ 
      break;
    }
    switch( c ) {
    case 'p':
      printHelp();
      throw QuietException();
    case 'v':
      verbose = true;
      break;
    case 'w':
      opt_width = strtol( optarg, NULL, 10 );
      break;
    case 'h':
      opt_height = strtol( optarg, NULL, 10 );
      break;
    case 'b':
      opt_bitdepth = strtol( optarg, NULL, 10 );
      break;
    case 's':
      opt_chroma_subsampling = optarg;
      break; 
    case 'n': 
      opt_noguess = true;
      break;
    case 'f':
      opt_fps = strtol(optarg, NULL, 10);
      break;
    case 'r': {
      pfs::parseFrameRange(optarg, opt_frame_min, opt_frame_max, opt_frame_stride);
      VERBOSE_STR << "Reading frames " << opt_frame_min << ":" << opt_frame_stride << ":"
                  << opt_frame_max << std::endl;      
      break;
    }
    case 'c':
      if( !strcasecmp( optarg, "pq2020" ) ) {
        opt_colorspace = pfs::CS_PQYCbCr2020;
      }
      else if( !strcasecmp( optarg, "bt709" )) {
      opt_colorspace = pfs::CS_YCbCr709;
      }
      else if( !strcasecmp( optarg, "hlg2020")){
        opt_colorspace = pfs::CS_HLGYCbCr2020;
      } 
      else {
        throw pfs::Exception( "Unrecognized colorspace name" );
      }
      break;
    case 'q':
      quiet = true;
      break;
    case '?':
      throw QuietException();
    case ':':
      throw QuietException();
    case '\1':
      filename = optarg;
    }
  }

  FILE * fh;
  if(filename != NULL){
    fh = fopen (filename, "rb");
  }

  if( fh == NULL ) {
    throw pfs::Exception("Couldn't find a filename to open, did you provide one");
  }; // No more frames

  int width = -1, height = -1, bitdepth = -1, frame_min = 1, frame_max = INT_MAX, frame_stride = 1, fps = -1;
  bool upscale_420 = true;
  pfs::ColorSpace colorspace = pfs::CS_INVALID;

  VERBOSE_STR << "reading file '" << filename << "'" << std::endl;
  //we infer the metadata from the filename unless noguess is specified
  if(!opt_noguess){
    std::string rawName(filename); //these three lines extract the filename proper, without the containing directory prefix
    long unsigned int substring = rawName.find_last_of("\\/");
    rawName = rawName.substr(substring == std::string::npos ? 0 : substring);
    bool goodFileName = parseFileName(rawName.c_str(), &width, &height, &bitdepth, &colorspace, &upscale_420, &fps);  
    if(!goodFileName){
      VERBOSE_STR << "Unable to parse filename: " << filename << "\n";
    }
  }
  // Over-ride the auto-recognized params with the ones specified as arguments
  if( opt_width > -1 )
    width = opt_width;
  if( opt_height > -1 )
    height = opt_height;
  if( opt_bitdepth > -1 )
    bitdepth = opt_bitdepth;
  if( opt_frame_min > -1 )
    frame_min = opt_frame_min;
  if( opt_frame_max > -1 )
    frame_max = opt_frame_max;
  if( opt_frame_stride != 0)
    frame_stride = opt_frame_stride;
  if( opt_colorspace != pfs::CS_INVALID )
    colorspace = opt_colorspace;
  if( opt_chroma_subsampling != NULL ){
    // bad reading, essentially just force upscaling if they input the subsampling as 420
    upscale_420 = strcmp(opt_chroma_subsampling, "420") == 0;   
  } 
  if( opt_fps > 0 ){
    fps = opt_fps;
  }

  VERBOSE_STR << "Yuv file " << width << "x" << height << " " << bitdepth << "bits" << std::endl;
  switch( colorspace ) {
    case pfs::CS_PQYCbCr2020:
      VERBOSE_STR << "colorspace: HDR PQ BT2020" << std::endl;
      break;
    case pfs::CS_YCbCr709:
      VERBOSE_STR << "colorspace: SDR BT709" << std::endl;
      break;
    case pfs::CS_HLGYCbCr2020:
      VERBOSE_STR << "colorspace: HDR HLG BT2020" << std::endl;
      break;
  }

  if( width <= 0 || height <= 0 )
    throw pfs::Exception( "Unspecified or incorrect resolution of the Yuv file" );

  if( bitdepth < 8 || bitdepth > 16 )
    throw pfs::Exception( "Unspecified or incorrect bit-depth of the Yuv file" );
  
  if( frame_min < 0 || frame_max < 0){
    throw pfs::Exception( "Frame range is invalid ");
  }
  if( colorspace == pfs::CS_INVALID ){
    throw pfs::Exception( "Unspecified colorspace of the Yuv file" );
  }
  if( fps > 0 ){
    VERBOSE_STR << "FPS: " << fps << std::endl;
  }

  YUVReader reader( fh, width, height, bitdepth, upscale_420);

  
  int currentFrame = frame_min;

  while( (currentFrame <= frame_max && frame_stride > 0)
      || (currentFrame >= frame_max && frame_stride < 0) ) { //inclusive
    
    pfs::Frame *frame = pfsio.createFrame( reader.getWidth(),
                                           reader.getHeight() );
    pfs::Channel *X, *Y, *Z;
    frame->createXYZChannels( X, Y, Z );

    if(reader.seekToFrame(currentFrame)) {
      //some error occured in reading :(
      pfsio.freeFrame( frame ); 
      break;
    }
    
    //Store RGB data temporarily in XYZ channels
    bool success = reader.readImage( X, Y, Z );
    if( !success ) { // EOF reached
      pfsio.freeFrame( frame );
      break;
    }

    VERBOSE_STR << "Reading frame " << currentFrame << std::endl;    

    if( colorspace == pfs::CS_YCbCr709 ) {
      // The trick to get LDR data in the pfs stream
      pfs::transformColorSpace( colorspace, X, Y, Z, pfs::CS_SRGB, X, Y, Z );
      pfs::transformColorSpace( pfs::CS_RGB, X, Y, Z, pfs::CS_XYZ, X, Y, Z );
    } else {      
      pfs::transformColorSpace( colorspace, X, Y, Z, pfs::CS_XYZ, X, Y, Z );
    }
    

    switch( colorspace ) {
      case pfs::CS_PQYCbCr2020:
        frame->getTags()->setString("LUMINANCE", "ABSOLUTE");
        break;
      case pfs::CS_YCbCr709:
        frame->getTags()->setString("LUMINANCE", "DISPLAY");
        break;
      case pfs::CS_HLGYCbCr2020:
        frame->getTags()->setString("LUMINANCE", "ABSOLUTE");
        break;
    }

    if( fps > 0 ){
      frame->getTags()->setString("FPS", pfs::intToString(fps).c_str());
    }
    
    const char *fileNameTag = strcmp( "-", filename )==0 ? "stdin" : filename;
    frame->getTags()->setString( "FILE_NAME", fileNameTag );

    pfsio.writeFrame( frame, stdout );
    pfsio.freeFrame( frame );
    
    currentFrame += frame_stride;
  }
  fclose(fh);
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
  catch( QuietException ) {
    return EXIT_FAILURE;
  }        
  return EXIT_SUCCESS;
}
