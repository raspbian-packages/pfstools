/**
 * @brief PFS library - additional utilities
 * 
 * This file is a part of PFSTOOLS package.
 * ---------------------------------------------------------------------- 
 * Copyright (C) 2006 Radoslaw Mantiuk
 * 
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * ---------------------------------------------------------------------- 
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <algorithm>
#include <math.h>
#include <pfs.h>
#include "picture_io.h"
//#define DEBUG
#define min(x,y) ( (x)<(y) ? (x) : (y) )
#define max(x,y) ( (x)>(y) ? (x) : (y) )

#define PIXEL_ALIGMENT 4 // number of bytes occupied by one pixel

extern bool verbose;

inline int clamp( int val, int min, int max)
{
  if( val < min ) return min;
  if( val > max ) return max;
  return val;
}

PictureIO::PictureIO(LumMappingMethod mappingMethod, float minLum, float maxLum)
{
	currentFrame = frameList.end();
	visibleChannel = CHANNEL_XYZ;
	imageMappingMethod = mappingMethod;
	minLuminance = minLum;
	maxLuminance = maxLum;
	pfsFrame = NULL; // current (active) pfsframe
	frameNo = 0;
	chR = chG = chB = NULL;
	CHANNEL_XYZ = "XYZ";
	CHANNEL_X = "X";
	CHANNEL_Y = "Y";
	CHANNEL_Z = "Z";
	
	try {
		if( !readNextFrame() ) 
			throw PFSglViewException();
		else
			frameNo++;
    
	} catch( pfs::Exception ex ) {
		throw PFSglViewException();
	}
	
}

PictureIO::~PictureIO() {
	delete pfsFrame;
	delete data;
	delete chR;
	delete chG;
	delete chB;
}

/**
*/
void PictureIO::setFrame(pfs::Frame *frame, const char *channel) {

	if( frame == NULL)
		return;

	// only XYZ channels are taken into consideration
	if( channel != "XYZ" &&  channel != "X00" && channel != "0Y0" && channel != "00Z" ) {
		if(verbose)
			fprintf( stderr, "WARNING: wrong channel (PictureIO::setFrame())\n");
		return;
	}

	if(pfsFrame != frame)
	{
		pfsFrame = frame;
		currentFileName = frame->getTags()->getString("FILE_NAME");
	}
	
	visibleChannel = channel;

	pfs::Channel *X, *Y, *Z;
	pfsFrame->getXYZChannels( X, Y, Z );
	if( X == NULL) {
		if(verbose)
			fprintf(stderr, "WARNING: No color channel avaible (PictureIO::setFrame())\n");
		return;
	}		
	
	width = X->getCols();
	height = X->getRows();	
	
	data = new unsigned char[width*height*PIXEL_ALIGMENT];
	
	chR = new pfs::Array2DImpl( width, height);
	chG = new pfs::Array2DImpl( width, height);
	chB = new pfs::Array2DImpl( width, height);
	
	pfs::transformColorSpace( pfs::CS_XYZ, X, Y, Z, pfs::CS_RGB, chR, chG, chB);	
	
	updateMapping();
} 


void PictureIO::updateMapping( void) {
	
	#define LUMSIZE 256
	
	int rows = chR->getRows();
	int cols = chR->getCols();
	int lumSize = 257+1;            // +1 - for lower bound
	float lumPixFloor[lumSize];

	for( int p = 1; p <= LUMSIZE+1; p++ ) {
	
		float p_left = ((float)p - 1.f)/255.f; // Should be -1.5f, but we don't want negative nums
		
		lumPixFloor[p] = getInverseMapping( imageMappingMethod, p_left, minLuminance, maxLuminance );
	}
		
	int index = 0;
	
	
	pfs::Array2D* ch = NULL;
	if( visibleChannel != CHANNEL_XYZ) {
	
		if( visibleChannel == CHANNEL_X)
			ch = chR;
		else	
		if( visibleChannel == CHANNEL_Y)
			ch = chG;
		else
		if( visibleChannel == CHANNEL_Z)
			ch = chB;
	}		
			
	for( int r = 0; r < rows; r++)
		for( int c = 0; c < cols; c++) {
			
			// Color channels
			int pr, pg, pb;
			
			if( visibleChannel == CHANNEL_XYZ) {
			
				pr = binarySearchPixels( (*chR)(c, r), lumPixFloor, lumSize );        
				pg = binarySearchPixels( (*chG)(c, r), lumPixFloor, lumSize );        
				pb = binarySearchPixels( (*chB)(c, r), lumPixFloor, lumSize );
				
				pr = clamp( pr, 1, 256 );
				pg = clamp( pg, 1, 256 );
				pb = clamp( pb, 1, 256 );
			}
			else {
				pr = binarySearchPixels( (*ch)(c, r), lumPixFloor, lumSize );
				pr = clamp( pr, 1, 256 );
				pg = pr;
				pb = pr;
			}	
						
			index = r * (cols * PIXEL_ALIGMENT) + c * PIXEL_ALIGMENT;
				
			data[index] = pr - 1;
			data[index + 1] = pg - 1;
			data[index + 2] = pb - 1;			
		}
}


/** Changes mapping method.
*/
void PictureIO::changeMapping( LumMappingMethod mappingMethod, float minLum, float maxLum) {

	if( imageMappingMethod == mappingMethod && minLum == minLuminance && maxLum == maxLuminance)
		return;
		
	imageMappingMethod = mappingMethod;
	minLuminance = minLum;
	maxLuminance = maxLum;
	
	updateMapping();
}

void PictureIO::changeMapping( LumMappingMethod mappingMethod) {

	if( imageMappingMethod == mappingMethod)
		return;
		
	imageMappingMethod = mappingMethod;
	updateMapping();
}

void PictureIO::changeMapping( float minLum, float maxLum) {

	if( minLum == minLuminance && maxLum == maxLuminance)
		return;
	minLuminance = minLum;
	maxLuminance = maxLum;
	
	updateMapping();
}

void PictureIO::changeMapping( void) {

	updateMapping();
}


/**
 * Searching for the closest pixel index (m) of a given luminance (lum) in the luminance 
 * mapping table - lumMap.
 */
inline int PictureIO::binarySearchPixels( float lum, const float *lumMap, const int lumSize )
{
  int l = 0, r = lumSize;
  while( true ) {
    int m = (l+r)/2;
    if( m == l ) break;
    if( lum < lumMap[m] )
      r = m;
    else
      l = m;
  }
  return l;
}


/**
 * Searching for the closest pixel index (m) of a given luminance (lum) in the luminance 
 * mapping table - lumMap.
 */
float PictureIO::getInverseMapping( LumMappingMethod mappingMethod, float v, float minValue, float maxValue )
{
  switch( mappingMethod ) {
  case MAP_GAMMA1_4:
    return powf( v, 1.4 )*(maxValue-minValue) + minValue;
  case MAP_GAMMA1_8:
    return powf( v, 1.8 )*(maxValue-minValue) + minValue;
  case MAP_GAMMA2_2:
    return powf( v, 2.2 )*(maxValue-minValue) + minValue;
  case MAP_GAMMA2_6:
    return powf( v, 2.6 )*(maxValue-minValue) + minValue;
  case MAP_LINEAR:
    return v*(maxValue-minValue) + minValue;
  case MAP_LOGARITHMIC:
    return powf( 10, v * (log10f(maxValue) - log10f(minValue)) + log10f( minValue ) );
  default:
    assert(0);
    return 0;
  }
}


	
/**
 * GUI action -> goto next frame
 * handle error and eof of frame
 */

void PictureIO::gotoNextFrame()
{
	try {
		if(readNextFrame())
			frameNo++;
	}
	catch( pfs::Exception ex ) 	{
		// Display message and keep the old frame
		fprintf(stderr, "pfsGLview error - exiting\n");
		exit(-1);
	}
}

void PictureIO::gotoPreviousFrame()
{
	currentFrame++;
	if( currentFrame == frameList.end() ) 
	{
		currentFrame--;
		//fprintf(stderr, "No more frames in buffer (buffer holds max 5 frames)\n");
		return;
	}
	setFrame( *currentFrame, CHANNEL_XYZ);
	frameNo--;
}

	

bool PictureIO::hasColorChannels( pfs::Frame *frame )
{
	if( frame == NULL ) 
		frame = pfsFrame;
	assert( frame != NULL );
	pfs::Channel *X, *Y, *Z;
	frame->getXYZChannels( X, Y, Z );
	return ( X != NULL );
}

const pfs::Array2D* PictureIO::getPrimaryChannel(void) {

	assert( pfsFrame != NULL );
	
	if( visibleChannel == CHANNEL_XYZ ) {
		pfs::Channel *X, *Y, *Z;
		pfsFrame->getXYZChannels( X, Y, Z );
		return Y;
	} else {
		return pfsFrame->getChannel( visibleChannel );
	}
  
}

int PictureIO::getWidth() {
	return width;
}
int PictureIO::getHeight() {
	return height;
}

int PictureIO::getPixel(int ch, int c, int r) {

	if( c < 0 || c >= width || r < 0 || r >= height) {
		//fprintf( stderr, "WARNING: pixel position out of range (PictureIO::getPixel())\n");
		return -1;
	}
	int index = r * (width * PIXEL_ALIGMENT) + c * PIXEL_ALIGMENT;
				
	return (int)data[index + ch];
}

int PictureIO::getPixelR(int c, int r) {
				
	return getPixel(0, c, r);
}
int PictureIO::getPixelG(int c, int r) {
				
	return getPixel(1, c, r);
}
int PictureIO::getPixelB(int c, int r) {
				
	return getPixel(2, c, r);
}

int PictureIO::getRawData( int x, int y, float& XX, float& YY, float& ZZ) {

	if( x < 0 || y < 0 || x >=width || y >= height || pfsFrame == NULL) {
		if(verbose) fprintf( stderr, "WARNING: wrong pixel coordinates (PictureIO::getRawData())\n");
		return 1;
	}	
	
	pfs::Channel *X, *Y, *Z;
	pfsFrame->getXYZChannels( X, Y, Z );	
	if( X == NULL) {
		if(verbose)
			fprintf(stderr, "WARNING: No color channel avaible (PictureIO::getRawData())\n");
		return 1;
	}			
	
	XX = (*X)( x,y);
	YY = (*Y)( x,y);
	ZZ = (*Z)( x,y);
	
	return 0;
}



const char *PictureIO::getCurrentFileName()
{
	return currentFileName;
}	 


const char *PictureIO::getVisibleChannel()
{
	return visibleChannel;
}

void PictureIO::setVisibleChannel(const char *channel) {

	if( channel == CHANNEL_XYZ 
		|| channel == CHANNEL_X 
		|| channel == CHANNEL_Y 
		|| channel == CHANNEL_Z)
		visibleChannel = channel;
	else
	if(verbose)
		fprintf( stderr, "WARNING: wrong channel name (%s) (PictureIO::setVisibleChannel())\n", channel);
}

pfs::Frame *PictureIO::getFrame()
{
	return pfsFrame;
}

/** Saves bitmap data to stdout in pfstools format. */

// gamma correction for MAC
#define GAMMA_CORRECTION 1.8f

int PictureIO::save(void){

	pfs::DOMIO io;
	
	pfs::Frame* frame = io.createFrame( width, height);
	pfs::Channel *X = frame->createChannel( "X");
	pfs::Channel *Y = frame->createChannel( "Y");
	pfs::Channel *Z = frame->createChannel( "Z");
	frame->createXYZChannels( X, Y, Z);
	
	for(int j = 0; j < (width * height); j++){
	
		(*X)(j) = powf((float)data[j*PIXEL_ALIGMENT] / 256.0f, GAMMA_CORRECTION);
		(*Y)(j) = powf((float)data[j*PIXEL_ALIGMENT+ 1] / 256.0f, GAMMA_CORRECTION);
		(*Z)(j) = powf((float)data[j*PIXEL_ALIGMENT + 2] / 256.0f, GAMMA_CORRECTION);
	}	
	
	pfs::transformColorSpace( pfs::CS_RGB, X, Y, Z, pfs::CS_XYZ, X, Y, Z );
	
	io.writeFrame( frame, stdout );
	io.freeFrame( frame );  
		
	return 0;
}


/** Returns minimum luminance.
*/
float PictureIO::computeLumMin(void) {

	float valMin = 1e30;
	
	const pfs::Array2D* image = getPrimaryChannel();
	const int size = image->getRows()*image->getCols();
	
	for( int i = 0; i < size; i++ ) {
	
		float val = (*image)(i);
		if( val < valMin)
			valMin = val;
	}		

	return valMin;
}

/** Returns maximum luminance.
*/
float PictureIO::computeLumMax(void) {

	float valMax = -1e30;
	
	const pfs::Array2D* image = getPrimaryChannel();
	const int size = image->getRows()*image->getCols();
	
	for( int i = 0; i < size; i++ ) {
	
		float val = (*image)(i);
		if( val > valMax)
			valMax = val;
	}		

	return valMax;
}

/** Returns maximum luminance.
*/
float PictureIO::getLumMax(void) {
	return maxLuminance;
}
float PictureIO::getLumMin(void) {
	return minLuminance;
}


/**
 * Load next frame from the stream.
 * @return false if there are no more frames
 */
bool PictureIO::readNextFrame()
{
	if( currentFrame != frameList.begin() )
	{
		currentFrame--;
		setFrame( *currentFrame, "XYZ" );
		return true;
	}

	if( frameList.size() == MAX_FRAMES_IN_MEMORY ) {
	
		#ifdef DEBUG
		fprintf(stderr, "No memory for the next frame.\n");
		#endif
		return false;
	}
  
	pfs::DOMIO pfsCtx;
	pfs::Frame *newFrame;
	#ifdef DEBUG
	fprintf(stderr, "Reading PFS Frame\n");
	#endif
	
	newFrame = pfsCtx.readFrame( stdin );
	if( newFrame == NULL ) 
	{
		#ifdef DEBUG
		fprintf(stderr, "No more frames\n");
		#endif
		return false; // No more frames
	}
		
	frameList.push_front( newFrame );
	currentFrame = frameList.begin();
	setFrame( newFrame, "XYZ" );  
	
	#ifdef DEBUG
	fprintf(stderr, "Number of frames: %d\n", frameList.size());
	#endif	
	
	return true;
}

/** Returns current frame number.
*/
int PictureIO::getFrameNo(void) {
	return frameNo;
}


/** 
*/
float PictureIO::getDynamicRange(void) {

	float valMin = computeLumMin();
	float valMax = computeLumMax();
	unsigned int max = (unsigned int)pow( 2, 32);
	
	printf(" lumMax:%f  lumMin:%f  max:%d  dr:%f\n", valMax, valMin, max, log10(valMax/valMin));
	
	const pfs::Array2D* image = getPrimaryChannel();
	int size = image->getRows() * image->getCols();
	
	float fval;
	unsigned int val;
	std::vector<unsigned int> vec;
	for( int i = 0; i < size; i++ ) {
	
		fval = (*image)(i);
			
		fval /= valMax;
		val = (unsigned int)(fval * max);
		vec.push_back(val);
		
	}	
	std::sort(vec.begin(), vec.end());
	
	printf("size: %d\n", vec.size());
	
	val = vec[0];
	std::vector<unsigned int> svec;
	svec.push_back(val);
	
	for( int i = 1; i < vec.size(); i++) {
		
		if( vec[i] != val) {
			svec.push_back(vec[i]);
			val = vec[i];
		}
	}
	
	float dr = svec[ svec.size()-1] / svec[0];
			
	printf("min:%ld  max:%ld  svec size: %d  dr:%f\n", vec[0], vec[ vec.size()-1], svec.size(), log10(dr));

	

	return valMax;
}


/**
*/
std::vector<const char*> PictureIO::getChannelNames() {

	std::vector<const char*> vec;
	if( pfsFrame == NULL)
		return vec;

	pfs::ChannelIterator *it = pfsFrame->getChannels();
	
	while(it->hasNext() ) 	
		vec.push_back( it->getNext()->getName());

	return vec;
}

/**
*/
void PictureIO::setMinLum( float val) {
	minLuminance = val;
}

/**
*/
void PictureIO::setMaxLum( float val) {
	maxLuminance = val;
}

/**
*/
void PictureIO::setMappingMethod( LumMappingMethod val) {
	imageMappingMethod = val;	
}

/**
*/
LumMappingMethod PictureIO::getMappingMethod( void) {
	return imageMappingMethod;	
}

/**
*/
unsigned char* PictureIO::getImageData( void) {
	return data;
}








