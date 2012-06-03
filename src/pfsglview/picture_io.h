#ifndef PICTUREIO_H
#define PICTUREIO_H

#include <list>
#include <pfs.h>

#define MAX_FRAMES_IN_MEMORY 10

class pfs::Frame;
enum LumMappingMethod {
			MAP_LINEAR,
			MAP_GAMMA1_4,
			MAP_GAMMA1_8,
			MAP_GAMMA2_2,
			MAP_GAMMA2_6,
			MAP_LOGARITHMIC,
			MAP_REINHARD
			};
			
static const char* lumMappingName[] = { // must be consistent with LumMappingMethod
			"Linear",
			"Gamma 1.4",
			"Gamma 1.8",
			"Gamma 2.2",
			"Gamma 2.6",
			"Logarithmic",
			"Photographic"
			};


class PictureIO
{
	private:
		pfs::Frame *pfsFrame;
		int width;
		int height;
		const char *currentFileName;
		const char *visibleChannel;
		pfs::Array2D *chR, *chG, *chB;
		std::list<pfs::Frame*> frameList;
		std::list<pfs::Frame*>::iterator currentFrame; 
		int frameNo;		

		unsigned char* data;
		LumMappingMethod imageMappingMethod;
		float minLuminance, maxLuminance;		

	public:	
		const char* CHANNEL_XYZ;
		const char* CHANNEL_X;
		const char* CHANNEL_Y;
		const char* CHANNEL_Z;	
	
		PictureIO(LumMappingMethod mappingMethod, float minLuminance, float maxLumuminance);
		~PictureIO();
		void gotoNextFrame();
		void gotoPreviousFrame();

		void changeMapping( LumMappingMethod mappingMethod, float minLum, float maxLum);
		void changeMapping( float minLum, float maxLum);
		void changeMapping( LumMappingMethod mappingMethod);
		void changeMapping( void);
		
		void setMinLum( float val);
		void setMaxLum( float val);
		
		unsigned char* getImageData( void);
		
		void setMappingMethod( LumMappingMethod val);
		LumMappingMethod getMappingMethod( void);
		
		const pfs::Array2D* getPrimaryChannel();
		int loadPicture(const char* location, int width, int height);
		const char *getCurrentFileName();
		const char *getVisibleChannel();
		void setVisibleChannel(const char *channel);
		pfs::Frame *getFrame();
		void setFrame(pfs::Frame *pfsFrame, const char *channel);
		
		int save(void);
		
		int getWidth();
		int getHeight();		
		int getPixel(int ch, int c, int r);
		int getPixelR(int x, int y);
		int getPixelG(int x, int y);
		int getPixelB(int x, int y);
		float getLumMin(void);
		float getLumMax(void);
		float computeLumMin(void);
		float computeLumMax(void);
		int getFrameNo(void);
		float getDynamicRange(void);
		std::vector<const char*> getChannelNames();
		
		int getRawData( int x, int y, float& XX, float& YY, float& ZZ);
		
	private:
		bool hasColorChannels( pfs::Frame *frame );
		void updateMapping( void);
		inline int binarySearchPixels( float lum, const float *lumMap, const int lumSize);
		float getInverseMapping( LumMappingMethod mappingMethod, float v, float minValue, float maxValue );
		bool readNextFrame();
};

class PFSglViewException
{
};
#endif

