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
 *
 */
 
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <getopt.h>

#include "glenv.h"
#include "picture_io.h"
#include "histogram.h"
#include "winstat.h"

#define PROG_NAME "pfsGLview"

GLint win, mainMenu, mappingSubmenu, channelSubmenu, mouse=0;		// for holding menu and window ids
PictureIO* bitmap;
Histogram* histogram;
WinStat* winStat;
GLboolean firstTime = true;

bool verbose = false;

enum MenuIds {
	MENU_SEPARATOR,
	MENU_NONE,
	MENU_HISTOGRAM,
	MENU_MONITOR_LUM,
	MENU_MAX_LUM, 
	MENU_EXIT,
	MENU_MAPPING_LINEAR,
	MENU_MAPPING_LOG,
	MENU_MAPPING_GAMMA_1_4,
	MENU_MAPPING_GAMMA_1_8,
	MENU_MAPPING_GAMMA_2_2,
	MENU_MAPPING_GAMMA_2_6,
	MENU_SAVE,
	MENU_CHANNEL_XYZ,
	MENU_CHANNEL_X,
	MENU_CHANNEL_Y,
	MENU_CHANNEL_Z,
	MENU_CHANNEL,
	MENU_INFO,
	MENU_FRAME_NEXT,
	MENU_FRAME_PREVIOUS,
	MENU_HIST_RIGHT,
	MENU_HIST_LEFT,
	MENU_HIST_INCREASE,
	MENU_HIST_DECREASE,
	MENU_ZOOM_IN,
	MENU_ZOOM_OUT,
	MENU_ZOOM_RESET
};

#define MENU_CHANNEL_SHIFT 100

#define INIT_MIN_LUM 0.1f
#define INIT_MAX_LUM 1.0f

#define X_BAR 8
#define Y_BAR 8

#define WINDOW_POS_X 60
#define WINDOW_POS_Y 100
#define WINDOW_SIZE_X 800
#define WINDOW_SIZE_Y 600

#define WINSTAT_WIDTH 300
#define WINSTAT_HEIGHT 45
#define WINSTAT_POS_X 20
#define WINSTAT_POS_Y 10

#define HISTOGRAM_HEIGHT 70 
#define HISTOGRAM_WIDTH 400

#define ZOOM_SCALE_MIN 0.1f
#define ZOOM_SCALE_MAX 20.0f
#define ZOOM_SCALE_STEP 0.005f
#define ZOOM_MOVE_STEP 2.0f

#define MONITOR_LUM_MIN 0.01f
#define MONITOR_LUM_MAX 1.0f

#define SLIDER_STEP 5
#define PAN_STEP 10

void resetWindow( int& size1, int& size2);
void resetHistogram(void);
void loadPicture();
void sliderMoveMax( int shift);
void sliderMoveMin( int shift);
void setWindowTitle( void);

//============================================

struct ZOOM {
	float scale;
	float scale_default;
	int x;
	int y;
	int vX, vY, vSizeX, vSizeY;
	int prevX, prevY;
	bool pan;
} szoom;	

struct VIEWPORT
{
	GLint xPos,yPos;
	GLsizei xSize, ySize;
} viewport;


/**
*/
void zoomReset(void) {
	szoom.scale = szoom.scale_default;
	szoom.x = 0;
	szoom.y = 0;
	szoom.prevX = -1;
	szoom.prevY = -1;
	szoom.pan = false;
}
void zoomIncrease(void) {
	szoom.scale += 0.1f;
}
void zoomDecrease(void) {
	szoom.scale -= 0.1f;
	if(szoom.scale <= 0)
		szoom.scale = 0.1;
}

void redrawWinStat(void) {

	winStat->setMapping( lumMappingName[bitmap->getMappingMethod()]);
	winStat->setMaxFreq( histogram->getMaxFrequency()); 	
	winStat->setFrameNo( bitmap->getFrameNo());
	winStat->setChannel( bitmap->getVisibleChannel());
	winStat->setPixelData( 0, 0, -1, -1, -1);
	winStat->setBZoom( !szoom.pan);	
}
		
void redrawHistogram(void) {

	histogram->setSliderPosMinMax( histogram->lum2pos(bitmap->getLumMin())
						, histogram->lum2pos(bitmap->getLumMax()));
}		
	
/** Change mapping or redraw after changing luminance range.
*/
void changeMapping( float lumMin, float lumMax) {

	// update slider
	histogram->setSliderPosMinMax( histogram->lum2pos(lumMin), histogram->lum2pos(lumMax));
	bitmap->changeMapping( lumMin, lumMax);
}	
	
void sliderMoveMax( int shift) {

	histogram->setSliderPosMax( histogram->getSliderPosMax() + (float)shift / histogram->getWidth());
	bitmap->setMaxLum( histogram->pos2lum(histogram->getSliderPosMax())); 
}	
void sliderMoveMin( int shift) {

	histogram->setSliderPosMin( histogram->getSliderPosMin() + (float)shift / histogram->getWidth());
	bitmap->setMinLum( histogram->pos2lum(histogram->getSliderPosMin())); 
}	
	
	
/** Main display routine
*/
void display(void) {

	glClear(GL_COLOR_BUFFER_BIT);

	// If the picture was loaded we need to rescale it to window size	 
	if(bitmap != NULL) 	{
		
		GLint viewp[4];
		glGetIntegerv( GL_VIEWPORT, viewp);	
		glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
		
		int winWidth = glutGet(GLUT_WINDOW_WIDTH);
		int winHeight = glutGet(GLUT_WINDOW_HEIGHT);
		
		szoom.vX = (int)(winWidth/2 - ((bitmap->getWidth() - szoom.x) * szoom.scale)/2);
		szoom.vY = (int)(winHeight/2 + ((bitmap->getHeight() + szoom.y) * szoom.scale)/2 - HISTOGRAM_HEIGHT/2);
		szoom.vSizeX = (int)(bitmap->getWidth() * szoom.scale);
		szoom.vSizeY = (int)(bitmap->getHeight() * szoom.scale);
		glViewport( szoom.vX, szoom.vY, szoom.vSizeX, szoom.vSizeY);
		
		glEnable( GL_SCISSOR_TEST);
		glScissor( X_BAR/2, Y_BAR/2
				, winWidth - X_BAR, winHeight - Y_BAR - HISTOGRAM_HEIGHT - Y_BAR/2); 
		
		glPixelZoom( szoom.scale, -szoom.scale);	
		glRasterPos2i( 0, 0);

		glDrawPixels(bitmap->getWidth(), bitmap->getHeight(),
			GL_RGBA,  
			GL_UNSIGNED_BYTE, 
			bitmap->getImageData()
			);
		
		glViewport( viewp[0], viewp[1], viewp[2], viewp[3]);			
		glDisable( GL_SCISSOR_TEST);						
	}
	
	// Histogram drawing (and slider)
	if(histogram->getFlag() == GL_TRUE) {
		if(histogram != NULL) {
			histogram->redraw();
		}
	}

	winStat->redraw();

	glMatrixMode(GL_COLOR);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glFlush();
	glutSwapBuffers();
}


/** Resizes main window.
*/
void resizeWindow(int width, int height) {

	// keep aspect of an input bitmap
	float ratio = (float)bitmap->getHeight() / (float)bitmap->getWidth();
	szoom.scale = 1.0f; // default scale coefficient
	if( ratio > 1.0f) { // vertical
		width = (int)((height - Y_BAR - HISTOGRAM_HEIGHT) / ratio);
		if( width < (HISTOGRAM_WIDTH+X_BAR)) {
			width = HISTOGRAM_WIDTH+X_BAR;
		}
		if( (bitmap->getHeight() + Y_BAR + HISTOGRAM_HEIGHT) > height)
			szoom.scale = (float)(height - Y_BAR - HISTOGRAM_HEIGHT) / (float)bitmap->getHeight();
	}
	else { // horizontal
		height = (int)((width + X_BAR) * ratio) + HISTOGRAM_HEIGHT;
		if( (bitmap->getWidth() + X_BAR) > width)
			szoom.scale = (float)(width - X_BAR) / (float)bitmap->getWidth();
	}
	szoom.scale_default = szoom.scale;	

	#ifdef __APPLE__ // "Think different" for OS/X :)
	glutReshapeWindow(width, height);
	#endif

	glViewport(0, 0, width, height);
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0.0f, width, 0.0f, height, -10.0f, 10.0f); // last 1.0 -1.0 it's enough
	
	glMatrixMode(GL_MODELVIEW);
	viewport.ySize = height;
	viewport.xSize = width;
	
	histogram->setWinPos( X_BAR/2, glutGet(GLUT_WINDOW_HEIGHT) - Y_BAR/2 - histogram->getWinHeight());

	glutPostRedisplay();
}

/** Menu listener
*/
void menuListener(int menuId)
{
	int sizeX, sizeY;

	switch(menuId) {
	
		case MENU_ZOOM_IN:
			zoomIncrease();
			break;
		case MENU_ZOOM_OUT:
			zoomDecrease();
			break;
		case MENU_ZOOM_RESET:
			zoomReset();
			break;				
		case MENU_HIST_RIGHT:
			sliderMoveMin( SLIDER_STEP);
			sliderMoveMax( SLIDER_STEP);
			bitmap->changeMapping();
			break;
		case MENU_HIST_LEFT:
			sliderMoveMin( -SLIDER_STEP);
			sliderMoveMax( -SLIDER_STEP);
			bitmap->changeMapping();
			break;
		case MENU_HIST_INCREASE:
			sliderMoveMin( -SLIDER_STEP);
			sliderMoveMax( SLIDER_STEP);
			bitmap->changeMapping();
			break;
		case MENU_HIST_DECREASE:
			sliderMoveMin( SLIDER_STEP);
			sliderMoveMax( -SLIDER_STEP);
			bitmap->changeMapping();
			break;
										
		case MENU_FRAME_NEXT: 
			bitmap->gotoNextFrame(); 
			resetWindow( sizeX, sizeY);
			setWindowTitle();
			resetHistogram();
			redrawHistogram();
			redrawWinStat();
			resizeWindow( sizeX, sizeY);
			break;
			
		case MENU_FRAME_PREVIOUS: 
			bitmap->gotoPreviousFrame(); 
			resetWindow( sizeX, sizeY);
			setWindowTitle();
			resetHistogram();
			redrawHistogram();
			redrawWinStat();
			resizeWindow( sizeX, sizeY);
			break;

		case MENU_HISTOGRAM :
			histogram->setFlag( !histogram->getFlag());
			break;

		case MENU_INFO:
			winStat->setFlag( !winStat->getFlag());
			break;
			
		case MENU_MONITOR_LUM :	
			bitmap->changeMapping( MONITOR_LUM_MIN, MONITOR_LUM_MAX);
			redrawHistogram();
			break;
			
		case MENU_MAX_LUM :
			float min, max;
			min = bitmap->computeLumMin();
			max = bitmap->computeLumMax();
			bitmap->changeMapping( min, max);
			redrawHistogram();
			break;

		case MENU_MAPPING_LINEAR: 
			bitmap->changeMapping( MAP_LINEAR);
			redrawWinStat();
			break;
		case MENU_MAPPING_GAMMA_1_4: 
			bitmap->changeMapping( MAP_GAMMA1_4);
			redrawWinStat();
			break;
		case MENU_MAPPING_GAMMA_1_8: 
			bitmap->changeMapping( MAP_GAMMA1_8);
			redrawWinStat();
			break;
		case MENU_MAPPING_GAMMA_2_2: 
			bitmap->changeMapping( MAP_GAMMA2_2);
			redrawWinStat();
			break;
		case MENU_MAPPING_GAMMA_2_6: 
			bitmap->changeMapping( MAP_GAMMA2_6);
			redrawWinStat();
			break;
		case MENU_MAPPING_LOG: 
			bitmap->changeMapping( MAP_LOGARITHMIC);
			redrawWinStat();
			break;
			
		case MENU_CHANNEL_XYZ: 
			bitmap->setVisibleChannel( bitmap->CHANNEL_XYZ);
			bitmap->changeMapping();
			resetHistogram();
			redrawHistogram();
			redrawWinStat();
			break;		
		case MENU_CHANNEL_X: 
			bitmap->setVisibleChannel( bitmap->CHANNEL_X);
			bitmap->changeMapping();
			resetHistogram();
			redrawHistogram();
			redrawWinStat();
			break;		
		case MENU_CHANNEL_Y: 
			bitmap->setVisibleChannel( bitmap->CHANNEL_Y);
			bitmap->changeMapping();
			resetHistogram();
			redrawHistogram();
			redrawWinStat();
			break;		
		case MENU_CHANNEL_Z: 
			bitmap->setVisibleChannel( bitmap->CHANNEL_Z);
			bitmap->changeMapping();
			resetHistogram();
			redrawHistogram();
			redrawWinStat();
			break;	
		
		case MENU_SAVE: // save current bitmap data (8-bits RGB) to stdout
			bitmap->save();
			exit(0);
			break;
					
		case MENU_EXIT: 
			exit(0); 
			break;
			
	}
	glutPostRedisplay();
}


/** Mouse listener
*/
int cursorPosX = -1;

void mouseListener(int button, int state, int x, int y)
{
	cursorPosX = x; // required for mouse motion listener

	// trig slider
	if( histogram->getFlag() == GL_TRUE && button == GLUT_LEFT_BUTTON 
		&& state == GLUT_DOWN && histogram->getSliderSelectionState() == NONE) {
		    
		histogram->processSliderSelection(x, y);
	}
	
	// slider was moved, the bitmap should be recalculated
	if( histogram->getFlag() == GL_TRUE && button == GLUT_LEFT_BUTTON && state == GLUT_UP 
		&& histogram->getSliderSelectionState() != NONE) {
		
		bitmap->changeMapping();
		histogram->setSliderSelectionState(NONE);
	}
	
	// reset cursor position (for zooming)	
	if(button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
	
		szoom.prevX = -1;
		szoom.prevY = -1;
	
		glutSetCursor(GLUT_CURSOR_INHERIT);
	}	
	
	if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
		if( winStat->processSelection( x, y))
			szoom.pan = !szoom.pan;
			redrawWinStat();
	}	
		
	glutPostRedisplay();
}

void mouseMotionListener(int x, int y) {
	
	if(histogram->getFlag() == GL_TRUE) // slider
	{
		int selectionState = histogram->getSliderSelectionState();
		switch(selectionState)
		{
			case LEFT_BAR: 	
				
				sliderMoveMin( x - cursorPosX);
				cursorPosX = x;
				
				glutSetCursor(GLUT_CURSOR_LEFT_SIDE);
				break;
				
			case RIGHT_BAR:	

				sliderMoveMax( x - cursorPosX);
				cursorPosX = x;

				glutSetCursor(GLUT_CURSOR_RIGHT_SIDE);
				break;
				
			case WHOLE_SLIDER: 
					
				sliderMoveMin( x - cursorPosX);
				sliderMoveMax( x - cursorPosX);				
				cursorPosX = x;	

				glutSetCursor(GLUT_CURSOR_LEFT_RIGHT);
			
				break;
		}
	}
	
	// zooming 
	if( y > (HISTOGRAM_HEIGHT + Y_BAR)) {
	
		if( szoom.prevX != -1){
			if( szoom.pan) {
				float dd = ZOOM_MOVE_STEP / szoom.scale;
				szoom.x += (int)((x - szoom.prevX) * dd);
				szoom.y += (int)((szoom.prevY - y) * dd);
			}
			else {
				float dd = ZOOM_SCALE_STEP * szoom.scale;
				szoom.scale += ( szoom.prevY - y) * dd;

				if( szoom.scale < ZOOM_SCALE_MIN)
					szoom.scale = ZOOM_SCALE_MIN;
				if( szoom.scale > ZOOM_SCALE_MAX)
					szoom.scale = ZOOM_SCALE_MAX;	
			}	
		}	
		szoom.prevX = x;
		szoom.prevY = y;
	}	
	
	glutPostRedisplay();
}


/** Mouse passive listener.
*/
void mousePassiveMotionListener(int x, int y) {

	// show bitmap pixel position and value
	int xPos, yPos;
	xPos = x - szoom.vX;
	yPos = y - (glutGet(GLUT_WINDOW_HEIGHT) - szoom.vY) - 3;
	
	int xx = (int)( xPos / szoom.scale);
	int yy = (int)( yPos / szoom.scale);
	
	float X, Y, Z;
	if(!bitmap->getRawData( xx, yy, X, Y, Z))
		winStat->setRawData( xx, yy, X, Y, Z);
	else	
		winStat->setRawData( -1, 0, 0, 0, 0);
	// show screen (display) pixel position and value
	float val;
	glReadPixels( x, glutGet(GLUT_WINDOW_HEIGHT) - y, 1, 1, GL_RED, GL_FLOAT, &val);
	int r = (int)(val * 255);
	glReadPixels( x, glutGet(GLUT_WINDOW_HEIGHT) - y, 1, 1, GL_GREEN, GL_FLOAT, &val);
	int g = (int)(val * 255);
	glReadPixels( x, glutGet(GLUT_WINDOW_HEIGHT) - y, 1, 1, GL_BLUE, GL_FLOAT, &val);
	int b = (int)(val * 255);
	winStat->setPixelData( x, y, r, g, b);	
	
	glutPostRedisplay();
}


/** Keyboard listener.
*/
void keyListener(unsigned char key, int x, int y) {

	int menuFuncId = 0;
	switch (key) {
		case '.': menuFuncId = MENU_ZOOM_IN; break;
		case ',': menuFuncId = MENU_ZOOM_OUT; break;
		case 'r': 
		case 'R':
			menuFuncId = MENU_ZOOM_RESET; break;		
		case '=': 
			menuFuncId = MENU_HIST_RIGHT;break;
		case '-': menuFuncId = MENU_HIST_LEFT; break;
		case ']': menuFuncId = MENU_HIST_INCREASE; break;
		case '[': menuFuncId = MENU_HIST_DECREASE; break;
		case 'h': menuFuncId = MENU_HISTOGRAM; break;
		case 'i': menuFuncId = MENU_INFO; break;
		case 12: menuFuncId = MENU_MONITOR_LUM; break;
		case '\\': menuFuncId = MENU_MAX_LUM; break;
		case '1': menuFuncId = MENU_MAPPING_GAMMA_1_4; break;
		case '2': menuFuncId = MENU_MAPPING_GAMMA_1_8; break;
		case '3': menuFuncId = MENU_MAPPING_GAMMA_2_2; break;
		case '4': menuFuncId = MENU_MAPPING_GAMMA_2_6; break;
		case 'L':
		case 'l': menuFuncId = MENU_MAPPING_LINEAR; break;
		case 'O':
		case 'o': menuFuncId = MENU_MAPPING_LOG; break;
		case 'n': menuFuncId = MENU_FRAME_NEXT; break;	
		case 'p': menuFuncId = MENU_FRAME_PREVIOUS; break;	
		case ' ': 
			szoom.pan = !szoom.pan;
			redrawWinStat();
			szoom.prevX = -1;
			szoom.prevY = -1;
			menuFuncId = MENU_NONE;
			break;
		case 'q':
		case 'Q':
		case 27: exit(0); break;
	}
	menuListener(menuFuncId);
}
void specialKeyListener(int key, int x, int y) {

	switch (key) {
		case GLUT_KEY_LEFT : 	
			szoom.x += PAN_STEP;
			glutPostRedisplay();
			break;
			
		case GLUT_KEY_RIGHT: 
			szoom.x -= PAN_STEP;
			glutPostRedisplay();
			break;
			
		case GLUT_KEY_UP:
			szoom.y -= PAN_STEP;
			glutPostRedisplay();	
			break;
			
		case GLUT_KEY_DOWN :
			szoom.y += PAN_STEP;
			glutPostRedisplay();	
			break;
	}	
	//glutPostRedisplay();	
	
}


/** Loads the bitmap from disk.  Display an error message if it doesn't load...
*/
void loadPicture() {

	bitmap = new PictureIO(MAP_GAMMA2_2, INIT_MIN_LUM, INIT_MAX_LUM);
}


/**
*/
void resetHistogram(void) {

	// histogram
	histogram->resetFrequencyMax();
	histogram->computeFrequency(bitmap->getPrimaryChannel());
	
	float min, max;
	histogram->computeLumRange( min, max); // starting luminance range on the slider (computed based on the histogram shape)
	
	// mapping method
	bitmap->changeMapping( min, max);
	//bitmap->getDynamicRange();

}



/** Adjusting viewport size to image resolution.
*/
void resetWindow( int& size1, int& size2) {

	size1 = WINDOW_SIZE_X;
	size2 = WINDOW_SIZE_Y;
	
	if(bitmap->getWidth() == 0)
		return;
		
	float ratio = (float)bitmap->getHeight() / (float)bitmap->getWidth();
	
	if( ratio < 1) { // horizontal
	
		if( bitmap->getWidth() > WINDOW_SIZE_X) {
			
			size1 = WINDOW_SIZE_X + X_BAR;
			size2 = (int)(WINDOW_SIZE_X * ratio)  + Y_BAR ;
		}
		else {
		
			if( bitmap->getWidth() < histogram->getBackgroundWidth()) { // scale up
			
				size1 = histogram->getBackgroundWidth() + X_BAR;
				size2 = (int)(histogram->getBackgroundWidth() * ratio)  + Y_BAR;
			}
			else {
				size1 = bitmap->getWidth() + X_BAR;
				size2 = (int)(bitmap->getWidth() * ratio)  + Y_BAR;			
			}
		}
	}
	else { // vertical
		if( bitmap->getHeight() > WINDOW_SIZE_Y) {
			
			size1 = (int)(WINDOW_SIZE_Y * 1.0 / ratio)  + X_BAR;
			size2 = WINDOW_SIZE_Y + Y_BAR;
		}
		else {
			float sizeX = bitmap->getHeight() * 1.0 / ratio;
			
			if( sizeX < histogram->getBackgroundWidth()) { // scale up
			
				size1 = (int)(WINDOW_SIZE_Y * 1.0 / ratio)  + X_BAR;
				size2 = WINDOW_SIZE_Y + Y_BAR;
			}
			else {
				size1 = (int)(bitmap->getHeight() * 1.0 / ratio)  + X_BAR;
				size2 = bitmap->getHeight() + Y_BAR;
			}
		}	
	}
}	

/** Changes main window title.
*/
void setWindowTitle( void) {

	// set window title
	char title[2000];
	sprintf(title, "PFS GLview v.1.2     %s   %dx%d", bitmap->getCurrentFileName(), bitmap->getWidth(), bitmap->getHeight());
	glutSetWindowTitle(title);
}


/** Parses command line.
*/
int pfsglview( int argc, char* argv[]) {

	static struct option cmdLineOptions[] = {
		{ "help", no_argument, NULL, 'h' },
		{ "verbose", no_argument, NULL, 'v' },
		{ NULL, 0, NULL, 0 }
	};

	int optionIndex = 0;
	while( 1 ) {
		int c = getopt_long (argc, argv, "hv", cmdLineOptions, &optionIndex);
		if( c == -1 ) break;
		switch( c ) {
			case 'h':
				fprintf( stderr, "pfsglview [--verbose] [--help]\nHigh dynamic range image viewer. Use within pfstools pipe (e.g. pfsin image.hdr | pfsglview).\n");
				return 1;
				
			case 'v':
				verbose = true;
				break;
		}
	}

	return 0;
}

/** Main routine.
*/
int main( int argc, char* argv[] )
{
	
	if( pfsglview( argc, argv ))
		return 1;

	loadPicture();
	histogram = new Histogram(X_BAR, viewport.ySize-Y_BAR/2, HISTOGRAM_WIDTH, HISTOGRAM_HEIGHT);
	winStat = new WinStat();
	zoomReset();

	glutInit(&argc, argv);
	glutInitWindowPosition( WINDOW_POS_X, WINDOW_POS_Y);

	//int sizeX, sizeY;
	//resetWindow( sizeX, sizeY);
	//glutInitWindowSize( sizeX, sizeY);
		
	glutInitWindowSize( WINDOW_SIZE_X, WINDOW_SIZE_Y);	
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);

	win = glutCreateWindow("PFS GLview v.1.2.1");

	histogram->setWinPos( X_BAR/2, glutGet(GLUT_WINDOW_HEIGHT) - Y_BAR/2 - histogram->getWinHeight());
	histogram->setWinSize( HISTOGRAM_WIDTH, HISTOGRAM_HEIGHT);	
	
	winStat->setWinPos( WINSTAT_POS_X, WINSTAT_POS_Y);
	winStat->setWinSize( WINSTAT_WIDTH, WINSTAT_HEIGHT);
	
	setWindowTitle();
	resetHistogram(); // compute histogram for a given image
	redrawHistogram();
	redrawWinStat();
	
	// popup menu
	mappingSubmenu = glutCreateMenu(menuListener);
	glutAddMenuEntry("Gamma 1.4 (1)", MENU_MAPPING_GAMMA_1_4);
	glutAddMenuEntry("Gamma 1.8 (2)", MENU_MAPPING_GAMMA_1_8);
	glutAddMenuEntry("Gamma 2.2 (3)", MENU_MAPPING_GAMMA_2_2);
	glutAddMenuEntry("Gamma 2.6 (4)", MENU_MAPPING_GAMMA_2_6);
	glutAddMenuEntry("Linear (L)", MENU_MAPPING_LINEAR);	
	glutAddMenuEntry("Logarithmic (O)", MENU_MAPPING_LOG);
	//glutAddMenuEntry("Reinhard", 17);
	
	channelSubmenu = glutCreateMenu(menuListener);
	glutAddMenuEntry("XYZ", MENU_CHANNEL_XYZ);
	
	//std::vector<const char*> vec = bitmap->getChannelNames();
	//for(int i = 0; i < vec.size(); i ++)
	//	glutAddMenuEntry(vec[i], MENU_CHANNEL+MENU_CHANNEL_SHIFT+i);
	
	glutAddMenuEntry("X", MENU_CHANNEL_X);	
	glutAddMenuEntry("Y", MENU_CHANNEL_Y);	
	glutAddMenuEntry("Z", MENU_CHANNEL_Z);
	
	mainMenu = glutCreateMenu(menuListener);

	glutAddMenuEntry("Zoom reset (r)", MENU_ZOOM_RESET);
	glutAddMenuEntry("Zoom in (.)", MENU_ZOOM_IN);
	glutAddMenuEntry("Zoom out (,)", MENU_ZOOM_OUT);
	glutAddMenuEntry("",MENU_SEPARATOR);
	glutAddMenuEntry("Increase exposure (=)", MENU_HIST_RIGHT);
	glutAddMenuEntry("Decrease exposure (-)", MENU_HIST_LEFT);
	glutAddMenuEntry("Extend dynamic range (])", MENU_HIST_INCREASE);
	glutAddMenuEntry("Shrink dynamic range ([)", MENU_HIST_DECREASE);
	glutAddMenuEntry("Low dynamic range (Ctrl-L)", MENU_MONITOR_LUM);
	glutAddMenuEntry("Fit to dynamic range (\\)", MENU_MAX_LUM);
	glutAddMenuEntry("",MENU_SEPARATOR);
	glutAddSubMenu("Choose channel", channelSubmenu);
	glutAddSubMenu("Mapping method", mappingSubmenu);
	glutAddMenuEntry("",MENU_SEPARATOR);
	glutAddMenuEntry("Next frame (n)", MENU_FRAME_NEXT);
	glutAddMenuEntry("Previous frame (p)", MENU_FRAME_PREVIOUS);	
	glutAddMenuEntry("",MENU_SEPARATOR);
	glutAddMenuEntry("Histogram (h)", MENU_HISTOGRAM);
	glutAddMenuEntry("Info (i)", MENU_INFO);
	glutAddMenuEntry("",MENU_SEPARATOR);	
	glutAddMenuEntry("Save&Quit", MENU_SAVE);
	glutAddMenuEntry("Quit (Q or Esc)", MENU_EXIT);

	glutAttachMenu(GLUT_RIGHT_BUTTON);
	
	// viewport
	GLint viewportSize[4];
	glGetIntegerv(GL_VIEWPORT, viewportSize);

	viewport.xPos = viewportSize[0];
	viewport.yPos = viewportSize[1];
	viewport.xSize = viewportSize[2];
	viewport.ySize = viewportSize[3];

	// mouse listeners
 	glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
	glutMouseFunc(mouseListener);
	glutMotionFunc(mouseMotionListener);
	glutPassiveMotionFunc( mousePassiveMotionListener);
	
	// keyboard listeners
	glutKeyboardFunc(keyListener);
	glutSpecialFunc(specialKeyListener);
	
	glutDisplayFunc(display);
	glutReshapeFunc(resizeWindow);
	glutMainLoop();
	
	return 0;   
}
