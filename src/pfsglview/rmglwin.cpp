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

#include<stdio.h>

#include "glenv.h"
#include "rmglwin.h"

RMGLWin::RMGLWin() {

	flag = GL_TRUE;
	winPosX = 10;
	winPosY = 10;
	winWidth = 100;
	winHeight = 40;

	// background color
	winBackgroundColor = new float[4];
	for(int i=0; i<3; i++)
		winBackgroundColor[i] = 1.0f;
	winBackgroundColor[3] = 0.8f;	
}


RMGLWin::~RMGLWin() {

	delete [] winBackgroundColor;
}	

void RMGLWin::redraw(void) {

	if( flag == GL_FALSE)
		return;

	// draw background
	glEnable(GL_BLEND);
	glColor4fv(winBackgroundColor);
	glRecti( 0, 0, winWidth, winHeight); 	
	glDisable(GL_BLEND);
}	

int RMGLWin::redrawStart(void) {

	if( flag == GL_FALSE)
		return 1;

	glGetIntegerv( GL_VIEWPORT, param); // remember old vieport
	
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	//glLoadIdentity();
		
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	
	glViewport( winPosX, winPosY, winWidth, winHeight);
	glOrtho( 0, winWidth, 0, winHeight, -1.0f, 1.0f); 
	
	redraw();
	
	return 0;
}

void RMGLWin::redrawEnd(void) {

	if( flag == GL_FALSE)
		return;

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glViewport( param[0], param[1], param[2], param[3]); // restore viewport
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();	
}

void RMGLWin::setWinPos( int posX, int posY) {
	winPosX = posX;
	winPosY = posY;
}

void RMGLWin::setWinSize( int width, int height) {
	winWidth = width;
	winHeight = height;
}

int RMGLWin::getWinHeight() {
	return winHeight;
}


void RMGLWin::setFlag(int bb) {

	flag = bb;
}

int RMGLWin::getFlag(void) {
	return flag;
}

int RMGLWin::processSelection(int xCoord, int yCoord) {

	if( flag == GL_FALSE)
		return 1;

	redrawStart();

	// Hits counter and viewport martix
	GLint hits, viewp[4];
	// Get actual viewport
	glGetIntegerv(GL_VIEWPORT, viewp);
	
	#define BUFFER_SIZE 64
	// Table for selection buffer data
	GLuint selectionBuffer[BUFFER_SIZE];
	// Prepare selection buffer
	glSelectBuffer(BUFFER_SIZE, selectionBuffer);
	
	// Change rendering mode
	glRenderMode(GL_SELECT);
	// Initializes the Name Stack
	glInitNames();
	// Push 0 (at least one entry) Onto the Stack
	glPushName(0);
	// Set new projection matrix as a box around xPos, yPos
	glLoadIdentity();
		
	int hh = glutGet(GLUT_WINDOW_HEIGHT);
	// Picking matrix at position xCoord, windowSize - yCoord (fliped window Y axis)
	// and size of 4 units in depth
	gluPickMatrix(xCoord, hh - yCoord, 4, 4, viewp);

	glOrtho(0.0f, viewp[2], 0.0f, viewp[3], -10.0f, 10.0f); // last 1.0 -1.0 it's enough
	redraw(); // draw only picked parts
	
	int ret = 0;
	
	hits = glRenderMode(GL_RENDER);

	if(hits > 0) {
		ret = 1;
	}
		
	redrawEnd();
	
	return ret;
}









