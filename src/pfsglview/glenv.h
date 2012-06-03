
#ifndef __GLENV_H
#define __GLENV_H

#ifdef __APPLE__ // "Think different" for OS/X :)
#include "GLUT/glut.h"
#include "OPENGL/gl.h"
#include "OPENGL/glu.h"
#include "OPENGL/glext.h"
#else
#include "GL/glut.h"
#include "GL/gl.h"
#include "GL/glu.h"
#include "GL/glext.h"
#endif

#endif
