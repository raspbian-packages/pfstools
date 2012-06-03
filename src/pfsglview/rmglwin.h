
#ifndef __RMGLWIN_H
#define __RMGLWIN_H

class RMGLWin  {
	
	private:
		GLint param[4];
	
	protected:
		int flag;
		int winPosX;
		int winPosY;
		int winWidth;
		int winHeight;
		GLfloat* winBackgroundColor;

		void redraw(void);				
		int redrawStart(void);
		void redrawEnd(void);				
						
	public:
	
		RMGLWin();
		~RMGLWin();

		void setWinPos( int posX, int posY);
		void setWinSize( int width, int height);
		
		int getWinHeight();
		void setFlag(int bb);
		int getFlag(void);
		
		int processSelection(int xCoord, int yCoord);
};


#endif
