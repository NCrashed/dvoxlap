/***************************************************************************************************
WINMAIN.CPP & SYSMAIN.H

Windows layer code written by Ken Silverman (http://advsys.net/ken) (1997-2009)
Additional modifications by Tom Dobrowolski (http://ged.ax.pl/~tomkh)
You may use this code for non-commercial purposes as long as credit is maintained.
***************************************************************************************************/
module sysmain;

import core.sys.windows.windows;

version(Windows)
{
	extern(C)
	{
		HWND ghwnd;
		HINSTANCE ghinst;
		HDC startdc();
		void stopdc();
		void ddflip2gid();
		void setalwaysactive(long);
		void setacquire(long mouse, long kbd);
		void smartsleep(long timeoutms);
		long canrender();
		long ddrawuseemulation;
		long ddrawdebugmode; // -1 = 0ff, old ddrawuseemulation = on
		void debugdirectdraw(); // toggle debug mode
		alias extern(C) long function(HWND, UINT, WPARAM, LPARAM) peekwindowproc;
	}
} else
{
	align(1) struct PALETTEENTRY
	{
		ubyte peRed, peGreen, peBlue, peFlags;
	}

	enum MAX_PATH = 260;
}



extern(C) long cputype;

// Program flow:
extern(C) char *prognam;
extern(C) long progresiz;

long initapp(long argc, char **argv);
void doframe();

extern(C) void quitloop();
void uninitapp();

// Video:
struct validmodetype
{
	long x, y;
	byte c, r0, g0, b0, a0, rn, gn, bn, an;
}

extern(C) validmodetype curvidmodeinfo;
extern(C) long xres, yres, colbits, fullscreen, maxpages;
extern(C) PALETTEENTRY pal[256];
extern(C) long startdirectdraw(long *, long *, long *, long *);
extern(C) void stopdirectdraw();
extern(C) void nextpage();
extern(C) long clearscreen(long fillcolor);
extern(C) void updatepalette(long start, long danum);
extern(C) long getvalidmodelist(validmodetype **validmodelist);
extern(C) long changeres(long, long, long, long);

// Sound:
enum KSND_3D 		= 1; // Use logical or to combine flags 
enum KSND_MOVE 		= 2;
enum KSND_LOOP 		= 4;
enum KSND_LOPASS 	= 8;
enum KSND_MEM 		= 16;

extern(C) void playsound(const char*, long, float, void *, long);
extern(C) void playsoundupdate(void *, void *);
extern(C) void setears3d(float, float, float, float, float, float, float, float, float);
extern(C) void setvolume(long);
extern(C) long umixerstart(void function(void *, long), long, long, long);
extern(C) void umixerkill(long);
extern(C) void umixerbreathe();

// Keyboard:
extern(C) char keystatus[256]; 		//bit0=1:down
extern(C) char ext_keystatus[256];	//bit0=1:down,bit1=1:was down
extern(C) long readkeyboard();
extern(C) long keyread(); 			//similar to kbhit()&getch() combined

// Mouse:
extern(C) char ext_mbstatus[8]; 	//bit0=1:down,bit1=1:was down, bits6&7=mwheel
extern(C) long ext_mwheel;
extern(C) long mousmoth;			//1:mouse smoothing (default), 0 otherwise
extern(C) float mousper;			//Esimated mouse interrupt rate

extern(C) void readmouse(float *, float *, long *);
extern(C) void readmouse(float *, float *, float*, long *);

extern(C) long ismouseout(long, long);
extern(C) void setmouseout(void function(long, long), long, long);

// Timer:
extern(C) void readklock(double *);
