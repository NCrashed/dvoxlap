//based on VOXLAP engine by Ken Silverman (http://advsys.net/ken)
module voxlap5;

import std.c.string;

enum MAXXDIM = 1024;
enum MAXYDIM = 768;
enum PI = 3.141592653589793;
/// Maximum .VXL dimensions in both x & y direction
enum VSID = 1024; 
/// Maximum .VXL dimensions in z direction (height)  
enum MAXZDIM = 256;

const SIZET = size_t.sizeof;

align(1) struct lpoint3d
{
	int x, y, z;
}
align(1) struct point3d
{
	float x, y, z;
}
align(1) struct point4d
{
	float x, y, z, z2;
}
align(1) struct dpoint3d
{
	double x, y, z;
}

// Sprite structures:
align(1) struct kv6voxtype
{
	int col;
	ushort z;
	ubyte vis, dir;
}

align(1) struct kv6data
{
	int leng, xsiz, ysiz, zsiz;
	float xpiv, ypiv, zpiv;
	uint numvoxs;
	int namoff;
	align(4) kv6data *lowermip; // to avoid gc problems
	align(4) kv6voxtype *vox; // numvoxs*sizeof(kv6voxtype)
	uint *xlen; // xsiz*sizeof(int)
	ushort *ylen; // xsiz*ysiz*sizeof(short)
}

align(1) struct hingertype
{
	/// index to parent sprite (-1 = none)
	int parent;
	/// "velcro" point of each object
	point3d p[2];
	/// axis of rotation for each object
	point3d v[2];
	/// min value / max value
	short vmin, vmax;
	ubyte htype;
	ubyte filler[7];
}

align(1) struct seqtyp
{
	int tim, frm;
}

/**
*	Notice that I aligned each point3d on a 16-byte boundary. This will be
*	helpful when I get around to implementing SSE instructions someday...
*/
align(1) struct vx5sprite
{
	/// position in VXL coordinates
	point3d p;
	/**
	*	flags bit 0:0 = use normal shading, 1 = disable normal shading
	*	flags bit 1:0 = points to kv6data, 1 = points to kfatype
	*	flags bit 2:0 = normal, 1 = invisible sprite
	*/
	int flags;
	/// kv6data.xsiz direction in VXL coordinates
	static union { point3d s, x; };
	static union
	{
		/// pointer to KV6 voxel data (bit 1 of flags = 0)
		kv6data *voxnum;
		/// pointer to KFA animation (bit 1 of flags = 1)
		kfatype *kfaptr;
	};
	/// kv6data.ysiz direction in VXL coordinates
	static union { point3d h, y; };
	/// time (in miilliseconds) of KFA animation
	int kfatim;
	/// kv6data.zsiz direction in VXL coordinates
	static union {point3d f, z; };
	/// make vx5sprite exactly 64 bytes :)
	int okfatim;
}

align(1) struct kfatype
{
	int numspr, numhin, numfrm, seqnum;
	int namoff;
	/// Points to original unconnected KV6 (maybe helpful?)
	align(4) kv6data *basekv6;
	align(4) vx5sprite *spr; 		// [numspr]
	align(4) hingertype *hinge;		// [numhin]
	align(4) int *hingesort;		// [numhin]
	align(4) short *frmval;			// [numfrm][numhin]
	align(4) seqtyp *seq;			// [seqnum]
}

/**
*	Falling voxels shared data: (flst = float list)
*	Max # of separate falling pieces
*/
enum FLPIECES = 256;

align(1) struct flstboxtype //(68 bytes)
{
	/// a solid point on piece (x,y,pointer) (don't touch!)
	lpoint3d chk; 
	/// indices to start&end of slab list (don't touch!)
	int i0, i1;
	/// bounding box, written bt startfalls
	int x0, y0, z0, x1, y1, z1;
	/// mass of piece, written by startfalls (1 unit per voxel)
	int mass;
	/// centroid of piece, written by startfalls
	point3d centroid;

	/**
	*	userval is set to -1 when a new piece is spawned. Voxlap does not
	*	read or write these values after that point. You should use these to
	*	play an initial sound and track velocity
	*/
	int userval, userval2;
}

/// Lighting variables: (used by updatelighting)
enum MAXLIGHTS = 256;
align(1) struct lightsrctype
{
	point3d p; float r2, sc;
}

/// Used by setspans/meltspans. Ordered this way to allow sorting as longs!
align(1) struct vspans
{
	ubyte z1, z0, x, y;
}

/// MUST be even number for alignment!
enum MAXFRM = 1024;

struct vx5_type
{
	//------------------------ DATA coming from VOXLAP5 ------------------------

	/** 
	*	Clipmove hit point info (use this after calling clipmove):
	*	clipmove always calls findmaxcr even with no movement
	*/
	double clipmaxcr;
	dpoint3d cliphit[3];
	int cliphitnum;

	/// Bounding box written by last set* VXL writing call
	int minx, miny, minz, maxx, maxy, maxz;

	/// Falling voxels shared data:
	int flstnum;
	flstboxtype flscnt[FLPIECES];

	/// Total count of solid voxels in .VXL map (included unexposed voxels)
	int globalmass;

	/**
	*	Temp workspace for KFA animation (hinge angles)
	*	Animsprite writes these values and you modify them before drawsprite
	*/
	short kfaval[MAXFRM];

	//------------------------ DATA provided to VOXLAP5 ------------------------

	/// Opticast variables:
	int anginc, sideshademode, mipscandist, maxscandist, vxlminpuse, fogcol;

	/// Drawsprite variables:
	int kv6mipfactor, kv6col;

	/**
	*	Drawsprite x-plane clipping (reset to 0, (high int) after use!)
	*	For example min=8, max=12 permits only planes 8,9,10,11 to draw
	*/
	int xplanemin, xplanemax;

	/// Map modification function data:
	int curcol, currad, curhei;
	float curpow;

	/// Procedural texture function data:
	alias int function(lpoint3d *) colfunc_t;
	colfunc_t colfunc;
	int cen, amount, bpl, xsiz, ysiz, xoru, xorv, picmode;
	int *pic;
	point3d fpico, fpicu, fpicv, fpicw;
	lpoint3d pico, picu, picv;
	float daf;

	/**
	*	Lighting variables: (used by updatelighting)
	*	0 (default), 1:simple lighting, 2:lightsrc lighting
	*/
	int lightmode;
	/// (?,?,?),128*128,262144
	lightsrctype lightsrc[MAXLIGHTS];
	int numlights;

	int fallcheck;
} 
vx5_type vx5;

/// Initialization functions:
/*extern(C) int initvoxlap();

extern(C) void uninitvoxlap();

/// File related functions:
extern(C) int loadsxl(const ubyte* , ubyte**, ubyte**, ubyte**);
extern(C) ubyte* parspr(vx5sprite*, ubyte**);
extern(C) void loadnul(dpoint3d *, dpoint3d *, dpoint3d *, dpoint3d *);
extern(C) int loaddta(const ubyte*, dpoint3d*, dpoint3d*, dpoint3d*, dpoint3d*);
extern(C) int loadpng(const ubyte*, dpoint3d*, dpoint3d*, dpoint3d*, dpoint3d*);
extern(C) void loadbsp(const ubyte*, dpoint3d*, dpoint3d*, dpoint3d*, dpoint3d*);
extern(C) int loadvxl(const ubyte*, dpoint3d*, dpoint3d*, dpoint3d*, dpoint3d*);
extern(C) int savevxl(const ubyte*, dpoint3d*, dpoint3d*, dpoint3d*, dpoint3d*);
extern(C) int loadsky(const ubyte*);

/// Screen related functions:
extern(C) void voxsetframebuffer(int, int, int, int);
extern(C) void setsideshades(ubyte, ubyte, ubyte, ubyte, ubyte, ubyte);
extern(C) void setcamera(dpoint3d*, dpoint3d*, dpoint3d*, dpoint3d*, float, float, float);
extern(C) void opticast();
extern(C) void drawpoint2d(int, int, int);
extern(C) void drawpoint3d(float, float, float, int);
extern(C) void drawline2d(float, float, float, float, int);
extern(C) void drawline3d(float, float, float, float, float, float, int);
extern(C) int project2d(float, float, float, float*, float*, float*);
extern(C) void drawspherefill(float, float, float, float, int);
extern(C) void drawpicinquad(int, int, int, int, int, int, int, int, float, float, float, float, float, float, float, float);
extern(C) void drawpolyquad (int, int, int, int, float, float, float, float, float, float, float, float, float, float, float, float, float, float, float, float, float, float);
extern(C) void print4x6 (int, int, int, int, const ubyte *, ...);
extern(C) void print6x8 (int, int, int, int, const ubyte *, ...);
extern(C) void drawtile (int, int, int, int, int, int, int, int, int, int, int, int);
extern(C) int screencapture32bit (const ubyte *);
extern(C) int surroundcapture32bit (dpoint3d *, const ubyte *, int);

/// Sprite related functions:
extern(C) kv6data *getkv6 (const char *);
extern(C) kfatype *getkfa (const char *);
extern(C) void freekv6 (kv6data *kv6);
extern(C) void savekv6 (const char *, kv6data *);
extern(C) void getspr (vx5sprite *, const char *);
extern(C) kv6data *genmipkv6 (kv6data *);
extern(C) char *getkfilname (int);
extern(C) void animsprite (vx5sprite *, int);
extern(C) void drawsprite (vx5sprite *);
extern(C) int meltsphere (vx5sprite *, lpoint3d *, int);
extern(C) int meltspans (vx5sprite *, vspans *, int, lpoint3d *);

	//Physics helper functions:
extern(C) void orthonormalize (point3d *, point3d *, point3d *);
extern(C) void dorthonormalize (dpoint3d *, dpoint3d *, dpoint3d *);
extern(C) void orthorotate (float, float, float, point3d *, point3d *, point3d *);
extern(C) void dorthorotate (double, double, double, dpoint3d *, dpoint3d *, dpoint3d *);
extern(C) void axisrotate (point3d *, point3d *, float);
extern(C) void slerp (point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, float);
extern(C) int cansee (point3d *, point3d *, lpoint3d *);
extern(C) void hitscan (dpoint3d *, dpoint3d *, lpoint3d *, int **, int *);
extern(C) void sprhitscan (dpoint3d *, dpoint3d *, vx5sprite *, lpoint3d *, kv6voxtype **, float *vsc);
extern(C) double findmaxcr (double, double, double, double);
extern(C) void clipmove (dpoint3d *, dpoint3d *, double);
extern(C) int triscan (point3d *, point3d *, point3d *, point3d *, lpoint3d *);
//extern(C) void estnorm (int, int, int, point3d *);

	//VXL reading functions (fast!):
//extern(C) int isvoxelsolid (int, int, int);
extern(C) int anyvoxelsolid (int, int, int, int);
extern(C) int anyvoxelempty (int, int, int, int);
extern(C) int getfloorz (int, int, int);
extern(C) int getcube (int, int, int);

	//VXL writing functions (optimized & bug-free):
extern(C) void setcube (int, int, int, int);
extern(C) void setsphere (lpoint3d *, int, int);
extern(C) void setellipsoid (lpoint3d *, lpoint3d *, int, int, int);
extern(C) void setcylinder (lpoint3d *, lpoint3d *, int, int, int);
extern(C) void setrect (lpoint3d *, lpoint3d *, int);
extern(C) void settri (point3d *, point3d *, point3d *, int);
extern(C) void setsector (point3d *, int *, int, float, int, int);
extern(C) void setspans (vspans *, int, lpoint3d *, int);
extern(C) void setheightmap (const ubyte *, int, int, int, int, int, int, int);
extern(C) void setkv6 (vx5sprite *, int);

	//VXL writing functions (slow or buggy):
extern(C) void sethull3d (point3d *, int, int, int);
extern(C) void setlathe (point3d *, int, int, int);
extern(C) void setblobs (point3d *, int, int, int);
extern(C) void setfloodfill3d (int, int, int, int, int, int, int, int, int);
extern(C) void sethollowfill ();
extern(C) void setkvx (const char *, int, int, int, int, int);
extern(C) void setflash (float, float, float, int, int, int);
extern(C) void setnormflash (float, float, float, int, int);

	//VXL MISC functions:
extern(C) void updatebbox (int, int, int, int, int, int, int);
extern(C) void updatevxl ();
extern(C) void genmipvxl (int, int, int, int);
extern(C) void updatelighting (int, int, int, int, int, int);

	//Falling voxels functions:
extern(C) void checkfloatinbox (int, int, int, int, int, int);
extern(C) void startfalls ();
extern(C) void dofall (int);
extern(C) int meltfall (vx5sprite *, int, int);
extern(C) void finishfalls ();

	//Procedural texture functions:
extern(C) int curcolfunc (lpoint3d *);
extern(C) int floorcolfunc (lpoint3d *);
extern(C) int jitcolfunc (lpoint3d *);
extern(C) int manycolfunc (lpoint3d *);
extern(C) int sphcolfunc (lpoint3d *);
extern(C) int woodcolfunc (lpoint3d *);
extern(C) int pngcolfunc (lpoint3d *);
extern(C) int kv6colfunc (lpoint3d *);

	//Editing backup/restore functions
extern(C) void voxbackup (int, int, int, int, int);
extern(C) void voxdontrestore ();
extern(C) void voxrestore ();
extern(C) void voxredraw ();

	//High-level (easy) picture loading function:
extern(C) void kpzload (const char *, int *, int *, int *, int *);
	//Low-level PNG/JPG functions:
extern(C) int kpgetdim (const char *, int, int *, int *);
extern(C) int kprender (const char *, int, int, int, int, int, int, int);

	//ZIP functions:
extern(C) int kzaddstack (const char *);
extern(C) void kzuninit ();
extern(C) int kzopen (const char *);
extern(C) int kzread (void *, int);
extern(C) int kzfilelength ();
extern(C) int kzseek (int, int);
extern(C) int kztell ();
extern(C) int kzgetc ();
extern(C) int kzeof ();
extern(C) void kzclose ();

extern(C) void kzfindfilestart (const char *); //pass wildcard string
extern(C) int kzfindfile (char *); //you alloc buf, returns 1:found,0:~found
*/
//============================================================
//	CPP PART
//============================================================
enum USEZBUFFER = 1;

enum PREC = 256*4096;
enum CMPPREC = 256*4096;
enum FPREC = 256*4096;
enum USEV5ASM = 1;
enum SCISDIST = 1.0;
enum GOLDRAT = 0.3819660112501052; //Golden Ratio: 1 - 1/((sqrt(5)+1)/2)
enum ESTNORMRAD = 2; // Specially optimized for 2: DON'T CHANGE unless testing! 

import core.sys.windows.windows;
import std.math;
import std.stdio;
import std.c.stdarg;
import std.c.stdlib;

extern char keystatus[256];
extern void readkeyboard();
extern void breath();
extern int startdirectdraw(int *, int *, int *, int *);
extern void stopdirectdraw();
extern void nextpage();
extern void evilquit(const char *);

enum VOXSIZ = VSID*VSID*128;
ubyte *sptr[(VSID*VSID*4)/3];

static int* vbuf = null, vbit = null;
static int vbiti;

	//WARNING: loaddta uses last 2MB of vbuf; vbuf:[VOXSIZ>>2], vbit:[VOXSIZ>>7]
	//WARNING: loadpng uses last 4MB of vbuf; vbuf:[VOXSIZ>>2], vbit:[VOXSIZ>>7]

//                     ÚÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄ¿
//        vbuf format: ³   0:   ³   1:   ³   2:   ³   3:   ³
//ÚÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÅÄÄÄÄÄÄÄÄÅÄÄÄÄÄÄÄÄÅÄÄÄÄÄÄÄÄÅÄÄÄÄÄÄÄÄ´
//³      First header: ³ nextptr³   z1   ³   z1c  ³  dummy ³
//³           Color 1: ³    b   ³    g   ³    r   ³ intens ³
//³           Color 2: ³    b   ³    g   ³    r   ³ intens ³
//³             ...    ³    b   ³    g   ³    r   ³ intens ³
//³           Color n: ³    b   ³    g   ³    r   ³ intens ³
//³ Additional header: ³ nextptr³   z1   ³   z1c  ³   z0   ³
//ÀÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÙ
//  nextptr: add this # <<2 to index to get to next header (0 if none)
//       z1: z floor (top of floor color list)
//      z1c: z bottom of floor color list MINUS 1! - needed to calculate
//             slab size with slng() and used as a separator for fcol/ccol
//       z0: z ceiling (bottom of ceiling color list)

	//Memory management variables:

enum MAXCSIZ = 1028;
ubyte tbuf[MAXCSIZ];
int tbuf2[MAXZDIM*3];
int templongbuf[MAXZDIM];

extern int cputype; //bit25=1: SSE, bits30&31=1,1:3DNow!+

static char nullst = 0; // nullst always NULL string

enum SETSPHMAXRAD = 256;
static double logint[SETSPHMAXRAD];
static float tempfloatbuf[SETSPHMAXRAD];
static int factr[SETSPHMAXRAD][2];

static if (USEZBUFFER == 0)
{
	align(1) struct castdat
	{
		int col;
	}
} else
{
	align(1) struct castdat
	{
		int col, dist;
	}
}

align(1) struct cftype
{
	castdat* i0, i1;
	int z0, z1, cx0, cy0, cx1, cy1;
}

align(1) struct uspoint2d
{
	ushort x, y;
}

align(1) struct lpoint2d
{
	int x, y;
}

align(1) struct point2d
{
	float x, y;
}

static if(USEV5ASM)
{
	extern(C) void *cfasm;
	extern(C) castdat skycast;
	enum cf = (cast(cftype *)&cfasm);
} else
{
	cftype cf[256];
}

// Screen related variables:
static int xres, yres, bytesperline, frameplace, xres4;
int ylookup[MAXYDIM+1];

static lpoint3d glipos;
static point3d gipos, gistr, gihei, gifor;
static point3d gixs, giys, gizs, giadd;
static float gihx, gihy, gihz, grd;
static float[2] gposxfrac, gposyfrac;
static int gposz, giforzsgn, gstartz0, gstartz1;
static int[2] gixyi;
static ubyte *gstartv;

int backtag, backedup = -1, bacx0, bacy0, bacx1, bacy1;
ubyte* bacsptr[262144];

// Flash variables
enum LOGFLASHVANG = 9;
static lpoint2d gfc[(1<<LOGFLASHVANG)*8];
static int gfclookup[8] = [4,7,2,5,9,3,6,1];
static int flashcnt = 0;
long flashbrival;

// Norm flash variables
enum GSIZ = 512; // NONE: GSIZ should be 1<,x, and must be <= 65536
static int bbuf[GSIZ][GSIZ>>5];
static int[32] p2c, p2m; // bbuf: 2.0K
static uspoint2d ffx[((GSIZ>>1)+2)*(GSIZ>>1)];
static uspoint2d *ffxptr; // ffx: 16.5K
static int xbsox = -17;
static int xbsoy, xbsof;
static long xbsbuf[25*5+1]; //need few bits befor&after for protection

// Look tables for expandbitstack256:
static int[32] xbsceil, xbsflor;

// float detection & falling code varialbes...
// WARNING: VLSTSIZ, FSTKSIZ, FLCHKSIZ can all have bound errors! :(
enum VLSTSIZ = 65536; //Theoretically should be at least: VOXSIZ/8
enum LOGHASHEAD = 12;
enum FSTKSIZ = 8192;

struct vlstyp
{
	int v, b;
}

vlstyp vlst[VLSTSIZ];
int hhead[1<<LOGHASHEAD];
int vlstcnt = 0x7fffffff;
lpoint3d fstk[FSTKSIZ]; //Note .z is actually used as a pointer, not z!
enum FLCHKSIZ = 4096;
lpoint3d flchk[FLCHKSIZ];
int flchkcnt = 0;

// Opticast global variables:
// radar: 320x200 requires  419560*2 bytes (area * 6.56*2)
// radar: 400x300 requires  751836*2 bytes (area * 6.27*2)
// radar: 640x480 requires 1917568*2 bytes (area * 6.24*2)
enum SCPITCH = 256;
int* radar = null, radarmem = null;

static if(USEZBUFFER == 1)
{
	static int *zbuffermem = null, zbuffersiz = null;
}

static castdat*[MAXXDIM*4]  angstart;
static castdat*				gscanptr;

static T max(T)(T a, T b)
{
	return a >= b ? a : b; 
}

enum CMPRECIPSIZ = MAXXDIM+32;
static float cmprecip[CMPRECIPSIZ];
static float wx0, wy0, wx1, wy1;
static int iwx0, iwy0, iwx1, iwy1;
static point3d gcorn[4];
	   point3d ginor[4]; //Should be static, but... necessary for stupid pingball hack :/
static int lastx[max(MAXYDIM, VSID)];
static int uurendmem[MAXXDIM*2+8];
static int* uurend;

void mat0(point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, point3d *);
void mat1(point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, point3d *);
void mat2(point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, point3d *);

//Parallaxing sky variables:
static int skypic = 0, nskypic = 0, skybpl, skyysiz, skycurlng, skycurdir;
static float skylngmul;
static point2d *skylng = null;

extern(C)
{
	//Parallaxing sky variables (accessed by assemly code)
	int skyoff = 0, skyxsiz;
	int *skylat = null;

	long gi;
	long gcsub[8] = 
	[
		0xff00ff00ff00ff,0xff00ff00ff00ff,0xff00ff00ff00ff,0xff00ff00ff00ff,
		0xff00ff00ff00ff,0xff00ff00ff00ff,0xff00ff00ff00ff,0xff00ff00ff00ff
	];
	int gylookup[512+36];
	int gmipnum = 0;  //256+4+128+4+64+4+...
	int[2] gpz, gdz, gixy;
	int gxmip, gxmax, gpixy;
	static int gmaxscandist;

	//int rEAX, rebx, recx, redx, resi, redi, rebp, resp, remm[16];
	void v5_asm_dep_unlock();
	void grouscanasm(int);

	static if(USEZBUFFER == 1)
	{
		int zbufoff;
	}
}

//enum gi0 = ((cast(int *)&gi)[0]); crash dmd
//enum gi1 = ((cast(int *)&gi)[1]);

static int gi0() @property @trusted nothrow
{
	return (cast(int*)&gi)[0];
}

static int gi1() @property @trusted nothrow
{
	return (cast(int*)&gi)[1];
}


static void fcossin(float a, float *c, float *s)
{
	asm {
		fld a;
		fsincos;
		mov EAX, c;
		fstp dword ptr [EAX];
		mov EAX, s;
		fstp dword ptr [EAX];
	}
}

static void dcossin(double a, double *c, double *s)
{
	asm {
		fld a;
		fsincos;
		mov EAX, c;
		fstp qword ptr [EAX];
		mov EAX, s;
		fstp qword ptr [EAX];
	}
}

static void ftol(float f, int *a)
{
	asm
	{
		mov EAX, a;
		fld f;
		fistp dword ptr [EAX];
	}
}

static void dtol(double d, int *a)
{
	asm
	{
		mov EAX, a;
		fld qword ptr d;
		fistp dword ptr [EAX];
	}
}

//WARNING: This ASM cod requires >= PPRO
static double dbound(double d, double dmin, double dmax)
{
	asm
	{
		fld dmin;
		fld d;
		fucomi ST, ST(1); //if(d < dmin)
		fcmovb ST, ST(1); //	d = dmin;
		fld dmax;
		fxch ST(1);
		fucomi ST, ST(1); //if(d > dmax)
		fcmovnb ST, ST(1);//	d = dmax;
		fstp d;
		fucompp;
	}
	return d;
}

static int mulshr16(int a, int d)
{
	asm
	{
		mov EAX, a;
		mov EDX, d;
		imul EDX;
		shrd EAX, EDX, 16;
	}
}

static long mul64(int a, int d)
{
	asm
	{
		mov EAX, a;
		imul d;
	}
}

static int shldiv16(int a, int b)
{
	asm
	{
		mov EAX, a;
		mov EDX, EAX;
		shl EAX, 16;
		sar EDX, 16;
		idiv b;
	}
}

static int isshldiv16safe(int a, int b)
{
	asm
	{
		mov EDX, a;
		test EDX, EDX;
		js short skipneg0;
		neg EDX;
	skipneg0:
		sar EDX, 14;

		mov EAX, b;
		test EAX, EAX;
		js short skipneg1;
		neg EAX;
	skipneg1:
		// abs((a<<16)/b) < (1<<30) 1 extra for good luck!
		//-abs(a)>>14 > -abs(b) use -abs because safe fo 0x80000000
		//EAX-edx < 0
		sub EAX, EDX;
		shr EAX, 31;
	}
}

static int umulshr32(int a, int d)
{
	asm
	{
		mov EAX, a;
		mul d;
		mov EAX, EDX;
	}
}

static int scale(int a, int d, int c)
{
	asm
	{
		mov EAX, a;
		imul d;
		idiv c;
	}
}

static int dmulrethigh(int b, int c, int a, int d)
{
	asm
	{
		mov EAX, a;
		imul d;
		mov ECX, EAX;
		push EDX;
		mov EAX, b;
		imul c;
		sub EAX, ECX;
		pop ECX;
		sbb EDX, ECX;
		mov EAX, EDX;
	}
}

static void copybuf(void *s, void *d, int c)
{
	asm
	{
		push ESI;
		push EDI;
		mov ESI, s;
		mov EDI, d;
		mov ECX, c;
		rep;
		movsd;
		pop EDI;
		pop ESI;
	}
}

static void clearbuf(void *d, int c, int a)
{
	asm
	{
		push EDI;
		mov EDI, d;
		mov ECX, c;
		mov EAX, a;
		rep;
		stosd;
		pop EDI;
	}
}

//if (a < 0) return(0); else if (a > b) return(b); else return(a);
static int lbound0(int a, int b) //b MUST be >= 0
{
	if (cast(uint)a <= b) return a;
	return((~(a>>31))&b);
}

//if (a < b) return(b); else if (a > c) return(c); else return(a);
static int lbound(int a, int b, int c) //c MUST be >= b
{
	c -= b;
	if (cast(uint)(a-b) <= c) return a;
	return((((b-a)>>31)&c) + b);
}

enum LSINSIZ = 8; // Must be >= 2!
static point2d usintab[(1<<LSINSIZ)+(1<<(LSINSIZ-2))];
static void ucossininit()
{
	int i, j;
	double a, ai, s, si, m;

	j = 0; usintab[0].y = 0.0;
	i = (1<<LSINSIZ)-1;
	ai = PI*(-2)/(cast(float)(1<<LSINSIZ));
	a = (cast(float)(-i))*ai;
	ai *= .5; m = sin(ai)*2;
	s = sin(a); si = cos(a+ai)*m;
	m = -m*m;

	for(;i>=0;i--)
	{
		usintab[cast(size_t)i].y = s; s += si; si += s*m; //MUCH faster than next line :)
		//usintab[i].y = sin(i*PI*2/((float)(1<<LSINSIZ)));
		usintab[cast(size_t)i].x = (usintab[cast(size_t)j].y-usintab[cast(size_t)i].y)/(cast(float)(1<<(32-LSINSIZ)));
		j = i;
	}
	for(i=(1<<(LSINSIZ-2))-1;i>=0;i--) usintab[cast(size_t)(i+(1<<LSINSIZ))] = usintab[cast(size_t)i];
}

//Calculates cos & sin of 32-bit unsigned long angle in ~15 clock cycles
//  Accuracy is approximately +/-.0001
static void ucossin(uint a, float *cosin)
{
	float f = (cast(float)(a&((1<<(32-LSINSIZ))-1))); a >>= (32-LSINSIZ);
	cosin[0] = usintab[cast(size_t)(a+(1<<(LSINSIZ-2)))].x*f+usintab[cast(size_t)(a+(1<<(LSINSIZ-2)))].y;
	cosin[1] = usintab[cast(size_t)a                  ].x*f+usintab[cast(size_t)a                    ].y;
}

static const int font4x6[] = //256 DOS chars, from Ken's Build SMALLFNT
[
	0x000000,0x6f9f60,0x69f960,0xaffe40,0x4efe40,0x6ff6f0,0x66f6f0,0x000000,
	0xeeaee0,0x000000,0x000000,0x000000,0x000000,0x000000,0x7755c0,0x96f690,
	0x8cec80,0x26e620,0x4e4e40,0xaaa0a0,0x7dd550,0x7ca6c0,0x000ee0,0x4e4ee0,
	0x4e4440,0x444e40,0x02f200,0x04f400,0x000000,0x000000,0x000000,0x000000,
	0x000000,0x444040,0xaa0000,0xafafa0,0x6c46c0,0xa248a0,0x4a4ce0,0x240000,
	0x488840,0x422240,0x0a4a00,0x04e400,0x000224,0x00e000,0x000040,0x224480,
	0xeaaae0,0x444440,0xe2e8e0,0xe2e2e0,0xaae220,0xe8e2e0,0xe8eae0,0xe22220,
	0xeaeae0,0xeae220,0x040400,0x040480,0x248420,0x0e0e00,0x842480,0xc24040,
	0xeaece0,0x4aeaa0,0xcacac0,0x688860,0xcaaac0,0xe8c8e0,0xe8c880,0xe8aae0,
	0xaaeaa0,0xe444e0,0xe22a60,0xaacaa0,0x8888e0,0xaeeaa0,0xaeeea0,0xeaaae0,
	0xeae880,0xeaae60,0xeacaa0,0xe8e2e0,0xe44440,0xaaaae0,0xaaa440,0xaaeea0,
	0xaa4aa0,0xaae440,0xe248e0,0xc888c0,0x844220,0x622260,0x4a0000,0x0000e0,
	0x420000,0x006a60,0x88eae0,0x00e8e0,0x22eae0,0x006e60,0x24e440,0x06a62c,
	0x88eaa0,0x040440,0x040448,0x88aca0,0x444440,0x08eee0,0x00caa0,0x00eae0,
	0x00eae8,0x00eae2,0x00e880,0x0064c0,0x04e440,0x00aa60,0x00aa40,0x00eee0,
	0x00a4a0,0x00aa6c,0x00c460,0x648460,0x440440,0xc424c0,0x6c0000,0x04ae00,
	0x68886c,0xa0aa60,0x606e60,0xe06a60,0xa06a60,0xc06a60,0x046a60,0x00e8e4,
	0xe06e60,0xa06e60,0xc06e60,0x0a0440,0x0e0440,0x0c0440,0xa4aea0,0x404ea0,
	0x60ece0,0x007a70,0x7afab0,0xe0eae0,0xa0eae0,0xc0eae0,0xe0aa60,0xc0aa60,
	0xa0aa6c,0xa0eae0,0xa0aae0,0x4e8e40,0x65c4f0,0xa4ee40,0xcafab0,0x64e4c0,
	0x606a60,0x060440,0x60eae0,0x60aa60,0xc0caa0,0xe0aea0,0x6a60e0,0xeae0e0,
	0x404860,0x007400,0x00c400,0x8a4e60,0x8a6e20,0x404440,0x05a500,0x0a5a00,
	0x282828,0x5a5a5a,0xd7d7d7,0x444444,0x44c444,0x44cc44,0x66e666,0x00e666,
	0x00cc44,0x66ee66,0x666666,0x00ee66,0x66ee00,0x66e000,0x44cc00,0x00c444,
	0x447000,0x44f000,0x00f444,0x447444,0x00f000,0x44f444,0x447744,0x667666,
	0x667700,0x007766,0x66ff00,0x00ff66,0x667766,0x00ff00,0x66ff66,0x44ff00,
	0x66f000,0x00ff44,0x00f666,0x667000,0x447700,0x007744,0x007666,0x66f666,
	0x44ff44,0x44c000,0x007444,0xffffff,0x000fff,0xcccccc,0x333333,0xfff000,
	0x00dad0,0xcacac8,0xea8880,0x00f660,0xe848e0,0x007a40,0x0aac80,0x05a220,
	0xe4a4e0,0x4aea40,0x6996f0,0x646ae0,0x06f600,0x16f680,0x68c860,0x4aaaa0,
	0xe0e0e0,0x4e40e0,0x4240e0,0x4840e0,0x4a8880,0x222a40,0x40e040,0x06ec00,
	0xeae000,0x0cc000,0x00c000,0x644c40,0xcaa000,0xc4e000,0x0eee00,0x000000,
];

void print4x6 (int x, int y, int fcol, int bcol, const char *fmt, ...)
{
	va_list arglist;
	char st[280];
	char *c;
	int i, j;

	if (!fmt) return;
	va_start(arglist,fmt);
	vsprintf(&st[0],fmt,arglist);
	va_end(arglist);

	y = y*bytesperline+(x<<2)+frameplace;
	if (bcol < 0)
	{
		for(j=20;j>=0;y+=bytesperline,j-=4)
			for(c=&st[0],x=y;*c;c++,x+=16)
			{
				i = (font4x6[*c]>>j);
				if (i&8) *cast(int *)(x   ) = fcol;
				if (i&4) *cast(int *)(x+ 4) = fcol;
				if (i&2) *cast(int *)(x+ 8) = fcol;
				if (i&1) *cast(int *)(x+12) = fcol;
				if ((*c) == 9) x += 32;
			}
		return;
	}
	fcol -= bcol;
	for(j=20;j>=0;y+=bytesperline,j-=4)
		for(c=&st[0],x=y;*c;c++,x+=16)
		{
			i = (font4x6[*c]>>j);
			*cast(int *)(x   ) = (((i<<28)>>31)&fcol)+bcol;
			*cast(int *)(x+ 4) = (((i<<29)>>31)&fcol)+bcol;
			*cast(int *)(x+ 8) = (((i<<30)>>31)&fcol)+bcol;
			*cast(int *)(x+12) = (((i<<31)>>31)&fcol)+bcol;
			if ((*c) == 9) { for(i=16;i<48;i+=4) *cast(int *)(x+i) = bcol; x += 32; }
		}
}

	//NOTE: font is stored vertically first! (like .ART files)
static const long font6x8[] = //256 DOS chars, from: DOSAPP.FON (tab blank)
[
	0x3E00000000000000,0x6F6B3E003E455145,0x1C3E7C3E1C003E6B,0x3000183C7E3C1800,
	0x7E5C180030367F36,0x000018180000185C,0x0000FFFFE7E7FFFF,0xDBDBC3FF00000000,
	0x0E364A483000FFC3,0x6000062979290600,0x0A7E600004023F70,0x2A1C361C2A003F35,
	0x0800081C3E7F0000,0x7F361400007F3E1C,0x005F005F00001436,0x22007F017F090600,
	0x606060002259554D,0x14B6FFB614000060,0x100004067F060400,0x3E08080010307F30,
	0x08083E1C0800081C,0x0800404040407800,0x3F3C3000083E083E,0x030F3F0F0300303C,
	0x0000000000000000,0x0003070000065F06,0x247E247E24000307,0x630000126A2B2400,
	0x5649360063640813,0x0000030700005020,0x00000000413E0000,0x1C3E080000003E41,
	0x08083E080800083E,0x0800000060E00000,0x6060000008080808,0x0204081020000000,
	0x00003E4549513E00,0x4951620000407F42,0x3649494922004649,0x2F00107F12141800,
	0x494A3C0031494949,0x0305097101003049,0x0600364949493600,0x6C6C00001E294949,
	0x00006CEC00000000,0x2400004122140800,0x2241000024242424,0x0609590102000814,
	0x7E001E555D413E00,0x49497F007E111111,0x224141413E003649,0x7F003E4141417F00,
	0x09097F0041494949,0x7A4949413E000109,0x00007F0808087F00,0x4040300000417F41,
	0x412214087F003F40,0x7F00404040407F00,0x04027F007F020402,0x3E4141413E007F08,
	0x3E00060909097F00,0x09097F005E215141,0x3249494926006619,0x3F0001017F010100,
	0x40201F003F404040,0x3F403C403F001F20,0x0700631408146300,0x4549710007087008,
	0x0041417F00000043,0x0000201008040200,0x01020400007F4141,0x8080808080800402,
	0x2000000007030000,0x44447F0078545454,0x2844444438003844,0x38007F4444443800,
	0x097E080008545454,0x7CA4A4A418000009,0x0000007804047F00,0x8480400000407D00,
	0x004428107F00007D,0x7C0000407F000000,0x04047C0078041804,0x3844444438000078,
	0x380038444444FC00,0x44784400FC444444,0x2054545408000804,0x3C000024443E0400,
	0x40201C00007C2040,0x3C6030603C001C20,0x9C00006C10106C00,0x54546400003C60A0,
	0x0041413E0800004C,0x0000000077000000,0x02010200083E4141,0x3C2623263C000001,
	0x3D001221E1A11E00,0x54543800007D2040,0x7855555520000955,0x2000785554552000,
	0x5557200078545555,0x1422E2A21C007857,0x3800085555553800,0x5555380008555455,
	0x00417C0100000854,0x0000004279020000,0x2429700000407C01,0x782F252F78007029,
	0x3400455554547C00,0x7F097E0058547C54,0x0039454538004949,0x3900003944453800,
	0x21413C0000384445,0x007C20413D00007D,0x3D00003D60A19C00,0x40413C00003D4242,
	0x002466241800003D,0x29006249493E4800,0x16097F00292A7C2A,0x02097E8840001078,
	0x0000785555542000,0x4544380000417D00,0x007D21403C000039,0x7A0000710A097A00,
	0x5555080000792211,0x004E51514E005E55,0x3C0020404D483000,0x0404040404040404,
	0x506A4C0817001C04,0x0000782A34081700,0x0014080000307D30,0x0814000814001408,
	0x55AA114411441144,0xEEBBEEBB55AA55AA,0x0000FF000000EEBB,0x0A0A0000FF080808,
	0xFF00FF080000FF0A,0x0000F808F8080000,0xFB0A0000FE0A0A0A,0xFF00FF000000FF00,
	0x0000FE02FA0A0000,0x0F0800000F080B0A,0x0F0A0A0A00000F08,0x0000F80808080000,
	0x080808080F000000,0xF808080808080F08,0x0808FF0000000808,0x0808080808080808,
	0xFF0000000808FF08,0x0808FF00FF000A0A,0xFE000A0A0B080F00,0x0B080B0A0A0AFA02,
	0x0A0AFA02FA0A0A0A,0x0A0A0A0AFB00FF00,0xFB00FB0A0A0A0A0A,0x0A0A0B0A0A0A0A0A,
	0x0A0A08080F080F08,0xF808F8080A0AFA0A,0x08080F080F000808,0x00000A0A0F000000,
	0xF808F8000A0AFE00,0x0808FF00FF080808,0x08080A0AFB0A0A0A,0xF800000000000F08,
	0xFFFFFFFFFFFF0808,0xFFFFF0F0F0F0F0F0,0xFF000000000000FF,0x0F0F0F0F0F0FFFFF,
	0xFE00241824241800,0x01017F0000344A4A,0x027E027E02000003,0x1800006349556300,
	0x2020FC00041C2424,0x000478040800001C,0x3E00085577550800,0x02724C00003E4949,
	0x0030595522004C72,0x1800182418241800,0x2A2A1C0018247E24,0x003C02023C00002A,
	0x0000002A2A2A2A00,0x4A4A510000242E24,0x00514A4A44000044,0x20000402FC000000,
	0x2A08080000003F40,0x0012241224000808,0x0000000609090600,0x0008000000001818,
	0x02023E4030000000,0x0900000E010E0100,0x3C3C3C0000000A0D,0x000000000000003C,
];

void print6x8 (int x, int y, int fcol, int bcol, const char *fmt, ...)
{
	va_list arglist;
	char st[280];
	char* c, v;
	int i, j;

	if (!fmt) return;
	va_start(arglist,fmt);
	vsprintf(&st[0],fmt,arglist);
	va_end(arglist);

	y = y*bytesperline+(x<<2)+frameplace;
	if (bcol < 0)
	{
		for(j=1;j<256;y+=bytesperline,j<<=1)
			for(c=&st[0],x=y;*c;c++,x+=24)
			{
				v = cast(char *)((cast(int)&font6x8) + (cast(int)c[0])*6);
				if (v[0]&j) *cast(int *)(x   ) = fcol;
				if (v[1]&j) *cast(int *)(x+ 4) = fcol;
				if (v[2]&j) *cast(int *)(x+ 8) = fcol;
				if (v[3]&j) *cast(int *)(x+12) = fcol;
				if (v[4]&j) *cast(int *)(x+16) = fcol;
				if (v[5]&j) *cast(int *)(x+20) = fcol;
				if ((*c) == 9) x += ((2*6)<<2);
			}
		return;
	}
	fcol -= bcol;
	for(j=1;j<256;y+=bytesperline,j<<=1)
		for(c=&st[0],x=y;*c;c++,x+=24)
		{
			v = cast(char *)((cast(int)&font6x8) + (cast(int)c[0])*6);
			*cast(int *)(x   ) = (((-(v[0]&j))>>31)&fcol)+bcol;
			*cast(int *)(x+ 4) = (((-(v[1]&j))>>31)&fcol)+bcol;
			*cast(int *)(x+ 8) = (((-(v[2]&j))>>31)&fcol)+bcol;
			*cast(int *)(x+12) = (((-(v[3]&j))>>31)&fcol)+bcol;
			*cast(int *)(x+16) = (((-(v[4]&j))>>31)&fcol)+bcol;
			*cast(int *)(x+20) = (((-(v[5]&j))>>31)&fcol)+bcol;
			if ((*c) == 9) { for(i=24;i<72;i+=4) *cast(int *)(x+i) = bcol; x += ((2*6)<<2); }
		}
}

static int gkrand = 0;
int colorjit(int i, int jitamount)
{
	gkrand = (gkrand*27584621)+1;
	return((gkrand&jitamount)^i);
}

static T min(T)(T a, T b)
{
	return a >= b ? b : a;
}

int lightvox (int i)
{
	int r, g, b;

	b = (cast(uint)i>>24);
	r = min((((i>>16)&255)*b)>>7,255);
	g = min((((i>>8 )&255)*b)>>7,255);
	b = min((((i    )&255)*b)>>7,255);
	return((r<<16)+(g<<8)+b);
}

	//Note: ebx = 512 is no change
	//If PENTIUM III:1.Replace punpcklwd&punpckldq with: pshufw MM1, MM1, 0
	//               2.Use pmulhuw, shift by 8 & mul by 256
	//  :(  Can't mix with floating point
//#pragma aux colormul =
//   "movd MM0, EAX"
//   "pxor MM1, MM1"
//   "punpcklbw MM0, MM1"
//   "psllw MM0, 7"
//   "movd MM1, ebx"
//   "punpcklwd MM1, MM1"
//   "punpckldq MM1, MM1"
//   "pmulhw MM0, MM1"
//   "packsswb MM0, MM0"
//   "movd EAX, MM0"
//   parm [EAX][ebx]
//   modify exact [EAX]
//   value [EAX]

int colormul(int i, int mulup8)
{
	int r, g, b;

	r = ((((i>>16)&255)*mulup8)>>8); if (r > 255) r = 255;
	g = ((((i>>8 )&255)*mulup8)>>8); if (g > 255) g = 255;
	b = ((((i    )&255)*mulup8)>>8); if (b > 255) b = 255;
	return((i&0xff000000)+(r<<16)+(g<<8)+b);
}

int curcolfunc (lpoint3d *p) { return vx5.curcol; }

int floorcolfunc(lpoint3d *p)
{
	ubyte *v;
	for(v=sptr[cast(size_t)(p.y*VSID+p.x)];(p.z>v[2]) && (v[0]);v+=v[0]*4) {}
	return *cast(int *)&v[4];	
}

int jitcolfunc(lpoint3d *p) { return colorjit(vx5.curcol, vx5.amount);}

static int manycolukup[64] =
[
	  0,  1,  2,  5, 10, 15, 21, 29, 37, 47, 57, 67, 79, 90,103,115,
	127,140,152,165,176,188,198,208,218,226,234,240,245,250,253,254,
	255,254,253,250,245,240,234,226,218,208,198,188,176,165,152,140,
	128,115,103, 90, 79, 67, 57, 47, 37, 29, 21, 15, 10,  5,  2,  1
];

int manycolfunc(lpoint3d *p)
{
	return (manycolukup[p.x&63]<<16)+(manycolukup[p.y&63]<<8)+manycolukup[p.z&63]+0x80000000;
}

int sphcolfunc(lpoint3d *p)
{
	int i;
	ftol(sin((p.x+p.y+p.z-vx5.cen)*vx5.daf)*(-96), &i);
	return ((i+128)<<24)|(vx5.curcol&0xffffff);
}

enum WOODXSIZ = 46;
enum WOODYSIZ = 24;
enum WOODZSIZ = 24;

static float[256] wx, wy, wz, vx, vy, vz;
int woodcolfunc(lpoint3d *p)
{
	float col, u, a, f, dx, dy, dz;
	int i, c, xof, yof, tx, ty, xoff;

	if (*cast(int *)&wx[0] == 0)
	{
		for(i=0;i<256;i++)
		{
			wx[cast(size_t)i] = WOODXSIZ * (cast(float)rand()/32768.0f-.5f) * .5f;
			wy[cast(size_t)i] = WOODXSIZ * (cast(float)rand()/32768.0f-.5f) * .5f;
			wz[cast(size_t)i] = WOODXSIZ * (cast(float)rand()/32768.0f-.5f) * .5f;

				//UNIFORM spherical randomization (see spherand.c)
			dz = 1.0f-cast(float)rand()/32768.0f*.04f;
			a = cast(float)rand()/32768.0f*PI*2.0f; fcossin(a,&dx,&dy);
			f = sqrt(1.0f-dz*dz); dx *= f; dy *= f;
				//??z: rings,  ?z?: vertical,  z??: horizontal (nice)
			vx[cast(size_t)i] = dz; vy[cast(size_t)i] = fabs(dy); vz[cast(size_t)i] = dx;
		}
	}

		//(tx&,ty&) = top-left corner of current panel
	ty = p.y - (p.y%WOODYSIZ);
	xoff = ((ty/WOODYSIZ)*(ty/WOODYSIZ)*51721 + (p.z/WOODZSIZ)*357) % WOODXSIZ;
	tx = ((p.x+xoff) - (p.x+xoff)%WOODXSIZ) - xoff;

	xof = p.x - (tx + (WOODXSIZ>>1));
	yof = p.y - (ty + (WOODYSIZ>>1));

	c = ((((tx*429 + 4695) ^ (ty*341 + 4355) ^ 13643) * 2797) & 255);
	dx = xof - wx[cast(size_t)c];
	dy = yof - wy[cast(size_t)c];
	dz = (p.z%WOODZSIZ) - wz[cast(size_t)c];

		//u = distance to center of randomly oriented cylinder
	u = vx[cast(size_t)c]*dx + vy[cast(size_t)c]*dy + vz[cast(size_t)c]*dz;
	u = sqrt(dx*dx + dy*dy + dz*dz - u*u);

		//ring randomness
	u += sin(cast(float)xof*.12 + cast(float)yof*.15) * .5;
	u *= (sin(u)*.05 + 1);

		//Ring function: smooth saw-tooth wave
	col = sin(u*2)*24;
	col *= pow(1.0f-vx[cast(size_t)c],.3f);

		//Thin shaded borders
	if ((p.x-tx == 0) || (p.y-ty == 0)) col -= 5;
	if ((p.x-tx == WOODXSIZ-1) || (p.y-ty == WOODYSIZ-1)) col -= 3;

	//f = col+c*.12+72; i = ftolp3(&f);
	  ftol(col+c*.12f+72.0f,&i);

	return(colormul(vx5.curcol,i<<1));	
}

int gxsizcache = 0, gysizcache = 0;
int pngcolfunc (lpoint3d *p)
{
	int x, y, z, u, v;
	float fx, fy, fz, rx, ry, rz;

	if (!vx5.pic) return(vx5.curcol);
	switch(vx5.picmode)
	{
		case 0:
			x = p.x-vx5.pico.x; y = p.y-vx5.pico.y; z = p.z-vx5.pico.z;
			u = (((x&vx5.picu.x) + (y&vx5.picu.y) + (z&vx5.picu.z))^vx5.xoru);
			v = (((x&vx5.picv.x) + (y&vx5.picv.y) + (z&vx5.picv.z))^vx5.xorv);
			break;
		case 1: case 2:
			fx = cast(float)p.x-vx5.fpico.x;
			fy = cast(float)p.y-vx5.fpico.y;
			fz = cast(float)p.z-vx5.fpico.z;
			rx = vx5.fpicu.x*fx + vx5.fpicu.y*fy + vx5.fpicu.z*fz;
			ry = vx5.fpicv.x*fx + vx5.fpicv.y*fy + vx5.fpicv.z*fz;
			rz = vx5.fpicw.x*fx + vx5.fpicw.y*fy + vx5.fpicw.z*fz;
			ftol(atan2(ry,rx)*vx5.xoru/(PI*2),&u);
			if (vx5.picmode == 1) ftol(rz,&v);
			else ftol((atan2(rz,sqrt(rx*rx+ry*ry))/PI+.5)*vx5.ysiz,&v);
			break;
		default: //case 3:
			fx = cast(float)p.x-vx5.fpico.x;
			fy = cast(float)p.y-vx5.fpico.y;
			fz = cast(float)p.z-vx5.fpico.z;
			ftol(vx5.fpicu.x*fx + vx5.fpicu.y*fy + vx5.fpicu.z*fz,&u);
			ftol(vx5.fpicv.x*fx + vx5.fpicv.y*fy + vx5.fpicv.z*fz,&v);
			break;
	}
	if (cast(uint)(u-gxsizcache) >= cast(uint)vx5.xsiz)
		if (u < 0) gxsizcache = u-(u+1)%vx5.xsiz-vx5.xsiz+1; else gxsizcache = u-(u%vx5.xsiz);
	if (cast(uint)(v-gysizcache) >= cast(uint)vx5.ysiz)
		if (v < 0) gysizcache = v-(v+1)%vx5.ysiz-vx5.ysiz+1; else gysizcache = v-(v%vx5.ysiz);
	return((vx5.pic[cast(size_t)((v-gysizcache)*(vx5.bpl>>2)+(u-gxsizcache))]&0xffffff)|0x80000000);
}

	//Special case for SETSEC & SETCEI bumpmapping (vx5.picmode == 3)
	//no safety checks, returns alpha as signed char in range: (-128 to 127)
int hpngcolfunc (point3d *p)
{
	int u, v;
	float fx, fy, fz;

	fx = p.x-vx5.fpico.x;
	fy = p.y-vx5.fpico.y;
	fz = p.z-vx5.fpico.z;
	ftol(vx5.fpicu.x*fx + vx5.fpicu.y*fy + vx5.fpicu.z*fz,&u);
	ftol(vx5.fpicv.x*fx + vx5.fpicv.y*fy + vx5.fpicv.z*fz,&v);

	if (cast(uint)(u-gxsizcache) >= cast(uint)vx5.xsiz)
		if (u < 0) gxsizcache = u-(u+1)%vx5.xsiz-vx5.xsiz+1; else gxsizcache = u-(u%vx5.xsiz);
	if (cast(uint)(v-gysizcache) >= cast(uint)vx5.ysiz)
		if (v < 0) gysizcache = v-(v+1)%vx5.ysiz-vx5.ysiz+1; else gysizcache = v-(v%vx5.ysiz);
	return(vx5.pic[cast(size_t)((v-gysizcache)*(vx5.bpl>>2)+(u-gxsizcache))]>>24);
}

static int slng (const ubyte *s)
{
	ubyte *v;

	for(v=cast(ubyte*)s;v[0];v+=v[0]*4) {}
	return(cast(int)v-cast(int)s+(v[2]-v[1]+1)*4+4);
}

void voxdealloc (const ubyte *v)
{
	int i, j;
	i = ((cast(int)v-cast(int)vbuf)>>2); j = (slng(v)>>2)+i;
	
	static if(0)
	{
		while (i < j) { vbit[cast(size_t)(i>>5)] &= ~(1<<i); i++; }
	} else
	{
		if (!((j^i)&~31))
			vbit[cast(size_t)(i>>5)] &= ~(p2m[cast(size_t)(j&31)]^p2m[cast(size_t)(i&31)]);
		else
		{
			vbit[cast(size_t)(i>>5)] &=   p2m[cast(size_t)(i&31)];  i >>= 5;
			vbit[cast(size_t)(j>>5)] &= (~p2m[cast(size_t)(j&31)]); j >>= 5;
			for(j--;j>i;j--) vbit[cast(size_t)j] = 0;
		}
	}
}

	//Note: danum MUST be a multiple of 4!
ubyte *voxalloc (int danum)
{
	int i, badcnt, p0, p1, vend;

	badcnt = 0; danum >>= 2; vend = (VOXSIZ>>2)-danum;
	do
	{
		for(;vbiti<vend;vbiti+=danum)
		{
			if (vbit[cast(size_t)(vbiti>>5)]&(1<<vbiti)) continue;
			for(p0=vbiti;(!(vbit[cast(size_t)((p0-1)>>5)]&(1<<(p0-1))));p0--) {}
			for(p1=p0+danum-1;p1>vbiti;p1--)
				if (vbit[cast(size_t)p1>>5]&(1<<p1)) goto allocnothere;

			vbiti = p0+danum;
			for(i=p0;i<vbiti;i++) vbit[cast(size_t)(i>>5)] |= (1<<i);
			return(cast(ubyte *)(&vbuf[cast(size_t)p0]));
allocnothere:;
		}
		vbiti = 0; badcnt++;
	} while (badcnt < 2);
	evilquit("voxalloc: vbuf full"); return null;
}

int isvoxelsolid (int x, int y, int z)
{
	ubyte *v;

	if (cast(uint)(x|y) >= VSID) return 0;
	v = sptr[cast(size_t)(y*VSID+x)];
	while (1)
	{
		if (z < v[1]) return(0);
		if (!v[0]) return(1);
		v += v[0]*4;
		if (z < v[3]) return(1);
	}
}

	//Returns 1 if any voxels in range (x,y,z0) to (x,y,z1-1) are solid, else 0
int anyvoxelsolid (int x, int y, int z0, int z1)
{
	ubyte *v;

		//         v1.....v3   v1.....v3    v1.......................>
		//                z0.........z1
	if (cast(uint)(x|y) >= VSID) return(0);
	v = sptr[cast(size_t)(y*VSID+x)];
	while (1)
	{
		if (z1 <= v[1]) return(0);
		if (!v[0]) return(1);
		v += v[0]*4;
		if (z0 < v[3]) return(1);
	}
}

	//Returns 1 if any voxels in range (x,y,z0) to (x,y,z1-1) are empty, else 0
int anyvoxelempty (int x, int y, int z0, int z1)
{
	ubyte *v;

		//         v1.....v3   v1.....v3    v1.......................>
		//                z0.........z1
	if (cast(uint)(x|y) >= VSID) return(1);
	v = sptr[cast(size_t)(y*VSID+x)];
	while (1)
	{
		if (z0 < v[1]) return(1);
		if (!v[0]) return(0);
		v += v[0]*4;
		if (z1 <= v[3]) return(0);
	}
}

	//Returns z of first solid voxel under (x,y,z). Returns z if in solid.
int getfloorz (int x, int y, int z)
{
	ubyte *v;

	if (cast(uint)(x|y) >= VSID) return(z);
	v = sptr[cast(size_t)(y*VSID+x)];
	while (1)
	{
		if (z <= v[1]) return(v[1]);
		if (!v[0]) break;
		v += v[0]*4;
		if (z < v[3]) break;
	}
	return(z);
}

	//Returns:
	//   0: air
	//   1: unexposed solid
	//else: address to color in vbuf (this can never be 0 or 1)
int getcube (int x, int y, int z)
{
	int ceilnum;
	ubyte *v;

	if (cast(uint)(x|y) >= VSID) return(0);
	v = sptr[cast(size_t)(y*VSID+x)];
	while (1)
	{
		if (z <= v[2])
		{
			if (z < v[1]) return(0);
			return(cast(int)&v[cast(size_t)((z-v[1])*4+4)]);
		}
		ceilnum = v[2]-v[1]-v[0]+2;

		if (!v[0]) return(1);
		v += v[0]*4;

		if (z < v[3])
		{
			if (z-v[3] < ceilnum) return(1);
			return(cast(int)&v[cast(size_t)((z-v[3])*4)]);
		}
	}
}

	// Inputs: uind[MAXZDIM]: uncompressed 32-bit color buffer (-1: air)
	//         nind?[MAXZDIM]: neighbor buf:
	//            -2: unexposed solid
	//            -1: air
	//    0-16777215: exposed solid (color)
	//         px,py: parameters for setting unexposed voxel colors
	//Outputs: cbuf[MAXCSIZ]: compressed output buffer
	//Returns: n: length of compressed buffer (in bytes)
int compilestack (int *uind, int *n0, int *n1, int *n2, int *n3, ubyte *cbuf, int px, int py)
{
	int oz, onext, n, cp2, cp1, cp0, rp1, rp0;
	lpoint3d p;

	p.x = px; p.y = py;

		//Do top slab (sky)
	oz = -1;
	p.z = -1; while (uind[cast(size_t)(p.z+1)] == -1) p.z++;
	onext = 0;
	cbuf[1] = cast(ubyte)(p.z+1);
	cbuf[2] = cast(ubyte)(p.z+1);
	cbuf[3] = 0;  //Top z0 (filler, not used yet)
	n = 4;
	cp1 = 1; cp0 = 0;
	rp1 = -1; rp0 = -1;

	do
	{
			//cp2 = state at p.z-1 (0 = air, 1 = next2air, 2 = solid)
			//cp1 = state at p.z   (0 = air, 1 = next2air, 2 = solid)
			//cp0 = state at p.z+1 (0 = air, 1 = next2air, 2 = solid)
		cp2 = cp1; cp1 = cp0; cp0 = 2;
		if (p.z < MAXZDIM-2)  //Bottom must be solid!
		{
			if (uind[cast(size_t)(p.z+1)] == -1)
				cp0 = 0;
			else if ((n0[cast(size_t)(p.z+1)] == -1) || (n1[cast(size_t)(p.z+1)] == -1) ||
						(n2[cast(size_t)(p.z+1)] == -1) || (n3[cast(size_t)(p.z+1)] == -1))
				cp0 = 1;
		}

			//Add slab
		if (cp1 != rp0)
		{
			if ((!cp1) && (rp0 > 0)) { oz = p.z; }
			else if ((rp0 < cp1) && (rp0 < rp1))
			{
				if (oz < 0) oz = p.z;
				cbuf[cast(size_t)onext] = cast(ubyte)(((n-onext)>>2)); onext = n;
				cbuf[cast(size_t)(n+1)] = cast(ubyte)p.z;
				cbuf[cast(size_t)(n+2)] = cast(ubyte)(p.z-1);
				cbuf[cast(size_t)(n+3)] = cast(ubyte)oz;
				n += 4; oz = -1;
			}
			rp1 = rp0; rp0 = cp1;
		}

			//Add color
		if ((cp1 == 1) || ((cp1 == 2) && ((!cp0) || (!cp2))))
		{
			if (cbuf[cast(size_t)(onext+2)] == p.z-1) cbuf[cast(size_t)(onext+2)] = cast(ubyte)p.z;
			if (uind[cast(size_t)p.z] == -2) *cast(int *)&cbuf[cast(size_t)n] = vx5.colfunc(&p);
								 else *cast(int *)&cbuf[cast(size_t)n] = uind[cast(size_t)p.z];
			n += 4;
		}

		p.z++;
	} while (p.z < MAXZDIM);
	cbuf[cast(size_t)onext] = 0;
	return(n);
}

static void expandbit256(void *s, void *d)
{
	asm
	{
		push ESI;
		push EDI;
		mov ESI, s;
		mov EDI, d;
		mov ECX, 32; // current bit index
		xor EDX, EDX; //value of current 32-bit bits
		jmp short in2it;
	begit:
		lea ESI, [ESI+EAX*4];
		movzx EAX, byte ptr [ESI+3];
		sub EAX, ECX; //xor mask [EAX] for ceiling begins		
		jl short xskpc;
	xdoc:
		mov [EDI], EDX;
		add EDI, 4;
		mov EDX, -1;
		add ECX, 32;
		sub EAX, 32;
		jge short xdoc;
	xskpc:
		and EDX, xbsceil[EAX*4+128]; //~(-1<<EAX); xor mask [EAX] for ceiling ends
	in2it:
		movzx EAX, byte ptr [ESI+1];
		sub EAX, ECX; // xor mask [EAX] for floor begins
		jl short xskpf;
	xdof:
		mov [EDI], EDX;
		add EDI, 4;
		xor EDX, EDX;
		add ECX, 32;
		sub EAX, 32;
		jge short xdof;
	xskpf:
		or EDX, xbsflor[EAX*4+128]; //(-1<<EAX); xor mask [EAX] for floor ends
		movzx EAX, byte ptr [ESI];
		test EAX, EAX;
		jnz short begit;
		sub ECX, 256; //finish writing buffer to [edi]
		jg short xskpe;
	xdoe:
		mov [EDI], EDX;
		add EDI, 4;
		mov EDX, -1;
		add ECX, 32;
		jle short xdoe;
	xskpe:
		pop EDI;
		pop ESI;
	}
}

void expandbitstack (int x, int y, long *bind)
{
	if ((x|y)&(~(VSID-1))) { clearbuf(cast(void *)bind,8,0L); return; }
	expandbit256(sptr[cast(size_t)(y*VSID+x)],cast(void *)bind);
}

void expandstack (int x, int y, int *uind)
{
	int z, topz;
	ubyte* v, v2;

	if ((x|y)&(~(VSID-1))) { clearbuf(cast(void *)uind,MAXZDIM,0); return; }

		//Expands compiled voxel info to 32-bit uind[?]
	v = sptr[cast(size_t)(y*VSID+x)]; z = 0;
	while (1)
	{
		while (z < v[1]) { uind[cast(size_t)z] = -1; z++; }
		while (z <= v[2]) { uind[cast(size_t)z] = (*cast(int *)&v[cast(size_t)((z-v[1])*4+4)]); z++; }
		v2 = &v[(v[2]-v[1]+1)*4+4];

		if (!v[0]) break;
		v += v[0]*4;

		topz = v[3]+((cast(int)v2-cast(int)v)>>2);
		while (z < topz) { uind[cast(size_t)z] = -2; z++; }
		while (z < v[3]) { uind[cast(size_t)z] = *cast(int *)v2; z++; v2 += 4; }
	}
	while (z < MAXZDIM) { uind[cast(size_t)z] = -2; z++; }
}

void gline (int leng, float x0, float y0, float x1, float y1)
{
	ulong q;
	float f, f1, f2, vd0, vd1, vz0, vx1, vy1, vz1;
	int j;
	cftype *c;

	static if(USEV5ASM == 0)
	{
		int gx, ogx, gy, ixy, col, dax, day;
		cftype* c2, ce;
		ubyte *v;
	}

	vd0 = x0*gistr.x + y0*gihei.x + gcorn[0].x;
	vd1 = x0*gistr.y + y0*gihei.y + gcorn[0].y;
	vz0 = x0*gistr.z + y0*gihei.z + gcorn[0].z;
	vx1 = x1*gistr.x + y1*gihei.x + gcorn[0].x;
	vy1 = x1*gistr.y + y1*gihei.y + gcorn[0].y;
	vz1 = x1*gistr.z + y1*gihei.z + gcorn[0].z;

	f = sqrt(vx1*vx1 + vy1*vy1);
	f1 = f / vx1;
	f2 = f / vy1;
	if (fabs(vx1) > fabs(vy1)) vd0 = vd0*f1; else vd0 = vd1*f2;
	if (*cast(int *)&vd0 < 0) vd0 = 0; //vd0 MUST NOT be negative: bad for asm
	vd1 = f;
	ftol(fabs(f1)*PREC,&gdz[0]);
	ftol(fabs(f2)*PREC,&gdz[1]);

	gixy[0] = (((*cast(int *)&vx1)>>31)<<3)+4; //=sgn(vx1)*4
	gixy[1] = gixyi[cast(size_t)((*cast(uint *)&vy1)>>31)]; //=sgn(vy1)*4*VSID
	if (gdz[0] <= 0) { ftol(gposxfrac[cast(size_t)((*cast(uint *)&vx1)>>31)]*fabs(f1)*PREC,&gpz[0]); if (gpz[0] <= 0) gpz[0] = 0x7fffffff; gdz[0] = 0x7fffffff-gpz[0]; } //Hack for divide overflow
	else ftol(gposxfrac[cast(size_t)((*cast(uint *)&vx1)>>31)]*cast(float)gdz[0],&gpz[0]);
	if (gdz[1] <= 0) { ftol(gposyfrac[cast(size_t)((*cast(uint *)&vy1)>>31)]*fabs(f2)*PREC,&gpz[1]); if (gpz[1] <= 0) gpz[1] = 0x7fffffff; gdz[1] = 0x7fffffff-gpz[1]; } //Hack for divide overflow
	else ftol(gposyfrac[cast(size_t)((*cast(uint *)&vy1)>>31)]*cast(float)gdz[1],&gpz[1]);

	c = &cf[128];
	c.i0 = gscanptr; c.i1 = &gscanptr[cast(size_t)leng];
	c.z0 = gstartz0; c.z1 = gstartz1;
	if (giforzsgn < 0)
	{
		//ftol((vd1-vd0)*cmprecip[cast(size_t)leng],&gi0); ftol(vd0*CMPPREC,&c.cx0); ((cast(int*)&gi)[0]);
		// Fixed gi1 gi0 problem
		ftol((vd1-vd0)*cmprecip[cast(size_t)leng],&((cast(int*)&gi)[0])); ftol(vd0*CMPPREC,&c.cx0);
		ftol((vz1-vz0)*cmprecip[cast(size_t)leng],&((cast(int*)&gi)[1])); ftol(vz0*CMPPREC,&c.cy0);
	}
	else
	{
		ftol((vd0-vd1)*cmprecip[cast(size_t)leng],&((cast(int*)&gi)[0])); ftol(vd1*CMPPREC,&c.cx0);
		ftol((vz0-vz1)*cmprecip[cast(size_t)leng],&((cast(int*)&gi)[1])); ftol(vz1*CMPPREC,&c.cy0);
	}
	c.cx1 = leng*gi0 + c.cx0;
	c.cy1 = leng*gi1 + c.cy0;

	gxmax = gmaxscandist;

		//Hack for early-out case when looking up towards sky
	static if(0) //DOESN'T WORK WITH LOWER MIPS!
	{  
		if (c.cy1 < 0)
			if (gposz > 0)
			{
				if (dmulrethigh(-gposz,c.cx1,c.cy1,gxmax) >= 0)
				{
					j = scale(-gposz,c.cx1,c.cy1)+PREC; //+PREC for good luck
					if (cast(uint)j < cast(uint)gxmax) gxmax = j;
				}
			} else gxmax = 0;
	}

		//Clip borders safely (MUST use integers!) - don't wrap around
	static if((USEZBUFFER == 1) && (USEV5ASM != 0))
	{
		skycast.dist = gxmax;
	}

	if (gixy[0] < 0) j = glipos.x; else j = VSID-1-glipos.x;
	q = mul64(gdz[0],j); q += cast(ulong)gpz[0];
	if (q < cast(ulong)gxmax)
	{
		gxmax = cast(int)q;
		static if((USEZBUFFER == 1) && (USEV5ASM != 0))
		{
			skycast.dist = 0x7fffffff;
		}
	}
	if (gixy[1] < 0) j = glipos.y; else j = VSID-1-glipos.y;
	q = mul64(gdz[1],j); q += cast(ulong)gpz[1];
	if (q < cast(ulong)gxmax)
	{
		gxmax = cast(int)q;
		static if((USEZBUFFER == 1) && (USEV5ASM != 0))
		{
			skycast.dist = 0x7fffffff;
		}	
	}

	if (vx5.sideshademode)
	{
		gcsub[0] = gcsub[cast(size_t)(((cast(uint)gixy[0])>>31)+4)];
		gcsub[1] = gcsub[cast(size_t)(((cast(uint)gixy[1])>>31)+6)];
	}

	static if(USEV5ASM)
	{
		if (nskypic)
		{
			if (skycurlng < 0)
			{
				ftol((atan2(vy1,vx1)+PI)*skylngmul-.5,&skycurlng);
				if (cast(uint)skycurlng >= skyysiz)
					skycurlng = ((skyysiz-1)&(j>>31));
			}
			else if (skycurdir < 0)
			{
				j = skycurlng+1; if (j >= skyysiz) j = 0;
				while (skylng[cast(size_t)j].x*vy1 > skylng[cast(size_t)j].y*vx1)
					{ skycurlng = j++; if (j >= skyysiz) j = 0; }
			}
			else
			{
				while (skylng[cast(size_t)skycurlng].x*vy1 < skylng[cast(size_t)skycurlng].y*vx1)
					{ skycurlng--; if (skycurlng < 0) skycurlng = skyysiz-1; }
			}
			skyoff = skycurlng*skybpl + nskypic;
		}

		//resp = 0;
		grouscanasm(cast(int)gstartv);
		//if (resp)
		//{
		//   static ubyte tempbuf[2048], tempbuf2[256];
		//   sprintf(tempbuf,"EAX:%08x\tMM0:%08x%08x\nebx:%08x\tMM1:%08x%08x\necx:%08x\tMM2:%08x%08x\nedx:%08x\tmm3:%08x%08x\nesi:%08x\tmm4:%08x%08x\nedi:%08x\tmm5:%08x%08x\nebp:%08x\tmm6:%08x%08x\nesp:%08x\tmm7:%08x%08x\n",
		//      rEAX,remm[ 1],remm[ 0], rebx,remm[ 3],remm[ 2],
		//      recx,remm[ 5],remm[ 4], redx,remm[ 7],remm[ 6],
		//      resi,remm[ 9],remm[ 8], redi,remm[11],remm[10],
		//      rebp,remm[13],remm[12], resp,remm[15],remm[14]);
		//
		//   for(j=0;j<3;j++)
		//   {
		//      sprintf(tempbuf2,"%d i0:%d i1:%d z0:%ld z1:%ld cx0:%08x cy0:%08x cx1:%08x cy1:%08x\n",
		//         j,(int)cf[j].i0-(int)gscanptr,(int)cf[j].i1-(int)gscanptr,cf[j].z0,cf[j].z1,cf[j].cx0,cf[j].cy0,cf[j].cx1,cf[j].cy1);
		//      strcat(tempbuf,tempbuf2);
		//   }
		//   evilquit(tempbuf);
		//}
	} else
	{
	//------------------------------------------------------------------------
		ce = c; v = gstartv;
		j = ((cast(uint)(gpz[1]-gpz[0]))>>31);
		gx = gpz[cast(size_t)j];
		ixy = gpixy;
		if (v == cast(ubyte *)*cast(int *)gpixy) goto drawflor; goto drawceil;

		while (1)
		{

	drawfwall:;
			if (v[1] != c.z1)
			{
				if (v[1] > c.z1) c.z1 = v[1];
				else { do
				{
					c.z1--; col = *cast(int *)&v[cast(size_t)((c.z1-v[1])*4+4)];
					while (dmulrethigh(gylookup[c.z1],c.cx1,c.cy1,ogx) < 0)
					{
						c.i1.col = col; c.i1--; if (c.i0 > c.i1) goto deletez;
						c.cx1 -= gi0; c.cy1 -= gi1;
					}
				} while (v[1] != c.z1); }
			}

			if (v == cast(ubyte *)*cast(int *)ixy) goto drawflor;

	//drawcwall:;
			if (v[3] != c.z0)
			{
				if (v[3] < c.z0) c.z0 = v[3];
				else { do
				{
					c.z0++; col = *cast(int *)&v[cast(size_t)((c.z0-v[3])*4-4)];
					while (dmulrethigh(gylookup[c.z0],c.cx0,c.cy0,ogx) >= 0)
					{
						c.i0.col = col; c.i0++; if (c.i0 > c.i1) goto deletez;
						c.cx0 += gi0; c.cy0 += gi1;
					}
				} while (v[3] != c.z0); }
			}

	drawceil:;
			while (dmulrethigh(gylookup[c.z0],c.cx0,c.y0,gx) >= 0)
			{
				c.i0.col = (*cast(int *)&v[-4]); c.i0++; if (c.i0 > c.i1) goto deletez;
				c.cx0 += gi0; c.cy0 += gi1;
			}

	drawflor:;
			while (dmulrethigh(gylookup[c.z1],c.cx1,c.cy1,gx) < 0)
			{
				c.i1.col = *cast(int *)&v[4]; c.i1--; if (c.i0 > c.i1) goto deletez;
				c.cx1 -= gi0; c.cy1 -= gi1;
			}

	afterdelete:;
			c--;
			if (c < &cf[128])
			{
				ixy += gixy[cast(size_t)j];
				gpz[cast(size_t)j] += gdz[cast(size_t)j];
				j = ((cast(uint)(gpz[1]-gpz[0]))>>31);
				ogx = gx; gx = gpz[cast(size_t)j];

				if (gx > gxmax) break;
				v = cast(ubyte *)*cast(int *)ixy; c = ce;
			}
				//Find highest intersecting vbuf slab
			while (1)
			{
				if (!v[0]) goto drawfwall;
				if (dmulrethigh(gylookup[cast(size_t)(v[2]+1)],c.cx0,c.cy0,ogx) >= 0) break;
				v += v[0]*4;
			}
				//If next slab ALSO intersects, split cf!
			gy = gylookup[cast(size_t)v[cast(size_t)(v[0]*4+3)]];
			if (dmulrethigh(gy,c.cx1,c.cy1,ogx) < 0)
			{
				col = cast(int)c.i1; dax = c.cx1; day = c.cy1;
				while (dmulrethigh(gylookup[cast(size_t)(v[2]+1)],dax,day,ogx) < 0)
					{ col -= castdat.sizeof; dax -= gi0; day -= gi1; }
				ce++; if (ce >= &cf[192]) return; //Give it max=64 entries like ASM
				for(c2=ce;c2>c;c2--) c2[0] = c2[-1];
				c[1].i1 = cast(castdat *)col; c.i0 = (cast(castdat *)col)+1;
				c[1].cx1 = dax; c.cx0 = dax+gi0;
				c[1].cy1 = day; c.cy0 = day+gi1;
				c[1].z1 = c.z0 = v[cast(size_t)(v[0]*4+3)];
				c++;
			}
		}
	//------------------------------------------------------------------------

		for(c=ce;c>=&cf[128];c--)
			while (c.i0 <= c.i1) { c.i0.col = 0; c.i0++; }
		return;

	deletez:;
		ce--; if (ce < &cf[128]) return;
		for(c2=c;c2<=ce;c2++) c2[0] = c2[1];
		goto afterdelete;
	}
}

static void mmxcoloradd(int *a)
{
	asm
	{
		mov EAX, a;
		movd MM0, [EAX];
		paddusb MM0, flashbrival;
		movd [EAX], MM0;
	}
}

static void mmxcolorsub(int *a)
{
	asm
	{
		mov EAX, a;
		movd MM0, [EAX];
		psubusb MM0, flashbrival;
		movd [EAX], MM0;
	}
}

static void addusb(ubyte *a, int b)
{
	(*a) += b; if ((*a) < b) (*a) = 255;
}

	// (cone diameter vs. % 3D angular area) or: (a vs. 2/(1-cos(a*.5*PI/180)))
	// ÚÄÄÄÄÄÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄÄÄÄÂÄÄÄÄÄÄÄÄÄ¿
	// ³  0: inf     ³ 25: 84.37 ³ 50: 21.35 ³ 75: 9.68 ³ 100: 5.60 ³ 180: 2  ³
	// ³  5: 2101.33 ³ 30: 58.70 ³ 55: 17.70 ³ 80: 8.55 ³ 105: 5.11 ³ 360: 1  ³
	// ³ 10:  525.58 ³ 35: 43.21 ³ 60: 14.93 ³ 85: 7.61 ³ 110: 4.69 ÃÄÄÄÄÄÄÄÄÄÙ
	// ³ 15:  233.78 ³ 40: 33.16 ³ 65: 12.77 ³ 90: 6.83 ³ 115: 4.32 ³
	// ³ 20:  131.65 ³ 45: 26.27 ³ 70: 11.06 ³ 95: 6.17 ³ 120: 4    ³
	// ÀÄÄÄÄÄÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÄÄÁÄÄÄÄÄÄÄÄÄÄÄÙ
void setflash (float px, float py, float pz, int flashradius, int numang, int intens)
{
	ulong q;
	float vx, vy;
	int i, j, gx, ogx, ixy, col, angoff;
	int ipx, ipy, ipz, sz0, sz1;
	cftype* c, c2, ce;
	ubyte* v, vs;

	ipx = cast(int)px; ipy = cast(int)py; ipz = cast(int)pz;
	vx5.minx = ipx-flashradius; vx5.maxx = ipx+flashradius+1;
	vx5.miny = ipy-flashradius; vx5.maxy = ipy+flashradius+1;
	vx5.minz = ipz-flashradius; vx5.maxz = ipz+flashradius+1;

	if (flashradius > 2047) flashradius = 2047;
	flashradius *= FPREC;

	flashbrival = (intens<<24);
	angoff = ((0x52741630>>(flashcnt<<2))&15); flashcnt++;

	gposxfrac[1] = px - cast(float)(ipx); gposxfrac[0] = 1 - gposxfrac[1];
	gposyfrac[1] = py - cast(float)(ipy); gposyfrac[0] = 1 - gposyfrac[1];
	gpixy = cast(int)&sptr[cast(size_t)(ipy*VSID + ipx)];
	ftol(pz*FPREC-.5f,&gposz);
	for(gylookup[0]=-gposz,i=1;i<260;i++) gylookup[cast(size_t)i] = gylookup[cast(size_t)(i-1)]+FPREC;

	vs = cast(ubyte *)*cast(int *)gpixy;
	if (ipz >= vs[1])
	{
		do
		{
			if (!vs[0]) return;
			vs += vs[0]*4;
		} while (ipz >= vs[1]);
		if (ipz < vs[3]) return;
		sz0 = vs[3];
	} else sz0 = 0;
	sz1 = vs[1];

	for(i=0;i<numang;i++)
	{
		asm {emms;}

		fcossin((cast(float)i+cast(float)angoff*.125f)*PI*2.0f/cast(float)numang,&vx,&vy);

		ftol(FPREC/fabs(vx),&gdz[0]);
		ftol(FPREC/fabs(vy),&gdz[1]);

		gixy[0] = (((*cast(int *)&vx)>>31) & (     -8)) +      4;
		gixy[1] = (((*cast(int *)&vy)>>31) & (VSID*-8)) + VSID*4;
		if (gdz[0] < 0) { gpz[0] = 0x7fffffff; gdz[0] = 0; } //Hack for divide overflow
		else ftol(gposxfrac[cast(size_t)((*cast(uint *)&vx)>>31)]*cast(float)gdz[0],&gpz[0]);
		if (gdz[1] < 0) { gpz[1] = 0x7fffffff; gdz[1] = 0; } //Hack for divide overflow
		else ftol(gposyfrac[cast(size_t)((*cast(uint *)&vy)>>31)]*cast(float)gdz[1],&gpz[1]);

		c = ce = &cf[128];
		v = vs; c.z0 = sz0; c.z1 = sz1;
			//Note!  These substitions are used in flashscan:
			//   c.i0 in flashscan is now: c.cx0
			//   c.i1 in flashscan is now: c.cx1
		c.cx0 = (((i+flashcnt+rand())&7)<<LOGFLASHVANG);
		c.cx1 = c.cx0+(1<<LOGFLASHVANG)-1;

		gxmax = flashradius;

			//Clip borders safely (MUST use integers!) - don't wrap around
		if (gixy[0] < 0) j = ipx; else j = VSID-1-ipx;
		q = mul64(gdz[0],j); q += cast(ulong)gpz[0];
		if (q < cast(ulong)gxmax) gxmax = cast(int)q;
		if (gixy[1] < 0) j = ipy; else j = VSID-1-ipy;
		q = mul64(gdz[1],j); q += cast(ulong)gpz[1];
		if (q < cast(ulong)gxmax) gxmax = cast(int)q;

	//------------------------------------------------------------------------
		j = ((cast(uint)(gpz[1]-gpz[0]))>>31);
		gx = gpz[cast(size_t)j];
		ixy = gpixy;
		if (v == cast(ubyte *)*cast(int *)gpixy) goto fdrawflor; goto fdrawceil;

		while (1)
		{

fdrawfwall:;
			if (v[1] != c.z1)
			{
				if (v[1] > c.z1) c.z1 = v[1];
				else { do
				{
					c.z1--; col = cast(int)&v[cast(size_t)((c.z1-v[1])*4+4)];
					while (dmulrethigh(gylookup[cast(size_t)c.z1],gfc[cast(size_t)c.cx1].x,gfc[cast(size_t)c.cx1].y,ogx) < 0)
					{
						mmxcoloradd(cast(int *)col); c.cx1--;
						if (c.cx0 > c.cx1) goto fdeletez;
					}
				} while (v[1] != c.z1); }
			}

			if (v == cast(ubyte *)*cast(int *)ixy) goto fdrawflor;

//fdrawcwall:;
			if (v[3] != c.z0)
			{
				if (v[3] < c.z0) c.z0 = v[3];
				else { do
				{
					c.z0++; col = cast(int)&v[cast(size_t)((c.z0-v[3])*4-4)];
					while (dmulrethigh(gylookup[cast(size_t)c.z0],gfc[cast(size_t)c.cx0].x,gfc[cast(size_t)c.cx0].y,ogx) >= 0)
					{
						mmxcoloradd(cast(int *)col); c.cx0++;
						if (c.cx0 > c.cx1) goto fdeletez;
					}
				} while (v[3] != c.z0); }
			}

fdrawceil:;
			while (dmulrethigh(gylookup[cast(size_t)c.z0],gfc[cast(size_t)c.cx0].x,gfc[cast(size_t)c.cx0].y,gx) >= 0)
			{
				mmxcoloradd(cast(int *)&v[-4]); c.cx0++;
				if (c.cx0 > c.cx1) goto fdeletez;
			}

fdrawflor:;
			while (dmulrethigh(gylookup[cast(size_t)c.z1],gfc[cast(size_t)c.cx1].x,gfc[cast(size_t)c.cx1].y,gx) < 0)
			{
				mmxcoloradd(cast(int *)&v[4]); c.cx1--;
				if (c.cx0 > c.cx1) goto fdeletez;
			}

fafterdelete:;
			c--;
			if (c < &cf[128])
			{
				ixy += gixy[cast(size_t)j];
				gpz[cast(size_t)j] += gdz[cast(size_t)j];
				j = ((cast(uint)(gpz[1]-gpz[0]))>>31);
				ogx = gx; gx = gpz[cast(size_t)j];

				if (gx > gxmax) break;
				v = cast(ubyte *)*cast(int *)ixy; c = ce;
			}
				//Find highest intersecting vbuf slab
			while (1)
			{
				if (!v[0]) goto fdrawfwall;
				if (dmulrethigh(gylookup[v[2]+1],gfc[cast(size_t)c.cx0].x,gfc[cast(size_t)c.cx0].y,ogx) >= 0) break;
				v += v[0]*4;
			}
				//If next slab ALSO intersects, split cf!
			if (dmulrethigh(gylookup[cast(size_t)(v[cast(size_t)(v[0]*4+3)])],gfc[cast(size_t)c.cx1].x,gfc[cast(size_t)c.cx1].y,ogx) < 0)
			{
				col = c.cx1;
				while (dmulrethigh(gylookup[cast(size_t)(v[2]+1)],gfc[cast(size_t)col].x,gfc[cast(size_t)col].y,ogx) < 0)
					col--;
				ce++; if (ce >= &cf[192]) break; //Give it max=64 entries like ASM
				for(c2=ce;c2>c;c2--) c2[0] = c2[-1];
				c[1].cx1 = col; c.cx0 = col+1;
				c[1].z1 = c.z0 = v[cast(size_t)(v[0]*4+3)];
				c++;
			}
		}
fcontinue:;
	}

	asm {emms;}
	updatebbox(vx5.minx,vx5.miny,vx5.minz,vx5.maxx,vx5.maxy,vx5.maxz,0);
	return;

fdeletez:;
	ce--; if (ce < &cf[128]) goto fcontinue;
	for(c2=c;c2<=ce;c2++) c2[0] = c2[1];
	goto fafterdelete;
}

static if(ESTNORMRAD == 2)
{
	static byte bitnum[32] =
	[
		0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4,
		1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5	
	];

	//static int bitsum[32] =
	//[
	//   0,-2,-1,-3, 0,-2,-1,-3, 1,-1, 0,-2, 1,-1, 0,-2,
	//   2, 0, 1,-1, 2, 0, 1,-1, 3, 1, 2, 0, 3, 1, 2, 0
	//];

	static int bitsnum[32] =
	[
		0        ,1-(2<<16),1-(1<<16),2-(3<<16),
		1        ,2-(2<<16),2-(1<<16),3-(3<<16),
		1+(1<<16),2-(1<<16),2        ,3-(2<<16),
		2+(1<<16),3-(1<<16),3        ,4-(2<<16),
		1+(2<<16),2        ,2+(1<<16),3-(1<<16),
		2+(2<<16),3        ,3+(1<<16),4-(1<<16),
		2+(3<<16),3+(1<<16),3+(2<<16),4,
		3+(3<<16),4+(1<<16),4+(2<<16),5
	];

	static float fsqrecip[5860]; //75*75 + 15*15 + 3*3 = 5859 is max value (5*5*5 box)
}

void estnorm(int x, int y, int z, point3d *fp)
{
	lpoint3d n;
	int *lptr;
	int xx, yy, zz, i, j, k;
	int b[5];
	float f;

	n.x = 0; n.y = 0; n.z = 0;

	static if(ESTNORMRAD == 2)
	{
		if (labs(cast(int)(x-xbsox)) + labs(cast(int)(y-xbsoy)) > 1)
		{
				//x,y not close enough to cache: calls expandbitstack 25 times :(
			xbsox = x; xbsoy = y; xbsof = 24*5;
			lptr = cast(int *)(&xbsbuf[24*5+1]);
			for(yy=-2;yy<=2;yy++)
				for(xx=-2;xx<=2;xx++,lptr-=10)
					expandbitstack(x+xx,y+yy,cast(long *)lptr);
		}
		else if (x != xbsox)
		{
				//shift xbsbuf cache left/right: calls expandbitstack 5 times :)
			if (x < xbsox) { xx = -2; xbsof -= 24*5; lptr = cast(int *)(&xbsbuf[cast(size_t)(xbsof+1)]); }
						 else { xx = 2; lptr = cast(int *)(&xbsbuf[cast(size_t)(xbsof-5*5+1)]); xbsof -= 1*5; }
			xbsox = x; if (xbsof < 0) xbsof += 25*5;
			for(yy=-2;yy<=2;yy++)
			{
				if (lptr < cast(int *)&xbsbuf[1]) lptr += 25*10;
				expandbitstack(x+xx,y+yy,cast(long *)lptr);
				lptr -= 5*10;
			}
		}
		else if (y != xbsoy)
		{
				//shift xbsbuf cache up/down: calls expandbitstack 5 times :)
			if (y < xbsoy) { yy = -2; xbsof -= 20*5; lptr = cast(int *)(&xbsbuf[cast(size_t)(xbsof+1)]); }
						 else { yy = 2; lptr = cast(int *)(&xbsbuf[cast(size_t)(xbsof+1)]); xbsof -= 5*5; }
			xbsoy = y; if (xbsof < 0) xbsof += 25*5;
			for(xx=-2;xx<=2;xx++)
			{
				if (lptr < cast(int *)&xbsbuf[1]) lptr += 25*10;
				expandbitstack(x+xx,y+yy,cast(long *)lptr);
				lptr -= 1*10;
			}
		}

		z -= 2;
		if ((z&31) <= 27) //2 <= (z&31) <= 29
			{ lptr = cast(int *)(cast(int)(&xbsbuf[cast(size_t)(xbsof+1)]) + ((z&~31)>>3)); z &= 31; }
		else
			{ lptr = cast(int *)(cast(int)(&xbsbuf[cast(size_t)(xbsof+1)]) + (z>>3)); z &= 7; }

		for(yy=-2;yy<=2;yy++)
		{
			if (lptr >= cast(int *)&xbsbuf[1+10*5])
			{
				b[0] = ((lptr[  0]>>z)&31); b[1] = ((lptr[-10]>>z)&31);
				b[2] = ((lptr[-20]>>z)&31); b[3] = ((lptr[-30]>>z)&31);
				b[4] = ((lptr[-40]>>z)&31); lptr -= 50;
			}
			else
			{
				b[0] = ((lptr[0]>>z)&31); lptr -= 10; if (lptr < cast(int *)&xbsbuf[1]) lptr += 25*10;
				b[1] = ((lptr[0]>>z)&31); lptr -= 10; if (lptr < cast(int *)&xbsbuf[1]) lptr += 25*10;
				b[2] = ((lptr[0]>>z)&31); lptr -= 10; if (lptr < cast(int *)&xbsbuf[1]) lptr += 25*10;
				b[3] = ((lptr[0]>>z)&31); lptr -= 10; if (lptr < cast(int *)&xbsbuf[1]) lptr += 25*10;
				b[4] = ((lptr[0]>>z)&31); lptr -= 10; if (lptr < cast(int *)&xbsbuf[1]) lptr += 25*10;
			}

				//Make filter spherical
			//if (yy&1) { b[0] &= 0xe; b[4] &= 0xe; }
			//else if (yy) { b[0] &= 0x4; b[1] &= 0xe; b[3] &= 0xe; b[4] &= 0x4; }

			n.x += ((bitnum[cast(size_t)b[4]]-bitnum[cast(size_t)b[0]])<<1)+bitnum[cast(size_t)b[3]]-bitnum[cast(size_t)b[1]];
			j = bitsnum[cast(size_t)b[0]]+bitsnum[cast(size_t)b[1]]+bitsnum[cast(size_t)b[2]]+bitsnum[cast(size_t)b[3]]+bitsnum[cast(size_t)b[4]];
			n.z += j; n.y += (*cast(short *)&j)*yy;
		}
		n.z >>= 16;
	} else
	{
		for(yy=-ESTNORMRAD;yy<=ESTNORMRAD;yy++)
			for(xx=-ESTNORMRAD;xx<=ESTNORMRAD;xx++)
				for(zz=-ESTNORMRAD;zz<=ESTNORMRAD;zz++)
					if (isvoxelsolid(x+xx,y+yy,z+zz))
						{ n.x += xx; n.y += yy; n.z += zz; }
	}

	static if(1)
	{
		f = fsqrecip[cast(size_t)(n.x*n.x + n.y*n.y + n.z*n.z)];
		fp.x = (cast(float)n.x)*f; fp.y = (cast(float)n.y)*f; fp.z = (cast(float)n.z)*f;
	} else
	{

		//f = 1.0 / sqrt((double)(n.x*n.x + n.y*n.y + n.z*n.z));
		//fp.x = f*(float)n.x; fp.y = f*(float)n.y; fp.z = f*(float)n.z;
		zz = n.x*n.x + n.y*n.y + n.z*n.z;
		if (cputype&(1<<25))
		{
			asm
			{
				cvtsi2ss XMM0, zz;
				rsqrtss XMM0, XMM0;
				//movss f, XMM0;

					//fp.x = f*(float)n.x; fp.y = f*(float)n.y; fp.z = f*(float)n.z;
				cvtsi2ss XMM1, n.z;
				shufps XMM0, XMM0, 0;
				mov EAX, fp;
				movlhps XMM1, XMM1;
				cvtpi2ps XMM1, n;
				mulps XMM0, XMM1;
				movlps [EAX], XMM0;
				movhlps XMM0, XMM0;
				movss [EAX+8], XMM0;
			}
		}
		else
		{
			asm
			{
				pi2fd MM0, zz;       //MM0:     0          zz
				pfrsqrt MM0, MM0;    //MM0: 1/sqrt(zz) 1/sqrt(zz)
				pi2fd MM1, n.x;      //MM1:     0         n.x
				pi2fd MM2, n.y;      //MM2:     0         n.y
				punpckldq MM1, MM2;  //MM1:    n.y        n.x
				pi2fd MM2, n.z;      //MM2:     0         n.z
				pfmul MM1, MM0;      //MM1:n.y/sqrt(zz) n.x/sqrt(zz)
				pfmul MM2, MM0;      //MM2:     0       n.z/sqrt(zz)
				mov EAX, fp;
				movq [EAX], MM1;
				movd [EAX+8], MM2;
				femms;
			}
		}
	}
}

static int vspan(int x, int y0, int y1)
{
	int y, yy;
	int* bbufx;

	y = (y0>>5); bbufx = &bbuf[x][0];
	if ((y1>>5) == y)
	{
		yy = bbufx[y]; bbufx[y] &= ~(p2m[y1&31]^p2m[y0&31]);
		return(bbufx[y] ^ yy);
	}

	if (!(bbufx[y]&(~p2m[y0&32])))
		if(!(bbufx[y1>>5]&p2m[y1&32]))
		{
			for(yy=(y1>>5)-1;yy>y;yy--)
				if(bbufx[yy]) goto vspan_skip;
			return 0;
		}
	vspan_skip:
	bbufx[y] &= p2m[y0&31];
	bbufx[y1>>5] &= (~p2m[y1&32]);
	for(yy=(y1>>5)-1; yy>y; yy--) bbufx[yy] = 0;

	return 1;
}

// line: 1754 docube
static int docube(int x, int y, int z)
{
	int x0, y0, x1, y1, g;

	ffxptr = &ffx[(z+1)*z-1];
	x0 = cast(int)ffxptr[x].x; x1 = cast(int)ffxptr[x].y;
	y0 = cast(int)ffxptr[y].x; y1 = cast(int)ffxptr[y].y;
	for(g=0;x0<x1;x0++) g |=vspan(x0,y0,y1);
	return g;
}

void setnormflash(float px, float py, float pz, int flashradius, int intens)
{
	point3d fp;
	float f, fintens;
	int i, j, k, l, m, x, y, z, xx, yy, xi, yi, xe, ye, ipx, ipy, ipz;
	int ceilnum, sq;
	ubyte *v;

	ipx = cast(int)px; ipy = cast(int)py; ipz = cast(int)pz;
	vx5.minx = ipx - flashradius+1; vx5.maxx = ipx+flashradius;
	vx5.miny = ipy - flashradius+1; vx5.maxy = ipy+flashradius;
	vx5.minz = ipz - flashradius+1; vx5.maxz = ipz+flashradius;

	if(isvoxelsolid(ipx, ipy, ipz)) return;

	fintens = intens;
	if(flashradius > (GSIZ>>1)) flashradius = (GSIZ >> 1);

	xbsox = -17;

	//       ÚÄ 7Ä¿
	//      11  . 8
	//  ÚÄ11ÄÅÄ 4ÄÅÄ 8ÄÂÄ 7Ä¿
	//  3 |  0  + 1  | 2  + 3
	//  ÀÄ10ÄÅÄ 5ÄÅÄ 9ÄÁÄ 6ÄÙ
	//      10  . 9
	//       ÀÄ 6ÄÙ

	//Do left&right faces of the cube	
	for(j=1; j>=0; j--)
	{
		clearbuf(cast(void*)bbuf, GSIZ*(GSIZ>>5), 0xffffffff);
		for(y=1; y<flashradius;y++)
		{
			if(j) yy = ipy-y; else yy = ipy + y;
			for(xi=1,xe=y+1;xi>=-1;xi-=2,xe=-xe)
				for(x=(xi>>1);x!=xe;x+=xi)
				{
					xx = ipx+x;
					if(cast(uint)(xx|yy) >= VSID) continue;
					v = sptr[yy*VSID+xx]; i=0; sq=x*x+y*y;
					while(true)
					{
						for(z=v[1];z<=v[2];z++)
						{
							if(z-ipz < 0) { tbuf2[i] = z-ipz; tbuf2[i+1] = cast(int)&v[(z-v[1])*4+4]; i+= 2;}
							else
							{
								//if (z-ipz < -y) continue; //TEMP HACK!!!
								if (z-ipz > y) goto normflash_exwhile1;
								if (!docube(x,z-ipz,y)) continue;
								estnorm(xx,yy,z,&fp); if(j) fp.y = -fp.y;
								f = fp.x*x + fp.y*y + fp.z*(z-ipz);
								if(*cast(int *)&f > 0) addusb(&v[(z-v[1])*4+7], cast(int)(f*fintens/((z-ipz)*(z-ipz)+sq)));
							}
						}
						if (!v[0]) break;
						ceilnum = v[2]-v[1]-v[0]+2; v += v[0]*4;
						for(z=v[3]+ceilnum;z<v[3];z++)
						{
							if (z < ipz) { tbuf2[i] = z-ipz; tbuf2[i+1] = cast(int)&v[(z-v[3])*4]; i += 2; }
							else
							{
								//if (z-ipz < -y) continue; // TEMP HACK!!!
								if(z-ipz > y) goto normflash_exwhile1;
								if (!docube(x,z-ipz,y)) continue;
								estnorm(xx, yy, z, &fp); if(j) fp.y = -fp.y;
								f = fp.x*x + fp.y*y + fp.z*(z-ipz);
								if (*cast(int *)&f > 0) addusb(&v[(z-v[3])*4+3], cast(int)(f*fintens/((z-ipz)*(z-ipz)+sq)));
							}
						}
					}
					normflash_exwhile1:
					while(i > 0)
					{
						i -= 2; if(tbuf2[i] < -y) break;
						if (!docube(x,tbuf2[i],y)) continue;
						estnorm(xx, yy, tbuf2[i]+ipz, &fp); if (j) fp.y = -fp.y;
						f = fp.x*x + fp.y*y + fp.z*tbuf2[i];
						if (*cast(int *)&f > 0) addusb(&(cast(ubyte*)tbuf2[i+1])[3], cast(int)(f*fintens/(tbuf2[i]*tbuf2[i]+sq)));
					}
				}

		}
	}

	// Do up&down faces of the cube
	for(j=1; j>=0; j--)
	{
		clearbuf(cast(void*)bbuf, GSIZ*(GSIZ>>5), 0xffffffff);
		for(y=1;y<flashradius;y++)
		{
			if(j) xx = ipx-y; else xx = ipx+y;
			for(xi=1, xe=y+1; xi>=-1; xi-=2, xe=-xe)
				for(x=(xi>>1);x!=xe;x+=xi)
				{
					yy = ipy+x;
					if (cast(uint)(xx|yy) >= VSID) continue;
					v = sptr[yy*VSID+xx]; i = 0; sq = x*x+y*y; m = x+xi-xe;
					while(true)
					{
						for(z=v[1];z<=v[2];z++)
						{
							if(z-ipz < 0) { tbuf2[i] = z-ipz; tbuf2[i+1] = cast(int)&v[(z-v[1])*4+4]; i += 2; }
							else
							{
								//if (z-ipz < -y) continue; //TEMP HACK!!!
								if (z-ipz > y) goto normflash_exwhile2;
								if ((!docube(x,z-ipz,y)) || (!m)) continue;
								estnorm(xx,yy,z,&fp); if(j) fp.x = -fp.x;
								f = fp.x*y + fp.y*x +fp.z*(z-ipz);
								if (*cast(int *)&f > 0) addusb(&v[(z-v[1])*4+7],cast(int)(f*fintens/((z-ipz)*(z-ipz)+sq)));
							}
						}
						if (!v[0]) break;
						ceilnum = v[2]-v[1]-v[0]+2; v += v[0]*4;
						for(z=v[3]+ceilnum;z<v[3];z++)
						{
							if (z < ipz) { tbuf2[i] = z-ipz; tbuf2[i+1] = cast(int)&v[(z-v[3])*4]; i += 2;}
							else
							{
								//if (z-ipz < -y) continue; //TEMP HACK!!!
								if (z-ipz > y) goto normflash_exwhile2;
								if ((!docube(x,z-ipz,y)) || (!m)) continue;
								estnorm(xx,yy,z,&fp); if(j) fp.x = -fp.x;
								f = fp.x*y + fp.y*x + fp.z*(z-ipz);
								if (*cast(int*)&f > 0) addusb(&v[(z-v[3])*4+3], cast(int)(f*fintens/((z-ipz)*(z-ipz)+sq)));
							}
						}
					}
					normflash_exwhile2:
					while(i>0)
					{
						i -= 2; if(tbuf2[i] < -y) break;
						if ((!docube(x,tbuf2[i],y)) || (!m)) continue;
						estnorm(xx,yy,tbuf2[i]+ipz,&fp); if (j) fp.x = -fp.x;
						f = fp.x*y + fp.y*x + fp.z*tbuf2[i];
						if (*cast(int *)&f > 0) addusb(&(cast(ubyte *)tbuf2[i+1])[3],cast(int)(f*fintens/(tbuf2[i]*tbuf2[i]+sq)));
					}
				}
		}

		//Do the bottom face of the cube
		clearbuf(cast(void *)bbuf, GSIZ*(GSIZ>>5), 0xffffffff);
		for(yi=1,ye=flashradius+1;yi>=-1;yi-=2,ye=-ye)
			for(y=(yi>>1);y!=ye;y+=yi)
				for(xi=1,xe=flashradius+1;xi>=-1;xi-=2,xe=-xe)
					for(x=(xi>>1);x!=xe;x+=xi)
					{
						xx = ipx+x; yy = ipy+y;						
						if (cast(uint)(xx|yy) >= VSID) goto normflash_exwhile3;
						k = max(labs(x),labs(y));

						v = sptr[yy*VSID+xx]; sq = x*x+y*y;
						while(true)
						{
							for(z=v[1];z<=v[2];z++)
							{
								if (z-ipz < k) continue;
								if (z-ipz >= flashradius) goto normflash_exwhile3;
								if ((!docube(x,y,z-ipz)) || (z-ipz == k)) continue;
								estnorm(xx,yy,z,&fp);
								f = fp.x*x + fp.y*y + fp.z*(z-ipz);
								if (*cast(int *)&f > 0) addusb(&v[(z-v[1])*4+7],cast(int)(f*fintens/((z-ipz)*(z-ipz)+sq)));
							}
							if (!v[0]) break;
							ceilnum = v[2]-v[1]-v[0]+2; v += v[0]*4;
							for(z=v[3]+ceilnum;z<v[3];z++)
							{
								if (z-ipz < k) continue;
								if (z-ipz >= flashradius) goto normflash_exwhile3;
								if ((!docube(x,y,z-ipz)) || (z-ipz <= k)) continue;
								estnorm(xx,yy,z,&fp);
								f = fp.x*x + fp.y*y + fp.z*(z-ipz);
								if (*cast(int *)&f > 0) addusb(&v[(z-v[3])*4+3],cast(int)(f*fintens/((z-ipz)*(z-ipz)+sq)));
							}
						}
						normflash_exwhile3:;
					}

			//Do the top face of the cube
	clearbuf(cast(void *)bbuf,GSIZ*(GSIZ>>5),0xffffffff);
	for(yi=1,ye=flashradius+1;yi>=-1;yi-=2,ye=-ye)
		for(y=(yi>>1);y!=ye;y+=yi)
			for(xi=1,xe=flashradius+1;xi>=-1;xi-=2,xe=-xe)
				for(x=(xi>>1);x!=xe;x+=xi)
				{
					xx = ipx+x; yy = ipy+y;
					if (cast(uint)(xx|yy) >= VSID) goto normflash_exwhile4;
					k = max(labs(x),labs(y)); m = ((x+xi != xe) && (y+yi != ye));

					v = sptr[yy*VSID+xx]; i = 0; sq = x*x+y*y;
					while (1)
					{
						for(z=v[1];z<=v[2];z++)
						{
							if (ipz-z >= flashradius) continue;
							if (ipz-z < k) goto normflash_exwhile4;
							tbuf2[i] = ipz-z; tbuf2[i+1] = cast(int)&v[(z-v[1])*4+4]; i += 2;
						}
						if (!v[0]) break;
						ceilnum = v[2]-v[1]-v[0]+2; v += v[0]*4;
						for(z=v[3]+ceilnum;z<v[3];z++)
						{
							if (ipz-z >= flashradius) continue;
							if (ipz-z < k) goto normflash_exwhile4;
							tbuf2[i] = ipz-z; tbuf2[i+1] = cast(int)&v[(z-v[3])*4]; i += 2;
						}
					}
					normflash_exwhile4:
					while (i > 0)
					{
						i -= 2;
						if ((!docube(x,y,tbuf2[i])) || (tbuf2[i] <= k)) continue;
						estnorm(xx,yy,ipz-tbuf2[i],&fp);
						f = fp.x*x + fp.y*y - fp.z*tbuf2[i];
						if (*cast(int *)&f > 0) addusb(&(cast(ubyte *)tbuf2[i+1])[3],cast(int)(f*fintens/(tbuf2[i]*tbuf2[i]+sq)));
					}
				}
	updatebbox(vx5.minx,vx5.miny,vx5.minz,vx5.maxx,vx5.maxy,vx5.maxz,0);
	}
}

void hline (float x0, float y0, float x1, float y1, int *ix0, int *ix1)
{
	float dyx;

	dyx = (y1-y0) * grd; //grd = 1/(x1-x0)

		  if (y0 < wy0) ftol((wy0-y0)/dyx+x0,ix0);
	else if (y0 > wy1) ftol((wy1-y0)/dyx+x0,ix0);
	else ftol(x0,ix0);
		  if (y1 < wy0) ftol((wy0-y0)/dyx+x0,ix1);
	else if (y1 > wy1) ftol((wy1-y0)/dyx+x0,ix1);
	else ftol(x1,ix1);
	if ((*ix0) < iwx0) (*ix0) = iwx0;
	if ((*ix0) > iwx1) (*ix0) = iwx1; //(*ix1) = min(max(*ix1,wx0),wx1);
	gline(labs((*ix1)-(*ix0)),cast(float)(*ix0),((*ix0)-x1)*dyx + y1,
									  cast(float)(*ix1),((*ix1)-x1)*dyx + y1);
}

void vline (float x0, float y0, float x1, float y1, int *iy0, int *iy1)
{
	float dxy;

	dxy = (x1-x0) * grd; //grd = 1/(y1-y0)

		  if (x0 < wx0) ftol((wx0-x0)/dxy+y0,iy0);
	else if (x0 > wx1) ftol((wx1-x0)/dxy+y0,iy0);
	else ftol(y0,iy0);
		  if (x1 < wx0) ftol((wx0-x0)/dxy+y0,iy1);
	else if (x1 > wx1) ftol((wx1-x0)/dxy+y0,iy1);
	else ftol(y1,iy1);
	if ((*iy0) < iwy0) (*iy0) = iwy0;
	if ((*iy0) > iwy1) (*iy0) = iwy1;
	gline(labs((*iy1)-(*iy0)),((*iy0)-y1)*dxy + x1,cast(float)(*iy0),
									  ((*iy1)-y1)*dxy + x1,cast(float)(*iy1));
}

static double optistrx, optistry, optiheix, optiheiy, optiaddx, optiaddy;

static long foglut[2048];
static long fogcol;
static long ofogdist = -1;

//#ifdef _MSC_VER

extern(C) {
	extern void *opti4asm;
	enum opti4 = (cast(point4d *)&opti4asm);

}

alias void function(long, long, int, int, int, int) da_hrend;
alias void function(long, long, int, int, int) da_vrend;
__gshared da_hrend hrend;
__gshared da_vrend vrend;

static if(USEZBUFFER != 1)
{
	void hrendnoz(int sx, int sy, int p1, int plc, int incr, int j)
	{
		sy = ylookup[sy] + frameplace; p1 = sy+(p1<<2); sy += (sx<<2);
		do
		{
			*cast(int *)sy = angstart[plc>>16][j].col;
			plc += incr; sy += 4;
		} while (sy != p1);		
	}

	void vrendnoz (int sx, int sy, int p1, int iplc, int iinc)
	{
		sy = ylookup[sy]+(sx<<2)+frameplace;
		for(;sx<p1;sx++)
		{
			*cast(int *)sy = angstart[uurend[sx]>>16][iplc].col;
			uurend[sx] += uurend[sx+MAXXDIM]; sy += 4; iplc += iinc;
		}
	}	
} else
{
	static if(false)
	{
			//Example C code
		void hrendz (int sx, int sy, int p1, int plc, int incr, int j)
		{
			int p0, i; float dirx, diry;
			p0 = ylookup[sy]+(sx<<2)+frameplace;
			p1 = ylookup[sy]+(p1<<2)+frameplace;
			dirx = optistrx*cast(float)sx + optiheix*cast(float)sy + optiaddx;
			diry = optistry*cast(float)sx + optiheiy*cast(float)sy + optiaddy;
			i = zbufoff;
			do
			{
				*cast(int *)p0 = angstart[plc>>16][j].col;
				*cast(float *)(p0+i) = cast(float)angstart[plc>>16][j].dist/sqrt(dirx*dirx+diry*diry);
				dirx += optistrx; diry += optistry; plc += incr; p0 += 4;
			} while (p0 != p1);
		}

			//Example C code
		void vrendz (int sx, int sy, int p1, int iplc, int iinc)
		{
			float dirx, diry; int i, p0;
			p0 = ylookup[sy]+(sx<<2)+frameplace;
			p1 = ylookup[sy]+(p1<<2)+frameplace;
			dirx = optistrx*cast(float)sx + optiheix*cast(float)sy + optiaddx;
			diry = optistry*cast(float)sx + optiheiy*cast(float)sy + optiaddy;
			i = zbufoff;
			while (p0 < p1)
			{
				*cast(int *)p0 = angstart[uurend[sx]>>16][iplc].col;
				*cast(float *)(p0+i) = cast(float)angstart[uurend[sx]>>16][iplc].dist/sqrt(dirx*dirx+diry*diry);
				dirx += optistrx; diry += optistry; uurend[sx] += uurend[sx+MAXXDIM]; p0 += 4; iplc += iinc; sx++;
			}
		}

			//Example C code
		void hrendzfog (int sx, int sy, int p1, int plc, int incr, int j)
		{
			int p0, i, k, l; float dirx, diry;
			p0 = ylookup[sy]+(sx<<2)+frameplace;
			p1 = ylookup[sy]+(p1<<2)+frameplace;
			dirx = optistrx*cast(float)sx + optiheix*cast(float)sy + optiaddx;
			diry = optistry*cast(float)sx + optiheiy*cast(float)sy + optiaddy;
			i = zbufoff;
			do
			{
				k = angstart[plc>>16][j].col;
				l = angstart[plc>>16][j].dist;
				l = (foglut[l>>20]&32767);
				*cast(int *)p0 = ((((( vx5.fogcol     &255)-( k     &255))*l)>>15)    ) +
								  ((((((vx5.fogcol>> 8)&255)-((k>> 8)&255))*l)>>15)<< 8) +
								  ((((((vx5.fogcol>>16)&255)-((k>>16)&255))*l)>>15)<<16)+k;
				*cast(float *)(p0+i) = cast(float)angstart[plc>>16][j].dist/sqrt(dirx*dirx+diry*diry);
				dirx += optistrx; diry += optistry; plc += incr; p0 += 4;
			} while (p0 != p1);
		}

			//Example C code
		void vrendzfog (int sx, int sy, int p1, int iplc, int iinc)
		{
			float dirx, diry; int i, k, l, p0;
			p0 = ylookup[sy]+(sx<<2)+frameplace;
			p1 = ylookup[sy]+(p1<<2)+frameplace;
			dirx = optistrx*cast(float)sx + optiheix*cast(float)sy + optiaddx;
			diry = optistry*cast(float)sx + optiheiy*cast(float)sy + optiaddy;
			i = zbufoff;
			while (p0 < p1)
			{
				k = angstart[uurend[sx]>>16][iplc].col;
				l = angstart[uurend[sx]>>16][iplc].dist;
				l = (foglut[l>>20]&32767);
				*cast(int *)p0 = ((((( vx5.fogcol     &255)-( k     &255))*l)>>15)    ) +
								  ((((((vx5.fogcol>> 8)&255)-((k>> 8)&255))*l)>>15)<< 8) +
								  ((((((vx5.fogcol>>16)&255)-((k>>16)&255))*l)>>15)<<16)+k;
				*cast(float *)(p0+i) = cast(float)angstart[uurend[sx]>>16][iplc].dist/sqrt(dirx*dirx+diry*diry);
				dirx += optistrx; diry += optistry; uurend[sx] += uurend[sx+MAXXDIM]; p0 += 4; iplc += iinc; sx++;
			}
		}

	}


	void hrendzsse (long sx, long sy, int p1, int plc, int incr, int j)
	{
		asm
		{
			push ESI;
			push EDI;
	beghasm_p3:
			mov EAX, sx;
			mov ECX, sy;
			mov ESI, p1;
			mov EDX, ylookup[ECX*4];
			add EDX, frameplace;
			lea EDI, [EDX+EAX*4];
			lea ESI, [EDX+ESI*4];

			and EAX, 0xfffffffc;
			cvtsi2ss XMM0, EAX;
			cvtsi2ss XMM4, ECX;
			movss XMM1, XMM0;
			movss XMM5, XMM4;
			mulss XMM0, optistrx;
			mulss XMM1, optistry;
			mulss XMM4, optiheix;
			mulss XMM5, optiheiy;
			addss XMM0, optiaddx;
			addss XMM1, optiaddy;
			addss XMM0, XMM4;
			addss XMM1, XMM5;

			mov ECX, zbufoff;
			mov EDX, j;
			movd MM6, plc;
			movd MM7, incr;

			shufps XMM0, XMM0, 0;
			shufps XMM1, XMM1, 0;
			movaps XMM2, opti4asm[2*16];
			movaps XMM3, opti4asm[3*16];
			addps XMM0, opti4asm[0*16];
			addps XMM1, opti4asm[1*16];
				//XMM0 =  XMM0      ^2 +  XMM1      ^2        (p)
				//XMM2 = (XMM0+XMM2)^2 + (XMM1+XMM3)^2 - XMM0 (v)
				//XMM1 = ...                                  (a)
			addps XMM2, XMM0;  //This block converts inner loop...
			addps XMM3, XMM1;  //from: 1 / sqrt(x*x + y*y), x += xi, y += yi;
			mulps XMM0, XMM0;  //  to: 1 / sqrt(p), p += v, v += a;
			mulps XMM1, XMM1;
			mulps XMM2, XMM2;
			mulps XMM3, XMM3;
			addps XMM0, XMM1;
			movaps XMM1, opti4asm[4*16];
			addps XMM2, XMM3;
			subps XMM2, XMM0;

				//Do first 0-3 pixels to align unrolled loop of 4
			test EDI, 15;
			jz short skip1ha;

			test EDI, 8;
			jz short skipshufa;
			shufps XMM0, XMM0, 0x4e; //rotate right by 2
	skipshufa:
			test EDI, 4;
			jz short skipshufb;
			shufps XMM0, XMM0, 0x39; //rotate right by 1
	skipshufb:

	beg1ha:
			pextrw EAX, MM6, 1;
			paddd MM6, MM7;
			mov EAX, angstart[EAX*4];
			movd MM0, [EAX+EDX*8];
			movd [EDI], MM0;
			cvtsi2ss XMM7, [EAX+EDX*8+4];
			rsqrtss XMM3, XMM0;
			mulss XMM7, XMM3;
			shufps XMM0, XMM0, 0x39; //rotate right by 1
			movss [EDI+ECX], XMM7;
			add EDI, 4;
			cmp EDI, ESI;
			jz short endh;
			test EDI, 15;
			jnz short beg1ha;

			addps XMM0, XMM2;
			addps XMM2, XMM1;
	skip1ha:
			lea EAX, [EDI+16];      //these 3 lines re-ordered
			cmp EAX, ESI;
			ja short skip4h;

			movq MM0, MM6;          //mm0: 0,plc
			paddd MM0, MM7;         //mm0: 0,plc+inc
			punpckldq MM7, MM7;     //mm7: inc,inc
			punpckldq MM6, MM0;     //MM6: plc+inc,plc
			paddd MM7, MM7;         //mm7: inc+inc,inc+inc

			sub ESI, 16;

			 //eax: temp   ³ mm0:  z0 argb0   argb1 argb0 ³ XMM0: plc3 plc2 plc1 plc0
			 //ebx:  -     ³ mm1:  z1 argb1               ³ XMM1: acc3 acc2 acc1 acc0
			 //ECX:zbufoff ³ mm2:  z2 argb2   argb3 argb2 ³ xmm2: inc3 inc2 inc1 inc0
			 //EDX:  j     ³ mm3:  z3 argb3               ³ xmm3:  r3   r2   r1   r0
			 //esi:  -     ³ mm4:              z1    z0   ³ XMM4:            z3   z2
			 //edi:scroff  ³ mm5:              z3    z2   ³ XMM5:
			 //ebp:  -     ³ MM6: plc1 plc0               ³ XMM6:
	beg4h:   //esp:  -     ³ mm7: inc1 inc0               ³ xmm7:  z3   z2   z1   z0
			pextrw EAX, MM6, 1;
			mov EAX, angstart[EAX*4];
			movq MM0, [EAX+EDX*8];
			pextrw EAX, MM6, 3;
			mov EAX, angstart[EAX*4];
			movq MM1, [EAX+EDX*8];
			paddd MM6, MM7;
			pextrw EAX, MM6, 1;
			mov EAX, angstart[EAX*4];
			movq MM2, [EAX+EDX*8];
			pextrw EAX, MM6, 3;
			mov EAX, angstart[EAX*4];
			movq MM3, [EAX+EDX*8];
			paddd MM6, MM7;

			movq MM4, MM0;
			movq MM5, MM2;
			punpckldq MM0, MM1;
			punpckldq MM2, MM3;
			movntq [EDI], MM0;
			movntq [EDI+8], MM2;

			punpckhdq MM4, MM1;
			punpckhdq MM5, MM3;
			cvtpi2ps XMM7, MM4;
			cvtpi2ps XMM4, MM5;
			rsqrtps XMM3, XMM0;
			movlhps XMM7, XMM4;
			mulps XMM7, XMM3;
			movntps [EDI+ECX], XMM7;
			addps XMM0, XMM2;
			addps XMM2, XMM1;

			add EDI, 16;
			cmp EDI, ESI;
			jbe short beg4h;
			add ESI, 16;
			cmp EDI, ESI;
			jae endh;

			psrad MM7, 1;    //Restore mm7 from incr*2 to just incr for single loop
	skip4h:
	beg1h:
			pextrw EAX, MM6, 1;
			paddd MM6, MM7;
			mov EAX, angstart[EAX*4];
			movd MM0, [EAX+EDX*8];
			movd [EDI], MM0;
			cvtsi2ss XMM7, [EAX+EDX*8+4];
			rsqrtss XMM3, XMM0;
			mulss XMM7, XMM3;
			shufps XMM0, XMM0, 0x39; //rotate right by 1
			movss [EDI+ECX], XMM7;
			add EDI, 4;
			cmp EDI, ESI;
			jb short beg1h;
	endh: 
			pop EDI;
			pop ESI;
		}
	}

	void hrendzfogsse (long sx, long sy, int p1, int plc, int incr, int j)
	{
		static long mm7bak;
		asm
		{
			push ESI;
			push EDI;
	beghasm_p3:
			mov EAX, sx;
			mov ECX, sy;
			mov ESI, p1;
			mov EDX, ylookup[ECX*4];
			add EDX, frameplace;
			lea EDI, [EDX+EAX*4];
			lea ESI, [EDX+ESI*4];

			and EAX, 0xfffffffc;
			cvtsi2ss XMM0, EAX;
			cvtsi2ss XMM4, ECX;
			movss XMM1, XMM0;
			movss XMM5, XMM4;
			mulss XMM0, optistrx;
			mulss XMM1, optistry;
			mulss XMM4, optiheix;
			mulss XMM5, optiheiy;
			addss XMM0, optiaddx;
			addss XMM1, optiaddy;
			addss XMM0, XMM4;
			addss XMM1, XMM5;

			mov ECX, zbufoff;
			mov EDX, j;
			movd MM6, plc;
			movd MM7, incr;

			shufps XMM0, XMM0, 0;
			shufps XMM1, XMM1, 0;
			movaps XMM2, opti4asm[2*16];
			movaps XMM3, opti4asm[3*16];
			addps XMM0, opti4asm[0*16];
			addps XMM1, opti4asm[1*16];
				//xmm0 =  xmm0      ^2 +  xmm1      ^2        (p)
				//xmm2 = (xmm0+xmm2)^2 + (xmm1+xmm3)^2 - xmm0 (v)
				//xmm1 = ...                                  (a)
			addps XMM2, XMM0;  //This block converts inner loop...
			addps XMM3, XMM1;  //from: 1 / sqrt(x*x + y*y), x += xi, y += yi;
			mulps XMM0, XMM0;  //  to: 1 / sqrt(p), p += v, v += a;
			mulps XMM1, XMM1;
			mulps XMM2, XMM2;
			mulps XMM3, XMM3;
			addps XMM0, XMM1;
			movaps XMM1, opti4asm[4*16];
			addps XMM2, XMM3;
			subps XMM2, XMM0;

				//Do first 0-3 pixels to align unrolled loop of 4
			test EDI, 15;
			jz short skip1ha;

			test EDI, 8;
			jz short skipshufa;
			shufps XMM0, XMM0, 0x4e; //rotate right by 2
	skipshufa:
			test EDI, 4;
			jz short skipshufb;
			shufps XMM0, XMM0, 0x39; //rotate right by 1
	skipshufb:

	beg1ha:
			pextrw EAX, MM6, 1;
			paddd MM6, MM7;
			mov EAX, angstart[EAX*4];

				//Z
			cvtsi2ss XMM7, [EAX+EDX*8+4];
			rsqrtss XMM3, XMM0;
			mulss XMM7, XMM3;
			shufps XMM0, XMM0, 0x39; //rotate right by 1
			movss [EDI+ECX], XMM7;

				//Col
			punpcklbw MM0, [EAX+EDX*8];
			psrlw MM0, 8;
			movq MM1, fogcol;
			psubw MM1, MM0;
			paddw MM1, MM1;
			mov EAX, [EAX+EDX*8+4];
			shr EAX, 16+4;
			pmulhw MM1, foglut[EAX*8];
			paddw MM0, MM1;
			packuswb MM0, MM1;
			movd [EDI], MM0;

			add EDI, 4;
			cmp EDI, ESI;
			jz short endh;
			test EDI, 15;
			jnz short beg1ha;

			addps XMM0, XMM2;
			addps XMM2, XMM1;
	skip1ha:
			lea EAX, [EDI+16];      //these 3 lines re-ordered
			cmp EAX, ESI;
			ja short skip4h;

			movq MM0, MM6;          //mm0: 0,plc
			paddd MM0, MM7;         //mm0: 0,plc+inc
			punpckldq MM7, MM7;     //mm7: inc,inc
			punpckldq MM6, MM0;     //mm6: plc+inc,plc
			paddd MM7, MM7;         //mm7: inc+inc,inc+inc

			sub ESI, 16;

			 //eax: temp   ³ mm0:  z0 argb0   argb1 argb0 ³ xmm0: plc3 plc2 plc1 plc0
			 //ebx:  -     ³ mm1:  z1 argb1               ³ xmm1: acc3 acc2 acc1 acc0
			 //ecx:zbufoff ³ mm2:  z2 argb2   argb3 argb2 ³ xmm2: inc3 inc2 inc1 inc0
			 //edx:  j     ³ mm3:  z3 argb3               ³ xmm3:  r3   r2   r1   r0
			 //esi:  -     ³ mm4:              z1    z0   ³ xmm4:            z3   z2
			 //edi:scroff  ³ mm5:              z3    z2   ³ xmm5:
			 //ebp:  -     ³ mm6: plc1 plc0               ³ xmm6:
			 //esp:  -     ³ mm7: inc1 inc0               ³ xmm7:  z3   z2   z1   z0

			movq mm7bak, MM7;
	beg4h:
			pextrw EAX, MM6, 1;
			mov EAX, angstart[EAX*4];
			movq MM4, [EAX+EDX*8];
			pextrw EAX, MM6, 3;
			mov EAX, angstart[EAX*4];
			movq MM1, [EAX+EDX*8];
			paddd MM6, mm7bak;
			pextrw EAX, MM6, 1;
			mov EAX, angstart[EAX*4];
			movq MM5, [EAX+EDX*8];
			pextrw EAX, MM6, 3;
			mov EAX, angstart[EAX*4];
			movq MM3, [EAX+EDX*8];
			paddd MM6, mm7bak;

			movq MM0, MM4;
			movq MM2, MM5;

				//Do Z
			punpckhdq MM4, MM1;
			punpckhdq MM5, MM3;
			cvtpi2ps XMM7, MM4;
			cvtpi2ps XMM4, MM5;
			rsqrtps XMM3, XMM0;
			movlhps XMM7, XMM4;
			mulps XMM7, XMM3;
			movntps [EDI+ECX], XMM7;
			addps XMM0, XMM2;
			addps XMM2, XMM1;

				//Do colors
				//mm4:dist1 dist0
				//mm5:dist3 dist2
			pxor MM7, MM7;
			punpcklbw MM0, MM7;
			punpcklbw MM1, MM7;
			punpcklbw MM2, MM7;
			punpcklbw MM3, MM7;

			movq MM7, fogcol;
			psubw MM7, MM0;
			paddw MM7, MM7;
			pextrw EAX, MM4, 1;
			shr EAX, 4;
			pmulhw MM7, foglut[EAX*8];
			paddw MM0, MM7;

			movq MM7, fogcol;
			psubw MM7, MM1;
			paddw MM7, MM7;
			pextrw EAX, MM4, 3;
			shr EAX, 4;
			pmulhw MM7, foglut[EAX*8];
			paddw MM1, MM7;

			movq MM7, fogcol;
			psubw MM7, MM2;
			paddw MM7, MM7;
			pextrw EAX, MM5, 1;
			shr EAX, 4;
			pmulhw MM7, foglut[EAX*8];
			paddw MM2, MM7;

			movq MM7, fogcol;
			psubw MM7, MM3;
			paddw MM7, MM7;
			pextrw EAX, MM5, 3;
			shr EAX, 4;
			pmulhw MM7, foglut[EAX*8];
			paddw MM3, MM7;

			packuswb MM0, MM1;
			packuswb MM2, MM3;
			movntq [EDI], MM0;
			movntq [EDI+8], MM2;

			add EDI, 16;
			cmp EDI, ESI;
			jbe short beg4h;
			add ESI, 16;
			cmp EDI, ESI;
			jae endh;

			movq MM7, mm7bak;
			psrad MM7, 1;    //Restore mm7 from incr*2 to just incr for single loop
	skip4h:
	beg1h:
			pextrw EAX, MM6, 1;
			paddd MM6, MM7;
			mov EAX, angstart[EAX*4];

				//Z
			cvtsi2ss XMM7, [EAX+EDX*8+4];
			rsqrtss XMM3, XMM0;
			mulss XMM7, XMM3;
			shufps XMM0, XMM0, 0x39; //rotate right by 1
			movss [EDI+ECX], XMM7;

				//Col
			punpcklbw MM0, [EAX+EDX*8];
			psrlw MM0, 8;
			movq MM1, fogcol;
			psubw MM1, MM0;
			paddw MM1, MM1;
			mov EAX, [EAX+EDX*8+4];
			shr EAX, 16+4;
			pmulhw MM1, foglut[EAX*8];
			paddw MM0, MM1;
			packuswb MM0, MM1;
			movd [EDI], MM0;

			add EDI, 4;
			cmp EDI, ESI;
			jb short beg1h;
	endh: 
			pop EDI;
			pop ESI;
		}
	}

	void hrendz3dn (long sx, long sy, int p1, int plc, int incr, int j)
	{
		asm
		{
			push ESI;
			push EDI;
			mov EAX, sy;
			mov EAX, ylookup[EAX*4];
			add EAX, frameplace;
			mov ESI, p1;
			lea ESI, [EAX+ESI*4];    //esi = p1
			mov EDI, sx;
			lea EDI, [EAX+EDI*4];    //edi = p0

			movd MM0, sx;
			punpckldq MM0, sy;
			pi2fd MM0, MM0;          //mm0: (float)sy (float)sx
			pshufw MM2, MM0, 0xee;   //mm2: (float)sy (float)sy
			punpckldq MM0, MM0;      //mm0: (float)sx (float)sx
			movd MM1, optistrx;
			punpckldq MM1, optistry;
			pfmul MM0, MM1;          //mm0: (float)sx*optistry (float)sx*optistrx
			movd MM3, optiheix;
			punpckldq MM3, optiheiy;
			pfmul MM2, MM3;          //mm2: (float)sy*optiheiy (float)sy*optiheix
			pfadd MM0, MM2;
			movd MM3, optiaddx;
			punpckldq MM3, optiaddy; //mm3: optiaddy optiaddx
			pfadd MM0, MM3;          //mm0: diry diry

			movd MM6, plc;
			movd MM7, incr;
			mov ECX, zbufoff;
			mov EDX, j;

	beg:  
			pextrw EAX, MM6, 1;
			mov EAX, angstart[EAX*4];
			movq MM2, [EAX+EDX*8];   //mm2:      dist       col
			pshufw MM3, MM2, 0xee;   //mm3:         ?      dist
			pi2fd MM3, MM3;          //mm3:         ?   (f)dist
			movq MM4, MM0;           //mm4:      diry      dirx
			pfmul MM4, MM4;          //mm4:    diry^2    dirx^2
			pfadd MM0, MM1;          //mm0: dirx+optx diry+opty (unrelated)
			pfacc MM4, MM4;          //mm4: (x^2+y^2)   x^2+y^2
			pfrsqrt MM4, MM4;        //mm4: 1/sqrt(*) 1/sqrt(*)
			pfmul MM3, MM4;          //mm3:         0    zvalue
			paddd MM6, MM7;          //mm6:            plc+incr (unrelated)
			movd [EDI], MM2;
			movd [EDI+ECX], MM3;
			add EDI, 4;
			cmp EDI, ESI;
			jb short beg;
			pop EDI;
			pop ESI;
		}
	}

	void hrendzfog3dn (long sx, long sy, int p1, int plc, int incr, int j)
	{
		asm
		{
			push ESI;
			push EDI;
			mov EAX, sy;
			mov EAX, ylookup[EAX*4];
			add EAX, frameplace;
			mov ESI, p1;
			lea ESI, [EAX+ESI*4];    //esi = p1
			mov EDI, sx;
			lea EDI, [EAX+EDI*4];    //edi = p0

			movd MM0, sx;
			punpckldq MM0, sy;
			pi2fd MM0, MM0;          //mm0: (float)sy (float)sx
			pshufw MM2, MM0, 0xee;   //mm2: (float)sy (float)sy
			punpckldq MM0, MM0;      //mm0: (float)sx (float)sx
			movd MM1, optistrx;
			punpckldq MM1, optistry;
			pfmul MM0, MM1;          //mm0: (float)sx*optistry (float)sx*optistrx
			movd MM3, optiheix;
			punpckldq MM3, optiheiy;
			pfmul MM2, MM3;          //mm2: (float)sy*optiheiy (float)sy*optiheix
			pfadd MM0, MM2;
			movd MM3, optiaddx;
			punpckldq MM3, optiaddy; //mm3: optiaddy optiaddx
			pfadd MM0, MM3;          //mm0: diry diry

			pxor MM5, MM5;

			movd MM6, plc;
			movd MM7, incr;
			mov ECX, zbufoff;
			mov EDX, j;

	beg:  
			pextrw EAX, MM6, 1;
			mov EAX, angstart[EAX*4];
			movq MM2, [EAX+EDX*8];   //mm2:      dist       col
			pshufw MM3, MM2, 0xee;   //mm3:         ?      dist
			pi2fd MM3, MM3;          //mm3:         ?   (f)dist
			movq MM4, MM0;           //mm4:      diry      dirx
			pfmul MM4, MM4;          //mm4:    diry^2    dirx^2
			pfadd MM0, MM1;          //mm0: dirx+optx diry+opty (unrelated)
			pfacc MM4, MM4;          //mm4: (x^2+y^2)   x^2+y^2
			pfrsqrt MM4, MM4;        //mm4: 1/sqrt(*) 1/sqrt(*)
			pfmul MM3, MM4;          //mm3:         0    zvalue
			paddd MM6, MM7;          //mm6:            plc+incr (unrelated)

				//Extra calculations for fog
			pextrw EAX, MM2, 3;
			punpcklbw MM2, MM5;
			movq MM4, fogcol;
			psubw MM4, MM2;
			paddw MM4, MM4;
			shr EAX, 4;
			pmulhw MM4, foglut[EAX*8];
			paddw MM2, MM4;
			packuswb MM2, MM4;

			movd [EDI], MM2;
			movd [EDI+ECX], MM3;
			add EDI, 4;
			cmp EDI, ESI;
			jb short beg;
			pop EDI;
			pop ESI;
		}
	}

	void vrendzsse (long sx, long sy, int p1, int iplc, int iinc)
	{
		asm
		{
			push EBX;
			push ESI;
			push EDI;
	begvasm_p3:
			mov ESI, sx;
			mov EAX, sy;
			mov EDX, p1;
			mov ECX, ylookup[EAX*4];
			add ECX, frameplace;
			lea EDX, [ECX+EDX*4];
			lea EDI, [ECX+ESI*4];

			mov ECX, ESI;
			and ECX, 0xfffffffc;
			cvtsi2ss XMM0, ECX;
			cvtsi2ss XMM4, EAX;
			movss XMM1, XMM0;
			movss XMM5, XMM4;
			mulss XMM0, optistrx;
			mulss XMM1, optistry;
			mulss XMM4, optiheix;
			mulss XMM5, optiheiy;
			addss XMM0, optiaddx;
			addss XMM1, optiaddy;
			addss XMM0, XMM4;
			addss XMM1, XMM5;

			shufps XMM0, XMM0, 0;
			shufps XMM1, XMM1, 0;
			movaps XMM2, opti4asm[2*16];
			movaps XMM3, opti4asm[3*16];
			addps XMM0, opti4asm[0*16];
			addps XMM1, opti4asm[1*16];
				//xmm0 =  xmm0      ^2 +  xmm1      ^2        (p)
				//xmm2 = (xmm0+xmm2)^2 + (xmm1+xmm3)^2 - xmm0 (v)
				//xmm1 = ...                                  (a)
			addps XMM2, XMM0;  //This block converts inner loop...
			addps XMM3, XMM1;  //from: 1 / sqrt(x*x + y*y), x += xi, y += yi;
			mulps XMM0, XMM0;  //  to: 1 / sqrt(p), p += v, v += a;
			mulps XMM1, XMM1;
			mulps XMM2, XMM2;
			mulps XMM3, XMM3;
			addps XMM0, XMM1;
			movaps XMM1, opti4asm[4*16];
			addps XMM2, XMM3;
			subps XMM2, XMM0;

			mov p1, EDX;
			mov ECX, zbufoff;
			shl ESI, 2;
			add ESI, uurend;
			mov EBX, iplc;

			cmp EDI, EDX;
			jae short endv;

				//Do first 0-3 pixels to align unrolled loop of 4
			test EDI, 15;
			jz short skip1va;

			test EDI, 8;
			jz short skipshufc;
			shufps XMM0, XMM0, 0x4e; //rotate right by 2
	skipshufc:
			test EDI, 4;
			jz short skipshufd;
			shufps XMM0, XMM0, 0x39; //rotate right by 1
	skipshufd:

	beg1va:
			mov EDX, [ESI];
			mov EAX, [ESI+MAXXDIM*4];
			add EAX, EDX;
			sar EDX, 16;
			mov EDX, angstart[EDX*4];
			mov [ESI], EAX;
			mov EAX, [EDX+EBX*8];
			mov [EDI], EAX;
			cvtsi2ss XMM7, [EDX+EBX*8+4];
			rsqrtss XMM3, XMM0;
			mulss XMM7, XMM3;
			shufps XMM0, XMM0, 0x39; //rotate right by 1
			movss [EDI+ECX], XMM7;
			add EBX, iinc;
			add ESI, 4;
			add EDI, 4;
			cmp EDI, p1;
			jz short endv;
			test EDI, 15;
			jnz short beg1va;

			addps XMM0, XMM2;
			addps XMM2, XMM1;
	skip1va:
			lea EDX, [EDI+16];
			cmp EDX, p1;
			ja short prebeg1v;

			cmp iinc, 0;
			jl short beg4vn;

	beg4vp:
			movq MM6, [ESI];
			movq MM7, [ESI+8];
			pextrw EAX, MM6, 1;
			pextrw EDX, MM6, 3;
			paddd MM6, [ESI+MAXXDIM*4];
			mov EAX, angstart[EAX*4];
			mov EDX, angstart[EDX*4];
			movq MM0, [EAX+EBX*8];
			movq MM1, [EDX+EBX*8+8];
			pextrw EAX, MM7, 1;
			pextrw EDX, MM7, 3;
			paddd MM7, [ESI+8+MAXXDIM*4];
			mov EAX, angstart[EAX*4];
			mov EDX, angstart[EDX*4];
			movq MM2, [EAX+EBX*8+16];
			movq MM3, [EDX+EBX*8+24];
			add EBX, 4;

			movq MM4, MM0;
			movq MM5, MM2;
			punpckldq MM0, MM1;
			punpckldq MM2, MM3;
			movntq [EDI], MM0;
			movntq [EDI+8], MM2;

			punpckhdq MM4, MM1;
			punpckhdq MM5, MM3;
			cvtpi2ps XMM7, MM4;
			cvtpi2ps XMM4, MM5;
			rsqrtps XMM3, XMM0;
			movlhps XMM7, XMM4;
			mulps XMM7, XMM3;
			movntps [EDI+ECX], XMM7;
			addps XMM0, XMM2;
			addps XMM2, XMM1;

			movq [ESI], MM6;
			movq [ESI+8], MM7;

			add ESI, 16;
			add EDI, 16;
			lea EDX, [EDI+16];
			cmp EDX, p1;
			jbe short beg4vp;
			cmp EDI, p1;
			jae short endv;
			jmp short prebeg1v;

	beg4vn:
			movq MM6, [ESI];
			movq MM7, [ESI+8];
			pextrw EAX, MM6, 1;
			pextrw EDX, MM6, 3;
			paddd MM6, [ESI+MAXXDIM*4];
			mov EAX, angstart[EAX*4];
			mov EDX, angstart[EDX*4];
			movq MM0, [EAX+EBX*8];
			movq MM1, [EDX+EBX*8-8];
			pextrw EAX, MM7, 1;
			pextrw EDX, MM7, 3;
			paddd MM7, [ESI+8+MAXXDIM*4];
			mov EAX, angstart[EAX*4];
			mov EDX, angstart[EDX*4];
			movq MM2, [EAX+EBX*8-16];
			movq MM3, [EDX+EBX*8-24];
			sub EBX, 4;

			movq MM4, MM0;
			movq MM5, MM2;
			punpckldq MM0, MM1;
			punpckldq MM2, MM3;
			movntq [EDI], MM0;
			movntq [EDI+8], MM2;

			punpckhdq MM4, MM1;
			punpckhdq MM5, MM3;
			cvtpi2ps XMM7, MM4;
			cvtpi2ps XMM4, MM5;
			rsqrtps XMM3, XMM0;
			movlhps XMM7, XMM4;
			mulps XMM7, XMM3;
			movntps [EDI+ECX], XMM7;
			addps XMM0, XMM2;
			addps XMM2, XMM1;

			movq [ESI], MM6;
			movq [ESI+8], MM7;

			add ESI, 16;
			add EDI, 16;
			lea EDX, [EDI+16];
			cmp EDX, p1;
			jbe short beg4vn;
			cmp EDI, p1;
			jae short endv;

	prebeg1v:
	beg1v:
			mov EDX, [ESI];
			mov EAX, [ESI+MAXXDIM*4];
			add EAX, EDX;
			sar EDX, 16;
			mov EDX, angstart[EDX*4];
			mov [ESI], EAX;
			mov EAX, [EDX+EBX*8];
			mov [EDI], EAX;
			cvtsi2ss XMM7, [EDX+EBX*8+4];
			rsqrtss XMM3, XMM0;
			mulss XMM7, XMM3;
			shufps XMM0, XMM0, 0x39; //rotate right by 1
			movss [EDI+ECX], XMM7;
			add EBX, iinc;
			add ESI, 4;
			add EDI, 4;
			cmp EDI, p1;
			jne short beg1v;
	endv: 
			pop EDI;
			pop ESI;
			pop EBX;
		}
	}

	void vrendzfogsse (long sx, long sy, int p1, int iplc, int iinc)
	{
		asm
		{
			push EBX;
			push ESI;
			push EDI;
	begvasm_p3:
			mov ESI, sx;
			mov EAX, sy;
			mov EDX, p1;
			mov ECX, ylookup[EAX*4];
			add ECX, frameplace;
			lea EDX, [ECX+EDX*4];
			lea EDI, [ECX+ESI*4];

			mov ECX, ESI;
			and ECX, 0xfffffffc;
			cvtsi2ss XMM0, ECX;
			cvtsi2ss XMM4, EAX;
			movss XMM1, XMM0;
			movss XMM5, XMM4;
			mulss XMM0, optistrx;
			mulss XMM1, optistry;
			mulss XMM4, optiheix;
			mulss XMM5, optiheiy;
			addss XMM0, optiaddx;
			addss XMM1, optiaddy;
			addss XMM0, XMM4;
			addss XMM1, XMM5;

			shufps XMM0, XMM0, 0;
			shufps XMM1, XMM1, 0;
			movaps XMM2, opti4asm[2*16];
			movaps XMM3, opti4asm[3*16];
			addps XMM0, opti4asm[0*16];
			addps XMM1, opti4asm[1*16];
				//xmm0 =  xmm0      ^2 +  xmm1      ^2        (p)
				//xmm2 = (xmm0+xmm2)^2 + (xmm1+xmm3)^2 - xmm0 (v)
				//xmm1 = ...                                  (a)
			addps XMM2, XMM0;  //This block converts inner loop...
			addps XMM3, XMM1;  //from: 1 / sqrt(x*x + y*y), x += xi, y += yi;
			mulps XMM0, XMM0;  //  to: 1 / sqrt(p), p += v, v += a;
			mulps XMM1, XMM1;
			mulps XMM2, XMM2;
			mulps XMM3, XMM3;
			addps XMM0, XMM1;
			movaps XMM1, opti4asm[4*16];
			addps XMM2, XMM3;
			subps XMM2, XMM0;

			mov p1, EDX;
			mov ECX, zbufoff;
			shl ESI, 2;
			add ESI, uurend;
			mov EBX, iplc;

			cmp EDI, EDX;
			jae short endv;

				//Do first 0-3 pixels to align unrolled loop of 4
			test EDI, 15;
			jz short skip1va;

			test EDI, 8;
			jz short skipshufc;
			shufps XMM0, XMM0, 0x4e; //rotate right by 2
	skipshufc:
			test EDI, 4;
			jz short skipshufd;
			shufps XMM0, XMM0, 0x39; //rotate right by 1
	skipshufd:

	beg1va:
			mov EDX, [ESI];
			mov EAX, [ESI+MAXXDIM*4];
			add EAX, EDX;
			sar EDX, 16;
			mov EDX, angstart[EDX*4];
			mov [ESI], EAX;

				//Z
			cvtsi2ss XMM7, [EDX+EBX*8+4];
			rsqrtss XMM3, XMM0;
			mulss XMM7, XMM3;
			shufps XMM0, XMM0, 0x39; //rotate right by 1
			movss [EDI+ECX], XMM7;

				//Col
			punpcklbw MM0, [EDX+EBX*8];
			psrlw MM0, 8;
			movq MM1, fogcol;
			psubw MM1, MM0;
			paddw MM1, MM1;
			mov EAX, [EDX+EBX*8+4];
			shr EAX, 16+4;
			pmulhw MM1, foglut[EAX*8];
			paddw MM0, MM1;
			packuswb MM0, MM1;
			movd [EDI], MM0;

			add EBX, iinc;
			add ESI, 4;
			add EDI, 4;
			cmp EDI, p1;
			jz short endv;
			test EDI, 15;
			jnz short beg1va;

			addps XMM0, XMM2;
			addps XMM2, XMM1;
	skip1va:
			lea EDX, [EDI+16];
			cmp EDX, p1;
			ja short prebeg1v;

			cmp iinc, 0;
			jl short beg4vn;

	beg4vp:
			movq MM6, [ESI];
			movq MM7, [ESI+8];
			pextrw EAX, MM6, 1;
			pextrw EDX, MM6, 3;
			paddd MM6, [ESI+MAXXDIM*4];
			mov EAX, angstart[EAX*4];
			mov EDX, angstart[EDX*4];
			movq MM4, [EAX+EBX*8];
			movq MM1, [EDX+EBX*8+8];
			pextrw EAX, MM7, 1;
			pextrw EDX, MM7, 3;
			paddd MM7, [ESI+8+MAXXDIM*4];
			mov EAX, angstart[EAX*4];
			mov EDX, angstart[EDX*4];
			movq MM5, [EAX+EBX*8+16];
			movq MM3, [EDX+EBX*8+24];
			add EBX, 4;

				//Do Z
			movq MM0, MM4;
			movq MM2, MM5;
			punpckhdq MM4, MM1;
			punpckhdq MM5, MM3;
			cvtpi2ps XMM7, MM4;
			cvtpi2ps XMM4, MM5;
			rsqrtps XMM3, XMM0;
			movlhps XMM7, XMM4;
			mulps XMM7, XMM3;
			movntps [EDI+ECX], XMM7;
			addps XMM0, XMM2;
			addps XMM2, XMM1;

			movq [ESI], MM6;
			movq [ESI+8], MM7;

				//Do color
			pxor MM7, MM7;
			punpcklbw MM0, MM7;
			punpcklbw MM1, MM7;
			punpcklbw MM2, MM7;
			punpcklbw MM3, MM7;

			movq MM7, fogcol;
			psubw MM7, MM0;
			paddw MM7, MM7;
			pextrw EAX, MM4, 1;
			shr EAX, 4;
			pmulhw MM7, foglut[EAX*8];
			paddw MM0, MM7;

			movq MM7, fogcol;
			psubw MM7, MM1;
			paddw MM7, MM7;
			pextrw EAX, MM4, 3;
			shr EAX, 4;
			pmulhw MM7, foglut[EAX*8];
			paddw MM1, MM7;

			movq MM7, fogcol;
			psubw MM7, MM2;
			paddw MM7, MM7;
			pextrw EAX, MM5, 1;
			shr EAX, 4;
			pmulhw MM7, foglut[EAX*8];
			paddw MM2, MM7;

			movq MM7, fogcol;
			psubw MM7, MM3;
			paddw MM7, MM7;
			pextrw EAX, MM5, 3;
			shr EAX, 4;
			pmulhw MM7, foglut[EAX*8];
			paddw MM3, MM7;

			packuswb MM0, MM1;
			packuswb MM2, MM3;
			movntq [EDI], MM0;
			movntq [EDI+8], MM2;

			add ESI, 16;
			add EDI, 16;
			lea EDX, [EDI+16];
			cmp EDX, p1;
			jbe short beg4vp;
			cmp EDI, p1;
			jae short endv;
			jmp short prebeg1v;

	beg4vn:
			movq MM6, [ESI];
			movq MM7, [ESI+8];
			pextrw EAX, MM6, 1;
			pextrw EDX, MM6, 3;
			paddd MM6, [ESI+MAXXDIM*4];
			mov EAX, angstart[EAX*4];
			mov EDX, angstart[EDX*4];
			movq MM4, [EAX+EBX*8];
			movq MM1, [EDX+EBX*8-8];
			pextrw EAX, MM7, 1;
			pextrw EDX, MM7, 3;
			paddd MM7, [ESI+8+MAXXDIM*4];
			mov EAX, angstart[EAX*4];
			mov EDX, angstart[EDX*4];
			movq MM5, [EAX+EBX*8-16];
			movq MM3, [EDX+EBX*8-24];
			sub EBX, 4;

				//Do Z
			movq MM0, MM4;
			movq MM2, MM5;
			punpckhdq MM4, MM1;
			punpckhdq MM5, MM3;
			cvtpi2ps XMM7, MM4;
			cvtpi2ps XMM4, MM5;
			rsqrtps XMM3, XMM0;
			movlhps XMM7, XMM4;
			mulps XMM7, XMM3;
			movntps [EDI+ECX], XMM7;
			addps XMM0, XMM2;
			addps XMM2, XMM1;

			movq [ESI], MM6;
			movq [ESI+8], MM7;

				//Do color
			pxor MM7, MM7;
			punpcklbw MM0, MM7;
			punpcklbw MM1, MM7;
			punpcklbw MM2, MM7;
			punpcklbw MM3, MM7;

			movq MM7, fogcol;
			psubw MM7, MM0;
			paddw MM7, MM7;
			pextrw EAX, MM4, 1;
			shr EAX, 4;
			pmulhw MM7, foglut[EAX*8];
			paddw MM0, MM7;

			movq MM7, fogcol;
			psubw MM7, MM1;
			paddw MM7, MM7;
			pextrw EAX, MM4, 3;
			shr EAX, 4;
			pmulhw MM7, foglut[EAX*8];
			paddw MM1, MM7;

			movq MM7, fogcol;
			psubw MM7, MM2;
			paddw MM7, MM7;
			pextrw EAX, MM5, 1;
			shr EAX, 4;
			pmulhw MM7, foglut[EAX*8];
			paddw MM2, MM7;

			movq MM7, fogcol;
			psubw MM7, MM3;
			paddw MM7, MM7;
			pextrw EAX, MM5, 3;
			shr EAX, 4;
			pmulhw MM7, foglut[EAX*8];
			paddw MM3, MM7;

			packuswb MM0, MM1;
			packuswb MM2, MM3;
			movntq [EDI], MM0;
			movntq [EDI+8], MM2;

			add ESI, 16;
			add EDI, 16;
			lea EDX, [EDI+16];
			cmp EDX, p1;
			jbe short beg4vn;
			cmp EDI, p1;
			jae short endv;

	prebeg1v:
	beg1v:
			mov EDX, [ESI];
			mov EAX, [ESI+MAXXDIM*4];
			add EAX, EDX;
			sar EDX, 16;
			mov EDX, angstart[EDX*4];
			mov [ESI], EAX;

				//Z
			cvtsi2ss XMM7, [EDX+EBX*8+4];
			rsqrtss XMM3, XMM0;
			mulss XMM7, XMM3;
			shufps XMM0, XMM0, 0x39; //rotate right by 1
			movss [EDI+ECX], XMM7;

				//Col
			punpcklbw MM0, [EDX+EBX*8];
			psrlw MM0, 8;
			movq MM1, fogcol;
			psubw MM1, MM0;
			paddw MM1, MM1;
			mov EAX, [EDX+EBX*8+4];
			shr EAX, 16+4;
			pmulhw MM1, foglut[EAX*8];
			paddw MM0, MM1;
			packuswb MM0, MM1;
			movd [EDI], MM0;

			add EBX, iinc;
			add ESI, 4;
			add EDI, 4;
			cmp EDI, p1;
			jne short beg1v;
	endv: 
			pop EDI;
			pop ESI;
			pop EBX;
		}
	}

	void vrendz3dn (long sx, long sy, int p1, int iplc, int iinc)
	{
		asm
		{
			push EBX;
			push ESI;
			push EDI;
			mov ESI, p1;
			mov EDI, sx;
			cmp EDI, ESI;
			jae short endv;
			mov EAX, sy;
			mov EAX, ylookup[EAX*4];
			add EAX, frameplace;
			lea ESI, [EAX+ESI*4];    //esi = p1
			lea EDI, [EAX+EDI*4];    //edi = p0

			movd MM0, sx;
			punpckldq MM0, sy;
			pi2fd MM0, MM0;          //mm0: (float)sy (float)sx
			pshufw MM2, MM0, 0xee;   //mm2: (float)sy (float)sy
			punpckldq MM0, MM0;      //mm0: (float)sx (float)sx
			movd MM1, optistrx;
			punpckldq MM1, optistry;
			pfmul MM0, MM1;          //mm0: (float)sx*optistry (float)sx*optistrx
			movd MM3, optiheix;
			punpckldq MM3, optiheiy;
			pfmul MM2, MM3;          //mm2: (float)sy*optiheiy (float)sy*optiheix
			pfadd MM0, MM2;
			movd MM3, optiaddx;
			punpckldq MM3, optiaddy; //mm3: optiaddy optiaddx
			pfadd MM0, MM3;          //mm0: diry diry

			mov ECX, zbufoff;
			mov EDX, iplc;
			mov EBX, sx;
			mov EAX, uurend;
			lea EBX, [EAX+EBX*4];

	begv_3dn:
			movd MM5, [EBX];
			pextrw EAX, MM5, 1;
			paddd MM5, [EBX+MAXXDIM*4];
			movd [EBX], MM5;
			mov EAX, angstart[EAX*4];
			movq MM2, [EAX+EDX*8];   //mm2:      dist       col
			pshufw MM3, MM2, 0xee;   //mm3:         ?      dist
			pi2fd MM3, MM3;          //mm3:         ?   (f)dist
			movq MM4, MM0;           //mm4:      diry      dirx
			pfmul MM4, MM4;          //mm4:    diry^2    dirx^2
			pfadd MM0, MM1;          //mm0: dirx+optx diry+opty (unrelated)
			pfacc MM4, MM4;          //mm4: (x^2+y^2)   x^2+y^2
			pfrsqrt MM4, MM4;        //mm4: 1/sqrt(*) 1/sqrt(*)
			pfmul MM3, MM4;          //mm3:         0    zvalue
			movd [EDI], MM2;
			movd [EDI+ECX], MM3;
			add EDX, iinc;
			add EBX, 4;
			add EDI, 4;
			cmp EDI, ESI;
			jb short begv_3dn;
	endv: 
			pop EDI;
			pop ESI;
			pop EBX;
		}
	}

	void vrendzfog3dn (long sx, long sy, int p1, int iplc, int iinc)
	{
		asm
		{
			push EBX;
			push ESI;
			push EDI;
			mov ESI, p1;
			mov EDI, sx;
			cmp EDI, ESI;
			jae short endv;
			mov EAX, sy;
			mov EAX, ylookup[EAX*4];
			add EAX, frameplace;
			lea ESI, [EAX+ESI*4];    //esi = p1
			lea EDI, [EAX+EDI*4];    //edi = p0

			movd MM0, sx;
			punpckldq MM0, sy;
			pi2fd MM0, MM0;          //mm0: (float)sy (float)sx
			pshufw MM2, MM0, 0xee;   //mm2: (float)sy (float)sy
			punpckldq MM0, MM0;      //mm0: (float)sx (float)sx
			movd MM1, optistrx;
			punpckldq MM1, optistry;
			pfmul MM0, MM1;          //mm0: (float)sx*optistry (float)sx*optistrx
			movd MM3, optiheix;
			punpckldq MM3, optiheiy;
			pfmul MM2, MM3;          //mm2: (float)sy*optiheiy (float)sy*optiheix
			pfadd MM0, MM2;
			movd MM3, optiaddx;
			punpckldq MM3, optiaddy; //mm3: optiaddy optiaddx
			pfadd MM0, MM3;          //mm0: diry diry

			pxor MM6, MM6;

			mov ECX, zbufoff;
			mov EDX, iplc;
			mov EBX, sx;
			mov EAX, uurend;
			lea EBX, [EAX+EBX*4];

	begv_3dn:
			movd MM5, [EBX];
			pextrw EAX, MM5, 1;
			paddd MM5, [EBX+MAXXDIM*4];
			movd [EBX], MM5;
			mov EAX, angstart[EAX*4];
			movq MM2, [EAX+EDX*8];   //mm2:      dist       col
			pshufw MM3, MM2, 0xee;   //mm3:         ?      dist
			pi2fd MM3, MM3;          //mm3:         ?   (f)dist
			movq MM4, MM0;           //mm4:      diry      dirx
			pfmul MM4, MM4;          //mm4:    diry^2    dirx^2
			pfadd MM0, MM1;          //mm0: dirx+optx diry+opty (unrelated)
			pfacc MM4, MM4;          //mm4: (x^2+y^2)   x^2+y^2
			pfrsqrt MM4, MM4;        //mm4: 1/sqrt(*) 1/sqrt(*)
			pfmul MM3, MM4;          //mm3:         0    zvalue

				//Extra calculations for fog
			pextrw EAX, MM2, 3;
			punpcklbw MM2, MM6;
			movq MM4, fogcol;
			psubw MM4, MM2;
			paddw MM4, MM4;
			shr EAX, 4;
			pmulhw MM4, foglut[EAX*8];
			paddw MM2, MM4;
			packuswb MM2, MM4;

			movd [EDI], MM2;
			movd [EDI+ECX], MM3;
			add EDX, iinc;
			add EBX, 4;
			add EDI, 4;
			cmp EDI, ESI;
			jb short begv_3dn;
	endv: 
			pop EDI;
			pop ESI;
			pop EBX;
		}
	}
} // end else USEZBUFFER != 1

void setcamera (dpoint3d *ipo, dpoint3d *ist, dpoint3d *ihe, dpoint3d *ifo,
					 float dahx, float dahy, float dahz)
{
	int i, j;

	gipos.x = ipo.x; gipos.y = ipo.y; gipos.z = ipo.z;
	gistr.x = ist.x; gistr.y = ist.y; gistr.z = ist.z;
	gihei.x = ihe.x; gihei.y = ihe.y; gihei.z = ihe.z;
	gifor.x = ifo.x; gifor.y = ifo.y; gifor.z = ifo.z;
	gihx = dahx; gihy = dahy; gihz = dahz;

	gixs.x = gistr.x; gixs.y = gihei.x; gixs.z = gifor.x;
	giys.x = gistr.y; giys.y = gihei.y; giys.z = gifor.y;
	gizs.x = gistr.z; gizs.y = gihei.z; gizs.z = gifor.z;
	giadd.x = -(gipos.x*gistr.x + gipos.y*gistr.y + gipos.z*gistr.z);
	giadd.y = -(gipos.x*gihei.x + gipos.y*gihei.y + gipos.z*gihei.z);
	giadd.z = -(gipos.x*gifor.x + gipos.y*gifor.y + gipos.z*gifor.z);

	gcorn[0].x = -gihx*gistr.x - gihy*gihei.x + gihz*gifor.x;
	gcorn[0].y = -gihx*gistr.y - gihy*gihei.y + gihz*gifor.y;
	gcorn[0].z = -gihx*gistr.z - gihy*gihei.z + gihz*gifor.z;
	gcorn[1].x = xres*gistr.x+gcorn[0].x;
	gcorn[1].y = xres*gistr.y+gcorn[0].y;
	gcorn[1].z = xres*gistr.z+gcorn[0].z;
	gcorn[2].x = yres*gihei.x+gcorn[1].x;
	gcorn[2].y = yres*gihei.y+gcorn[1].y;
	gcorn[2].z = yres*gihei.z+gcorn[1].z;
	gcorn[3].x = yres*gihei.x+gcorn[0].x;
	gcorn[3].y = yres*gihei.y+gcorn[0].y;
	gcorn[3].z = yres*gihei.z+gcorn[0].z;
	for(j=0,i=3;j<4;i=j++)
	{
		ginor[i].x = gcorn[i].y*gcorn[j].z - gcorn[i].z*gcorn[j].y;
		ginor[i].y = gcorn[i].z*gcorn[j].x - gcorn[i].x*gcorn[j].z;
		ginor[i].z = gcorn[i].x*gcorn[j].y - gcorn[i].y*gcorn[j].x;
	}
}

void opticast()
{
	float f, ff, cx, cy, fx, fy, gx, gy, x0, y0, x1, y1, x2, y2, x3, y3;
	int i, j, sx, sy, p0, p1, cx16, cy16, kadd, kmul, u, u1, ui;

	if (gifor.z < 0) giforzsgn = -1; else giforzsgn = 1; //giforzsgn = (gifor.z < 0);

	gixyi[0] = (VSID<<2); gixyi[1] = -gixyi[0];
	glipos.x = (cast(int)gipos.x);
	glipos.y = (cast(int)gipos.y);
	glipos.z = (cast(int)gipos.z);
	gpixy = cast(int)&sptr[glipos.y*VSID + glipos.x];
	ftol(gipos.z*PREC-.5f,&gposz);
	gposxfrac[1] = gipos.x - cast(float)glipos.x; gposxfrac[0] = 1-gposxfrac[1];
	gposyfrac[1] = gipos.y - cast(float)glipos.y; gposyfrac[0] = 1-gposyfrac[1];
	static if(USEV5ASM)
	{
		for(j=u=0;j<gmipnum;j++,u+=i)
			for(i=0;i<(256>>j)+4;i++)
				gylookup[i+u] = ((((gposz>>j)-i*PREC)>>(16-j))&0x0000ffff);
		gxmip = max(vx5.mipscandist,4)*PREC;
	} else
	{
		for(i=0;i<256+4;i++) gylookup[i] = (i*PREC-gposz);
	}
	gmaxscandist = min(max(vx5.maxscandist,1),2047)*PREC;

	static if(USEZBUFFER != 1)
	{
		hrend = hrendnoz; vrend = vrendnoz;
	} else
	{
		if (ofogdist < 0)
		{
			if (cputype&(1<<25)) { hrend = &hrendzsse; vrend = &vrendzsse; }
								 else { hrend = &hrendz3dn; vrend = &vrendz3dn; }
		}
		else
		{
			if (cputype&(1<<25)) { hrend = &hrendzfogsse; vrend = &vrendzfogsse; }
								 else { hrend = &hrendzfog3dn; vrend = &vrendzfog3dn; }

		}
	}

	if (ofogdist < 0) nskypic = skypic;
				  else { nskypic = skyoff = 0; } //Optimization hack: draw sky as pure black when using fog

	gstartv = cast(ubyte *)*cast(int *)gpixy;
	if (glipos.z >= gstartv[1])
	{
		do
		{
			if (!gstartv[0]) return;
			gstartv += gstartv[0]*4;
		} while (glipos.z >= gstartv[1]);
		if (glipos.z < gstartv[3]) return;
		gstartz0 = gstartv[3];
	} else gstartz0 = 0;
	gstartz1 = gstartv[1];

	if (gifor.z == 0) f = 32000; else f = gihz/gifor.z;
	f = min(max(f,-32000.0f),32000.0f);
	cx = gistr.z*f + gihx;
	cy = gihei.z*f + gihy;

	wx0 = cast(float)(-(vx5.anginc)); wx1 = cast(float)(xres-1+(vx5.anginc));
	wy0 = cast(float)(-(vx5.anginc)); wy1 = cast(float)(yres-1+(vx5.anginc));
	ftol(wx0,&iwx0); ftol(wx1,&iwx1);
	ftol(wy0,&iwy0); ftol(wy1,&iwy1);

	fx = wx0-cx; fy = wy0-cy; gx = wx1-cx; gy = wy1-cy;
	x0 = x3 = wx0; y0 = y1 = wy0; x1 = x2 = wx1; y2 = y3 = wy1;
	if (fy < 0)
	{
		if (fx < 0) { f = sqrt(fx*fy); x0 = cx-f; y0 = cy-f; }
		if (gx > 0) { f = sqrt(-gx*fy); x1 = cx+f; y1 = cy-f; }
	}
	if (gy > 0)
	{
		if (gx > 0) { f = sqrt(gx*gy); x2 = cx+f; y2 = cy+f; }
		if (fx < 0) { f = sqrt(-fx*gy); x3 = cx-f; y3 = cy+f; }
	}
	if (x0 > x1) { if (fx < 0) y0 = fx/gx*fy + cy; else y1 = gx/fx*fy + cy; }
	if (y1 > y2) { if (fy < 0) x1 = fy/gy*gx + cx; else x2 = gy/fy*gx + cx; }
	if (x2 < x3) { if (fx < 0) y3 = fx/gx*gy + cy; else y2 = gx/fx*gy + cy; }
	if (y3 < y0) { if (fy < 0) x0 = fy/gy*fx + cx; else x3 = gy/fy*fx + cx; }
		//This makes precision errors cause pixels to overwrite rather than omit
	x0 -= .01; x1 += .01;
	y1 -= .01; y2 += .01;
	x3 -= .01; x2 += .01;
	y0 -= .01; y3 += .01;

	f = cast(float)PREC / gihz;
	optistrx = gistr.x*f; optiheix = gihei.x*f; optiaddx = gcorn[0].x*f;
	optistry = gistr.y*f; optiheiy = gihei.y*f; optiaddy = gcorn[0].y*f;
//#ifdef _MSC_VER
	opti4[0].y = optistrx; opti4[0].z = optistrx*2; opti4[0].z2 = optistrx*3;
	opti4[1].y = optistry; opti4[1].z = optistry*2; opti4[1].z2 = optistry*3;
	opti4[2].x = opti4[2].y = opti4[2].z = opti4[2].z2 = optistrx*4.0f;
	opti4[3].x = opti4[3].y = opti4[3].z = opti4[3].z2 = optistry*4.0f;
	opti4[4].x = opti4[4].y = opti4[4].z = opti4[4].z2 = (optistrx*optistrx + optistry*optistry)*32.0f; //NEW ALGO!
//#endif

	ftol(cx*65536,&cx16);
	ftol(cy*65536,&cy16);

	ftol((x1-x0)/vx5.anginc,&j);
	if ((fy < 0) && (j > 0)) //(cx,cy),(x0,wy0),(x1,wy0)
	{
		ff = (x1-x0) / cast(float)j; grd = 1.0f / (wy0-cy);
		gscanptr = cast(castdat *)radar; skycurlng = -1; skycurdir = -giforzsgn;
		for(i=0,f=x0+ff*.5f;i<j;f+=ff,i++)
		{
			vline(cx,cy,f,wy0,&p0,&p1);
			if (giforzsgn < 0) angstart[i] = gscanptr+p0; else angstart[i] = gscanptr-p1;
			gscanptr += labs(p1-p0)+1;
		}

		j <<= 16; f = cast(float)j / ((x1-x0)*grd); ftol((cx-x0)*grd*f,&kadd);
		ftol(cx-.5f,&p1); p0 = lbound0(p1+1,xres); p1 = lbound0(p1,xres);
		ftol(cy-0.50005f,&sy); if (sy >= yres) sy = yres-1;
		ff = (fabs(cast(float)p1-cx)+1)*f/2147483647.0 + cy; //Anti-crash hack
		while ((ff < sy) && (sy >= 0)) sy--;
		if (sy >= 0)
		{
			ftol(f,&kmul);
			for(;sy>=0;sy--) if (isshldiv16safe(kmul,(sy<<16)-cy16)) break; //Anti-crash hack
			if (giforzsgn < 0) i = -sy; else i = sy;
			for(;sy>=0;sy--,i-=giforzsgn)
			{
				ui = shldiv16(kmul,(sy<<16)-cy16);
				u = mulshr16((p0<<16)-cx16,ui)+kadd;
				while ((p0 > 0) && (u >= ui)) { u -= ui; p0--; }
				u1 = (p1-p0)*ui + u;
				while ((p1 < xres) && (u1 < j)) { u1 += ui; p1++; }
				if (p0 < p1) hrend(p0,sy,p1,u,ui,i);
			}
			asm {emms;}
		}
	}

	ftol((y2-y1)/vx5.anginc,&j);
	if ((gx > 0) && (j > 0)) //(cx,cy),(wx1,y1),(wx1,y2)
	{
		ff = (y2-y1) / cast(float)j; grd = 1.0f / (wx1-cx);
		gscanptr = cast(castdat *)radar; skycurlng = -1; skycurdir = -giforzsgn;
		for(i=0,f=y1+ff*.5f;i<j;f+=ff,i++)
		{
			hline(cx,cy,wx1,f,&p0,&p1);
			if (giforzsgn < 0) angstart[i] = gscanptr-p0; else angstart[i] = gscanptr+p1;
			gscanptr += labs(p1-p0)+1;
		}

		j <<= 16; f = cast(float)j / ((y2-y1)*grd); ftol((cy-y1)*grd*f,&kadd);
		ftol(cy-.5f,&p1); p0 = lbound0(p1+1,yres); p1 = lbound0(p1,yres);
		ftol(cx+0.50005f,&sx); if (sx < 0) sx = 0;
		ff = (fabs(cast(float)p1-cy)+1)*f/2147483647.0 + cx; //Anti-crash hack
		while ((ff > sx) && (sx < xres)) sx++;
		if (sx < xres)
		{
			ftol(f,&kmul);
			for(;sx<xres;sx++) if (isshldiv16safe(kmul,(sx<<16)-cx16)) break; //Anti-crash hack
			for(;sx<xres;sx++)
			{
				ui = shldiv16(kmul,(sx<<16)-cx16);
				u = mulshr16((p0<<16)-cy16,ui)+kadd;
				while ((p0 > 0) && (u >= ui)) { u -= ui; lastx[--p0] = sx; }
				uurend[sx] = u; uurend[sx+MAXXDIM] = ui; u += (p1-p0)*ui;
				while ((p1 < yres) && (u < j)) { u += ui; lastx[p1++] = sx; }
			}
			if (giforzsgn < 0)
				  { for(sy=p0;sy<p1;sy++) vrend(lastx[sy],sy,xres,lastx[sy],1); }
			else { for(sy=p0;sy<p1;sy++) vrend(lastx[sy],sy,xres,-lastx[sy],-1); }
			asm {emms;}
		}
	}

	ftol((x2-x3)/vx5.anginc,&j);
	if ((gy > 0) && (j > 0)) //(cx,cy),(x2,wy1),(x3,wy1)
	{
		ff = (x2-x3) / cast(float)j; grd = 1.0f / (wy1-cy);
		gscanptr = cast(castdat *)radar; skycurlng = -1; skycurdir = giforzsgn;
		for(i=0,f=x3+ff*.5f;i<j;f+=ff,i++)
		{
			vline(cx,cy,f,wy1,&p0,&p1);
			if (giforzsgn < 0) angstart[i] = gscanptr-p0; else angstart[i] = gscanptr+p1;
			gscanptr += labs(p1-p0)+1;
		}

		j <<= 16; f = cast(float)j / ((x2-x3)*grd); ftol((cx-x3)*grd*f,&kadd);
		ftol(cx-.5f,&p1); p0 = lbound0(p1+1,xres); p1 = lbound0(p1,xres);
		ftol(cy+0.50005f,&sy); if (sy < 0) sy = 0;
		ff = (fabs(cast(float)p1-cx)+1)*f/2147483647.0 + cy; //Anti-crash hack
		while ((ff > sy) && (sy < yres)) sy++;
		if (sy < yres)
		{
			ftol(f,&kmul);
			for(;sy<yres;sy++) if (isshldiv16safe(kmul,(sy<<16)-cy16)) break; //Anti-crash hack
			if (giforzsgn < 0) i = sy; else i = -sy;
			for(;sy<yres;sy++,i-=giforzsgn)
			{
				ui = shldiv16(kmul,(sy<<16)-cy16);
				u = mulshr16((p0<<16)-cx16,ui)+kadd;
				while ((p0 > 0) && (u >= ui)) { u -= ui; p0--; }
				u1 = (p1-p0)*ui + u;
				while ((p1 < xres) && (u1 < j)) { u1 += ui; p1++; }
				if (p0 < p1) hrend(p0,sy,p1,u,ui,i);
			}
			asm {emms;}
		}
	}

	ftol((y3-y0)/vx5.anginc,&j);
	if ((fx < 0) && (j > 0)) //(cx,cy),(wx0,y3),(wx0,y0)
	{
		ff = (y3-y0) / cast(float)j; grd = 1.0f / (wx0-cx);
		gscanptr = cast(castdat *)radar; skycurlng = -1; skycurdir = giforzsgn;
		for(i=0,f=y0+ff*.5f;i<j;f+=ff,i++)
		{
			hline(cx,cy,wx0,f,&p0,&p1);
			if (giforzsgn < 0) angstart[i] = gscanptr+p0; else angstart[i] = gscanptr-p1;
			gscanptr += labs(p1-p0)+1;
		}

		j <<= 16; f = cast(float)j / ((y3-y0)*grd); ftol((cy-y0)*grd*f,&kadd);
		ftol(cy-.5f,&p1); p0 = lbound0(p1+1,yres); p1 = lbound0(p1,yres);
		ftol(cx-0.50005f,&sx); if (sx >= xres) sx = xres-1;
		ff = (fabs(cast(float)p1-cy)+1)*f/2147483647.0 + cx; //Anti-crash hack
		while ((ff < sx) && (sx >= 0)) sx--;
		if (sx >= 0)
		{
			ftol(f,&kmul);
			for(;sx>=0;sx--) if (isshldiv16safe(kmul,(sx<<16)-cx16)) break; //Anti-crash hack
			for(;sx>=0;sx--)
			{
				ui = shldiv16(kmul,(sx<<16)-cx16);
				u = mulshr16((p0<<16)-cy16,ui)+kadd;
				while ((p0 > 0) && (u >= ui)) { u -= ui; lastx[--p0] = sx; }
				uurend[sx] = u; uurend[sx+MAXXDIM] = ui; u += (p1-p0)*ui;
				while ((p1 < yres) && (u < j)) { u += ui; lastx[p1++] = sx; }
			}
			for(sy=p0;sy<p1;sy++) vrend(0,sy,lastx[sy]+1,0,giforzsgn);
			asm {emms;}
		}
	}
}

	//0: asm temp for current x
	//1: asm temp for current y
	//2: bottom (28)
	//3: top    ( 0)
	//4: left   ( 8)
	//5: right  (24)
	//6: up     (12)
	//7: down   (12)
	//setsideshades(0,0,0,0,0,0);
	//setsideshades(0,28,8,24,12,12);
void setsideshades (ubyte sto, ubyte sbo, ubyte sle, ubyte sri, ubyte sup, ubyte sdo)
{
	(cast(ubyte *)&gcsub[2])[7] = sbo; (cast(ubyte *)&gcsub[3])[7] = sto;
	(cast(ubyte *)&gcsub[4])[7] = sle; (cast(ubyte *)&gcsub[5])[7] = sri;
	(cast(ubyte *)&gcsub[6])[7] = sup; (cast(ubyte *)&gcsub[7])[7] = sdo;
	if (!(sto|sbo|sle|sri|sup|sdo))
	{
		vx5.sideshademode = 0;
		(cast(ubyte *)&gcsub[0])[7] = (cast(ubyte *)&gcsub[1])[7] = 0x00;
	}
	else vx5.sideshademode = 1;
}

	//MUST have more than: CEILING(max possible CLIPRADIUS) * 4 entries!
enum MAXCLIPIT = VSID*4; //VSID*2+4 is not a power of 2!
static lpoint2d clipit[MAXCLIPIT];

double findmaxcr (double px, double py, double pz, double cr)
{
	double f, g, maxcr, thresh2;
	int x, y, z, i0, i1, ix, y0, y1, z0, z1;
	ubyte *v;

	thresh2 = cr+1.7321+1; thresh2 *= thresh2;
	maxcr = cr*cr;

		//Find closest point of all nearby cubes to (px,py,pz)
	x = cast(int)px; y = cast(int)py; z = cast(int)pz; i0 = i1 = 0; ix = x; y0 = y1 = y;
	while (true)
	{
		f = max(cast(double)fabs(cast(double)x+.5-px)-0.5,0.0);
		g = max(cast(double)fabs(cast(double)y+.5-py)-0.5,0.0);
		f = f*f + g*g;
		if (f < maxcr)
		{
			if ((cast(uint)x >= VSID) || (cast(uint)y >= VSID))
				{ z0 = z1 = 0; }
			else
			{
				v = sptr[y*VSID+x];
				if (z >= v[1])
				{
					while (1)
					{
						if (!v[0]) { z0 = z1 = 0; break; }
						v += v[0]*4;
						if (z < v[1]) { z0 = v[3]; z1 = v[1]; break; }
					}
				}
				else { z0 = MAXZDIM-2048; z1 = v[1]; }
			}

			if ((pz <= z0) || (pz >= z1))
				maxcr = f;
			else
			{
				g = min(pz-cast(double)z0,cast(double)z1-pz);
				f += g*g; if (f < maxcr) maxcr = f;
			}
		}

		if ((x-px)*(x-px)+(y-py)*(y-py) < thresh2)
		{
			if ((x <= ix) && (x > 0))
				{ clipit[i1].x = x-1; clipit[i1].y = y; i1 = ((i1+1)&(MAXCLIPIT-1)); }
			if ((x >= ix) && (x < VSID-1))
				{ clipit[i1].x = x+1; clipit[i1].y = y; i1 = ((i1+1)&(MAXCLIPIT-1)); }
			if ((y <= y0) && (y > 0))
				{ clipit[i1].x = x; clipit[i1].y = y-1; i1 = ((i1+1)&(MAXCLIPIT-1)); y0--; }
			if ((y >= y1) && (y < VSID-1))
				{ clipit[i1].x = x; clipit[i1].y = y+1; i1 = ((i1+1)&(MAXCLIPIT-1)); y1++; }
		}
		if (i0 == i1) break;
		x = clipit[i0].x; y = clipit[i0].y; i0 = ((i0+1)&(MAXCLIPIT-1));
	}
	return(sqrt(maxcr));
}

static if(false)
{
		//Point: (x,y), line segment: (px,py)-(px+vx,py+vy)
		//Returns 1 if point is closer than sqrt(cr2) to line
	long dist2linept2d (double x, double y, double px, double py, double vx, double vy, double cr2)
	{
		double f, g;
		x -= px; y -= py; f = x*vx + y*vy; if (f <= 0) return(x*x + y*y <= cr2);
		g = vx*vx + vy*vy; if (f >= g) { x -= vx; y -= vy; return(x*x + y*y <= cr2); }
		x = x*g-vx*f; y = y*g-vy*f; return(x*x + y*y <= cr2*g*g);
	}

	static char clipbuf[MAXZDIM+16]; //(8 extra on each side)
	long sphtraceo (double px, double py, double pz,    //start pt
						double vx, double vy, double vz,    //move vector
						double *nx, double *ny, double *nz, //new pt after collision
						double *fx, double *fy, double *fz, //pt that caused collision
						double cr, double acr)
	{
		double t, u, ex, ey, ez, Za, Zb, Zc, thresh2;
		double vxyz, vyz, vxz, vxy, rvxyz, rvyz, rvxz, rvxy, rvx, rvy, rvz, cr2;
		int i, i0, i1, x, y, z, xx, yy, zz, v, vv, ix, y0, y1, z0, z1;
		char *vp;

		t = 1;
		(*nx) = px + vx;
		(*ny) = py + vy;
		(*nz) = pz + vz;

		z0 = max(cast(int)(min(pz,*nz)-cr)-2,-1);
		z1 = min(cast(int)(max(pz,*nz)+cr)+2,MAXZDIM);

		thresh2 = cr+1.7321+1; thresh2 *= thresh2;

		vyz = vz*vz; vxz = vx*vx; vxy = vy*vy; cr2 = cr*cr;
		vyz += vxy; vxy += vxz; vxyz = vyz + vxz; vxz += vz*vz;
		rvx = 1.0 / vx; rvy = 1.0 / vy; rvz = 1.0 / vz;
		rvyz = 1.0 / vyz; rvxz = 1.0 / vxz; rvxy = 1.0 / vxy;
		rvxyz = 1.0 / vxyz;

			//Algorithm fails (stops short) if cr < 2 :(
		i0 = i1 = 0; ix = x = cast(int)px; y = y0 = y1 = cast(int)py;
		while (1)
		{
			for(z=z0;z<=z1;z++) clipbuf[z+8] = 0;
			i = 16;
			for(yy=y;yy<y+2;yy++)
				for(xx=x;xx<x+2;xx++,i<<=1)
				{
					z = z0;
					if (cast(uint)(xx|yy) < VSID)
					{
						vp = sptr[yy*VSID+xx];
						while (true)
						{
							if (vp[1] > z) z = vp[1];
							if (!vp[0]) break;
							vp += vp[0]*4;
							zz = vp[3]; if (zz > z1) zz = z1;
							while (z < zz) clipbuf[(z++)+8] |= i;
						}
					}
					while (z <= z1) clipbuf[(z++)+8] |= i;
				}

			xx = x+1; yy = y+1; v = clipbuf[z0+8];
			for(z=z0;z<z1;z++)
			{
				zz = z+1; v = (v>>4)|clipbuf[zz+8];
				if ((!v) || (v == 255)) continue;

	//---------------Check 1(8) corners of cube (sphere intersection)-------------

				//if (((v-1)^v) >= v)  //True if v is: {1,2,4,8,16,32,64,128}
				if (!(v&(v-1)))      //Same as above, but {0,1,2,4,...} (v's never 0)
				{
					ex = xx-px; ey = yy-py; ez = zz-pz;
					Zb = ex*vx + ey*vy + ez*vz;
					Zc = ex*ex + ey*ey + ez*ez - cr2;
					u = Zb*Zb - vxyz*Zc;
					if (((cast(int *)&u)[1] | (cast(int *)&Zb)[1]) >= 0)
					//if ((u >= 0) && (Zb >= 0))
					{
							//   //Proposed compare optimization:
							//f = Zb*Zb-u; g = vxyz*t; h = (Zb*2-g)*g;
							//if ((unsigned __int64 *)&f < (unsigned __int64 *)&h)
						u = (Zb - sqrt(u)) * rvxyz;
						if ((u >= 0) && (u < t))
						{
							*fx = xx; *fy = yy; *fz = zz; t = u;
							*nx = vx*u + px; *ny = vy*u + py; *nz = vz*u + pz;
						}
					}
				}

	//---------------Check 3(12) edges of cube (cylinder intersection)-----------

				vv = v&0x55; if (((vv-1)^vv) >= vv)  //True if (v&0x55)={1,4,16,64}
				{
					ey = yy-py; ez = zz-pz;
					Zb = ey*vy + ez*vz;
					Zc = ey*ey + ez*ez - cr2;
					u = Zb*Zb - vyz*Zc;
					if (((cast(int *)&u)[1] | (cast(int *)&Zb)[1]) >= 0)
					//if ((u >= 0) && (Zb >= 0))
					{
						u = (Zb - sqrt(u)) * rvyz;
						if ((u >= 0) && (u < t))
						{
							ex = vx*u + px;
							if ((ex >= x) && (ex <= xx))
							{
								*fx = ex; *fy = yy; *fz = zz; t = u;
								*nx = ex; *ny = vy*u + py; *nz = vz*u + pz;
							}
						}
					}
				}
				vv = v&0x33; if (((vv-1)^vv) >= vv) //True if (v&0x33)={1,2,16,32}
				{
					ex = xx-px; ez = zz-pz;
					Zb = ex*vx + ez*vz;
					Zc = ex*ex + ez*ez - cr2;
					u = Zb*Zb - vxz*Zc;
					if (((cast(int *)&u)[1] | (cast(int *)&Zb)[1]) >= 0)
					//if ((u >= 0) && (Zb >= 0))
					{
						u = (Zb - sqrt(u)) * rvxz;
						if ((u >= 0) && (u < t))
						{
							ey = vy*u + py;
							if ((ey >= y) && (ey <= yy))
							{
								*fx = xx; *fy = ey; *fz = zz; t = u;
								*nx = vx*u + px; *ny = ey; *nz = vz*u + pz;
							}
						}
					}
				}
				vv = v&0x0f; if (((vv-1)^vv) >= vv) //True if (v&0x0f)={1,2,4,8}
				{
					ex = xx-px; ey = yy-py;
					Zb = ex*vx + ey*vy;
					Zc = ex*ex + ey*ey - cr2;
					u = Zb*Zb - vxy*Zc;
					if (((cast(int *)&u)[1] | (cast(int *)&Zb)[1]) >= 0)
					//if ((u >= 0) && (Zb >= 0))
					{
						u = (Zb - sqrt(u)) * rvxy;
						if ((u >= 0) && (u < t))
						{
							ez = vz*u + pz;
							if ((ez >= z) && (ez <= zz))
							{
								*fx = xx; *fy = yy; *fz = ez; t = u;
								*nx = vx*u + px; *ny = vy*u + py; *nz = ez;
							}
						}
					}
				}

	//---------------Check 3(6) faces of cube (plane intersection)---------------

				if (vx)
				{
					switch(v&0x03)
					{
						case 0x01: ex = xx+cr; if ((vx > 0) || (px < ex)) goto skipfacex; break;
						case 0x02: ex = xx-cr; if ((vx < 0) || (px > ex)) goto skipfacex; break;
						default: goto skipfacex;
					}
					u = (ex - px) * rvx;
					if ((u >= 0) && (u < t))
					{
						ey = vy*u + py;
						ez = vz*u + pz;
						if ((ey >= y) && (ey <= yy) && (ez >= z) && (ez <= zz))
						{
							*fx = xx; *fy = ey; *fz = ez; t = u;
							*nx = ex; *ny = ey; *nz = ez;
						}
					}
				}
				skipfacex:
				if (vy)
				{
					switch(v&0x05)
					{
						case 0x01: ey = yy+cr; if ((vy > 0) || (py < ey)) goto skipfacey; break;
						case 0x04: ey = yy-cr; if ((vy < 0) || (py > ey)) goto skipfacey; break;
						default: goto skipfacey;
					}
					u = (ey - py) * rvy;
					if ((u >= 0) && (u < t))
					{
						ex = vx*u + px;
						ez = vz*u + pz;
						if ((ex >= x) && (ex <= xx) && (ez >= z) && (ez <= zz))
						{
							*fx = ex; *fy = yy; *fz = ez; t = u;
							*nx = ex; *ny = ey; *nz = ez;
						}
					}
				}
				skipfacey:
				if (vz)
				{
					switch(v&0x11)
					{
						case 0x01: ez = zz+cr; if ((vz > 0) || (pz < ez)) goto skipfacez; break;
						case 0x10: ez = zz-cr; if ((vz < 0) || (pz > ez)) goto skipfacez; break;
						default: goto skipfacez;
					}
					u = (ez - pz) * rvz;
					if ((u >= 0) && (u < t))
					{
						ex = vx*u + px;
						ey = vy*u + py;
						if ((ex >= x) && (ex <= xx) && (ey >= y) && (ey <= yy))
						{
							*fx = ex; *fy = ey; *fz = zz; t = u;
							*nx = ex; *ny = ey; *nz = ez;
						}
					}
				}
				skipfacez:;
			}

			if ((x <= ix) && (x > 0) && (dist2linept2d(x-1,y,px,py,vx,vy,thresh2)))
				{ clipit[i1].x = x-1; clipit[i1].y = y; i1 = ((i1+1)&(MAXCLIPIT-1)); }
			if ((x >= ix) && (x < VSID-1) && (dist2linept2d(x+1,y,px,py,vx,vy,thresh2)))
				{ clipit[i1].x = x+1; clipit[i1].y = y; i1 = ((i1+1)&(MAXCLIPIT-1)); }
			if ((y <= y0) && (y > 0) && (dist2linept2d(x,y-1,px,py,vx,vy,thresh2)))
				{ clipit[i1].x = x; clipit[i1].y = y-1; i1 = ((i1+1)&(MAXCLIPIT-1)); y0--; }
			if ((y >= y1) && (y < VSID-1) && (dist2linept2d(x,y+1,px,py,vx,vy,thresh2)))
				{ clipit[i1].x = x; clipit[i1].y = y+1; i1 = ((i1+1)&(MAXCLIPIT-1)); y1++; }
			if (i0 == i1) break;
			x = clipit[i0].x; y = clipit[i0].y; i0 = ((i0+1)&(MAXCLIPIT-1));
		}

		if ((*nx) < acr) (*nx) = acr;
		if ((*ny) < acr) (*ny) = acr;
		if ((*nx) > VSID-acr) (*nx) = VSID-acr;
		if ((*ny) > VSID-acr) (*ny) = VSID-acr;
		if ((*nz) > MAXZDIM-1-acr) (*nz) = MAXZDIM-1-acr;
		if ((*nz) < MAXZDIM-2048) (*nz) = MAXZDIM-2048;

		return (t == 1);
	}	
}

static double gx0, gy0, gcrf2, grdst, gendt, gux, guy;
static int gdist2square (double x, double y)
{
	double t;
	x -= gx0; y -= gy0; t = x*gux + y*guy; if (t <= 0) t = gcrf2;
	else if (t*grdst >= gendt) { x -= gux*gendt; y -= guy*gendt; t = gcrf2; }
	else t = t*t*grdst + gcrf2;
	return(x*x + y*y <= t);
}

int sphtrace (double x0, double y0, double z0,          //start pt
					double vx, double vy, double vz,          //move vector
					double *hitx, double *hity, double *hitz, //new pt after collision
					double *clpx, double *clpy, double *clpz, //pt causing collision
					double cr, double acr)
{
	double f, t, dax, day, daz, vyx, vxy, vxz, vyz, rvz, cr2, fz, fc;
	double dx, dy, dx1, dy1;
	double nx, ny, intx, inty, intz, dxy, dxz, dyz, dxyz, rxy, rxz, ryz, rxyz;
	int i, j, x, y, ix, iy0, iy1, i0, i1, cz0, cz1;
	int[2] iz;
	ubyte *v;

		 //Precalculate global constants for ins & getval functions
	if ((vx == 0) && (vy == 0) && (vz == 0))
		{ (*hitx) = x0; (*hity) = y0; (*hitz) = z0; return(1); }
	gux = vx; guy = vy; gx0 = x0; gy0 = y0; dxy = vx*vx + vy*vy;
	if (dxy != 0) rxy = 1.0 / dxy; else rxy = 0;
	grdst = rxy; gendt = 1; cr2 = cr*cr; t = cr + 0.7072; gcrf2 = t*t;

	if ((cast(int *)&vz)[1] >= 0) { dtol(   z0-cr-.5,&cz0); dtol(vz+z0+cr-.5,&cz1); }
								 else { dtol(vz+z0-cr-.5,&cz0); dtol(   z0+cr-.5,&cz1); }

		//Precalculate stuff for closest point on cube finder
	dax = 0; day = 0; vyx = 0; vxy = 0; rvz = 0; vxz = 0; vyz = 0;
	if (vx != 0) { vyx = vy/vx; if ((cast(int *)&vx)[1] >= 0) dax = x0+cr; else dax = x0-cr-1; }
	if (vy != 0) { vxy = vx/vy; if ((cast(int *)&vy)[1] >= 0) day = y0+cr; else day = y0-cr-1; }
	if (vz != 0)
	{
		rvz = 1.0/vz; vxz = vx*rvz; vyz = vy*rvz;
		if ((cast(int *)&vz)[1] >= 0) daz = z0+cr; else daz = z0-cr;
	}

	dxyz = vz*vz;
	dxz = vx*vx+dxyz; if (dxz != 0) rxz = 1.0 / dxz;
	dyz = vy*vy+dxyz; if (dyz != 0) ryz = 1.0 / dyz;
	dxyz += dxy; rxyz = 1.0 / dxyz;

	dtol(x0-.5,&x); dtol(y0-.5,&y);
	ix = x; iy0 = iy1 = y;
	i0 = 0; clipit[0].x = x; clipit[0].y = y; i1 = 1;
	do
	{
		x = clipit[i0].x; y = clipit[i0].y; i0 = ((i0+1)&(MAXCLIPIT-1));

		dx = cast(double)x; dx1 = cast(double)(x+1);
		dy = cast(double)y; dy1 = cast(double)(y+1);

			//closest point on cube finder
			//Plane intersection (both vertical planes)
		static if(false)
		{
			intx = dbound((dy-day)*vxy + x0,dx,dx1);
			inty = dbound((dx-dax)*vyx + y0,dy,dy1);
		} else
		{
			intx = (dy-day)*vxy + x0;
			inty = (dx-dax)*vyx + y0;
			if ((cast(int *)&intx)[1] < (cast(int *)&dx)[1]) intx = dx;
			if ((cast(int *)&inty)[1] < (cast(int *)&dy)[1]) inty = dy;
			if ((cast(int *)&intx)[1] >= (cast(int *)&dx1)[1]) intx = dx1;
			if ((cast(int *)&inty)[1] >= (cast(int *)&dy1)[1]) inty = dy1;
			//if (intx < (double)x) intx = (double)x;
			//if (inty < (double)y) inty = (double)y;
			//if (intx > (double)(x+1)) intx = (double)(x+1);
			//if (inty > (double)(y+1)) inty = (double)(y+1);
		}

		do
		{
			if ((cast(int *)&dxy)[1] == 0) { t = -1.0; continue; }
			nx = intx-x0; ny = inty-y0; t = vx*nx + vy*ny; if ((cast(int *)&t)[1] < 0) continue;
			f = cr2 - nx*nx - ny*ny; if ((cast(int *)&f)[1] >= 0) { t = -1.0; continue; }
			f = f*dxy + t*t; if ((cast(int *)&f)[1] < 0) { t = -1.0; continue; }
			t = (t-sqrt(f))*rxy;
		} while (0);
		if (t >= gendt) goto sphtracecont;
		if ((cast(int *)&t)[1] < 0) intz = z0; else intz = vz*t + z0;

			//Find closest ceil(iz[0]) & flor(iz[1]) in (x,y) column
		dtol(intz-.5,&i);
		if (cast(uint)(x|y) < VSID)
		{
			v = sptr[y*VSID+x]; iz[0] = MAXZDIM-2048; iz[1] = v[1];
			while (i >= iz[1])
			{
				if (!v[0]) { iz[1] = -1; break; }
				v += v[0]*4;
				iz[0] = v[3]; if (i < iz[0]) { iz[1] = -1; break; }
				iz[1] = v[1];
			}
		}
		else iz[1] = -1;

			//hit xz plane, yz plane or z-axis edge?
		if (iz[1] < 0) //Treat whole column as solid
		{
			if ((cast(int *)&t)[1] >= 0) { gendt = t; (*clpx) = intx; (*clpy) = inty; (*clpz) = intz; goto sphtracecont; }
		}

			//Must check tops & bottoms of slab
		for(i=1;i>=0;i--)
		{
				//Ceil/flor outside of quick&dirty bounding box
			if ((iz[i] < cz0) || (iz[i] > cz1)) continue;

				//Plane intersection (parallel to ground)
			intz = cast(double)iz[i]; t = intz-daz;
			intx = t*vxz + x0;
			inty = t*vyz + y0;

			j = 0;                         // A ³ 8 ³ 9
			//     if (intx < dx)  j |= 2; //ÄÄÄÅÄÄÄÅÄÄÄ
			//else if (intx > dx1) j |= 1; // 2 ³ 0 ³ 1
			//     if (inty < dy)  j |= 8; //ÄÄÄÅÄÄÄÅÄÄÄ
			//else if (inty > dy1) j |= 4; // 6 ³ 4 ³ 5
				  if ((cast(int *)&intx)[1] <  (cast(int *)&dx)[1])  j |= 2;
			else if ((cast(int *)&intx)[1] >= (cast(int *)&dx1)[1]) j |= 1;
				  if ((cast(int *)&inty)[1] <  (cast(int *)&dy)[1])  j |= 8;
			else if ((cast(int *)&inty)[1] >= (cast(int *)&dy1)[1]) j |= 4;

				//NOTE: only need to check once per "for"!
			if ((!j) && (vz != 0)) //hit xy plane?
			{
				t *= rvz;
				if (((cast(int *)&t)[1] >= 0) && (t < gendt)) { gendt = t; (*clpx) = intx; (*clpy) = inty; (*clpz) = intz; }
				continue;
			}

				//common calculations used for rest of checks...
			fz = intz-z0; fc = cr2-fz*fz; fz *= vz;

			if (j&3)
			{
				nx = cast(double)((j&1)+x);
				if ((cast(int *)&dxz)[1] != 0) //hit y-axis edge?
				{
					f = nx-x0; t = vx*f + fz; f = (fc - f*f)*dxz + t*t;
					if ((cast(int *)&f)[1] >= 0) t = (t-sqrt(f))*rxz; else t = -1.0;
				} else t = -1.0;
				ny = vy*t + y0;
					  if ((cast(int *)&ny)[1] > (cast(int *)&dy1)[1]) j |= 0x10;
				else if ((cast(int *)&ny)[1] >= (cast(int *)&dy)[1])
				{
					if (((cast(int *)&t)[1] >= 0) && (t < gendt)) { gendt = t; (*clpx) = nx; (*clpy) = ny; (*clpz) = intz; }
					continue;
				}
				inty = cast(double)(((j>>4)&1)+y);
			}
			else inty = cast(double)(((j>>2)&1)+y);

			if (j&12)
			{
				ny = cast(double)(((j>>2)&1)+y);
				if ((cast(int *)&dyz)[1] != 0) //hit x-axis edge?
				{
					f = ny-y0; t = vy*f + fz; f = (fc - f*f)*dyz + t*t;
					if ((cast(int *)&f)[1] >= 0) t = (t-sqrt(f))*ryz; else t = -1.0;
				} else t = -1.0;
				nx = vx*t + x0;
					  if ((cast(int *)&nx)[1] > (cast(int *)&dx1)[1]) j |= 0x20;
				else if ((cast(int *)&nx)[1] >= (cast(int *)&dx)[1])
				{
					if (((cast(int *)&t)[1] >= 0) && (t < gendt)) { gendt = t; (*clpx) = nx; (*clpy) = ny; (*clpz) = intz; }
					continue;
				}
				intx = cast(double)(((j>>5)&1)+x);
			}
			else intx = cast(double)((j&1)+x);

				//hit corner?
			nx = intx-x0; ny = inty-y0;
			t = vx*nx + vy*ny + fz; if ((cast(int *)&t)[1] < 0) continue;
			f = fc - nx*nx - ny*ny; if ((cast(int *)&f)[1] >= 0) continue;
			f = f*dxyz + t*t; if ((cast(int *)&f)[1] < 0) continue;
			t = (t-sqrt(f))*rxyz;
			if (t < gendt) { gendt = t; (*clpx) = intx; (*clpy) = inty; (*clpz) = intz; }
		}
sphtracecont:;
		if ((x <= ix)  && (x >      0) && (gdist2square(dx- .5,dy+ .5))) { clipit[i1].x = x-1; clipit[i1].y = y; i1 = ((i1+1)&(MAXCLIPIT-1)); }
		if ((x >= ix)  && (x < VSID-1) && (gdist2square(dx+1.5,dy+ .5))) { clipit[i1].x = x+1; clipit[i1].y = y; i1 = ((i1+1)&(MAXCLIPIT-1)); }
		if ((y <= iy0) && (y >      0) && (gdist2square(dx+ .5,dy- .5))) { clipit[i1].x = x; clipit[i1].y = y-1; i1 = ((i1+1)&(MAXCLIPIT-1)); iy0 = y-1; }
		if ((y >= iy1) && (y < VSID-1) && (gdist2square(dx+ .5,dy+1.5))) { clipit[i1].x = x; clipit[i1].y = y+1; i1 = ((i1+1)&(MAXCLIPIT-1)); iy1 = y+1; }
	} while (i0 != i1);
	static if(true)
	{
		(*hitx) = dbound(vx*gendt + x0,acr,VSID-acr);
		(*hity) = dbound(vy*gendt + y0,acr,VSID-acr);
		(*hitz) = dbound(vz*gendt + z0,MAXZDIM-2048+acr,MAXZDIM-1-acr);
	} else
	{
		(*hitx) = min(max(vx*gendt + x0,acr),VSID-acr);
		(*hity) = min(max(vy*gendt + y0,acr),VSID-acr);
		(*hitz) = min(max(vz*gendt + z0,MAXZDIM-2048+acr),MAXZDIM-1-acr);
	}
	return(gendt == 1);
}

void clipmove (dpoint3d *p, dpoint3d *v, double acr)
{
	double f, gx, gy, gz, nx, ny, nz, ex, ey, ez, hitx, hity, hitz, cr;
	//double nx2, ny2, nz2, ex2, ey2, ez2; //double ox, oy, oz;
	int i, j, k;

	//ox = p.x; oy = p.y; oz = p.z;
	gx = p.x+v.x; gy = p.y+v.y; gz = p.z+v.z;

	cr = findmaxcr(p.x,p.y,p.z,acr);
	vx5.clipmaxcr = cr;

	vx5.cliphitnum = 0;
	for(i=0;i<3;i++)
	{
		if ((v.x == 0) && (v.y == 0) && (v.z == 0)) break;

		cr -= 1e-7;  //Shrinking radius error control hack

		//j = sphtraceo(p.x,p.y,p.z,v.x,v.y,v.z,&nx,&ny,&nz,&ex,&ey,&ez,cr,acr);
		//k = sphtraceo(p.x,p.y,p.z,v.x,v.y,v.z,&nx2,&ny2,&nz2,&ex2,&ey2,&ez2,cr,acr);

		j = sphtrace(p.x,p.y,p.z,v.x,v.y,v.z,&nx,&ny,&nz,&ex,&ey,&ez,cr,acr);

		//if ((j != k) || (fabs(nx-nx2) > .000001) || (fabs(ny-ny2) > .000001) || (fabs(nz-nz2) > .000001) ||
		//   ((j == 0) && ((fabs(ex-ex2) > .000001) || (fabs(ey-ey2) > .000001) || (fabs(ez-ez2) > .000001))))
		//{
		//   printf("%d %f %f %f %f %f %f\n",i,p.x,p.y,p.z,v.x,v.y,v.z);
		//   printf("%f %f %f ",nx,ny,nz); if (!j) printf("%f %f %f\n",ex,ey,ez); else printf("\n");
		//   printf("%f %f %f ",nx2,ny2,nz2); if (!k) printf("%f %f %f\n",ex2,ey2,ez2); else printf("\n");
		//   printf("\n");
		//}
		if (j) { p.x = nx; p.y = ny; p.z = nz; break; }

		vx5.cliphit[i].x = ex; vx5.cliphit[i].y = ey; vx5.cliphit[i].z = ez;
		vx5.cliphitnum = i+1;
		p.x = nx; p.y = ny; p.z = nz;

			//Calculate slide vector
		v.x = gx-nx; v.y = gy-ny; v.z = gz-nz;
		switch(i)
		{
			case 0:
				hitx = ex-nx; hity = ey-ny; hitz = ez-nz;
				f = (v.x*hitx + v.y*hity + v.z*hitz) / (cr * cr);
				v.x -= hitx*f; v.y -= hity*f; v.z -= hitz*f;
				break;
			case 1:
				nx -= ex; ny -= ey; nz -= ez;
				ex = hitz*ny - hity*nz;
				ey = hitx*nz - hitz*nx;
				ez = hity*nx - hitx*ny;
				f = ex*ex + ey*ey + ez*ez; if (f <= 0) break;
				f = (v.x*ex + v.y*ey + v.z*ez) / f;
				v.x = ex*f; v.y = ey*f; v.z = ez*f;
				break;
			default: break;
		}
	}

		//If you didn't move much, then don't move at all. This helps prevents
		//cliprad from shrinking, but you get stuck too much :(
	//if ((p.x-ox)*(p.x-ox) + (p.y-oy)*(p.y-oy) + (p.z-oz)*(p.z-oz) < 1e-12)
	//   { p.x = ox; p.y = oy; p.z = oz; }
}

int cansee (point3d *p0, point3d *p1, lpoint3d *hit)
{
	lpoint3d a, c, d, p, i;
	point3d f, g;
	int cnt;

	ftol(p0.x-.5,&a.x); ftol(p0.y-.5,&a.y); ftol(p0.z-.5,&a.z);
	if (isvoxelsolid(a.x,a.y,a.z)) { hit.x = a.x; hit.y = a.y; hit.z = a.z; return(0); }
	ftol(p1.x-.5,&c.x); ftol(p1.y-.5,&c.y); ftol(p1.z-.5,&c.z);
	cnt = 0;

		  if (c.x <  a.x) { d.x = -1; f.x = p0.x-a.x;   g.x = (p0.x-p1.x)*1024; cnt += a.x-c.x; }
	else if (c.x != a.x) { d.x =  1; f.x = a.x+1-p0.x; g.x = (p1.x-p0.x)*1024; cnt += c.x-a.x; }
	else f.x = g.x = 0;
		  if (c.y <  a.y) { d.y = -1; f.y = p0.y-a.y;   g.y = (p0.y-p1.y)*1024; cnt += a.y-c.y; }
	else if (c.y != a.y) { d.y =  1; f.y = a.y+1-p0.y; g.y = (p1.y-p0.y)*1024; cnt += c.y-a.y; }
	else f.y = g.y = 0;
		  if (c.z <  a.z) { d.z = -1; f.z = p0.z-a.z;   g.z = (p0.z-p1.z)*1024; cnt += a.z-c.z; }
	else if (c.z != a.z) { d.z =  1; f.z = a.z+1-p0.z; g.z = (p1.z-p0.z)*1024; cnt += c.z-a.z; }
	else f.z = g.z = 0;

	ftol(f.x*g.z - f.z*g.x,&p.x); ftol(g.x,&i.x);
	ftol(f.y*g.z - f.z*g.y,&p.y); ftol(g.y,&i.y);
	ftol(f.y*g.x - f.x*g.y,&p.z); ftol(g.z,&i.z);

		//NOTE: GIGO! This can happen if p0,p1 (cansee input) is NaN, Inf, etc...
	if (cast(ulong)cnt > (VSID+VSID+2048)*2) cnt = (VSID+VSID+2048)*2;
	while (cnt > 0)
	{
		if (((p.x|p.y) >= 0) && (a.z != c.z)) { a.z += d.z; p.x -= i.x; p.y -= i.y; }
		else if ((p.z >= 0) && (a.x != c.x))  { a.x += d.x; p.x += i.z; p.z -= i.y; }
		else                                  { a.y += d.y; p.y += i.z; p.z += i.x; }
		if (isvoxelsolid(a.x,a.y,a.z)) break;
		cnt--;
	}
	hit.x = a.x; hit.y = a.y; hit.z = a.z; return(!cnt);
}

	//  p: start position
	//  d: direction
	//  h: coordinate of voxel hit (if any)
	//ind: pointer to surface voxel's 32-bit color (0 if none hit)
	//dir: 0-5: last direction moved upon hit (-1 if inside solid)
void hitscan (dpoint3d *p, dpoint3d *d, lpoint3d *h, int **ind, int *dir)
{
	int ixi, iyi, izi, dx, dy, dz, dxi, dyi, dzi, z0, z1, minz;
	float f, kx, ky, kz;
	ubyte *v;

		//Note: (h.x,h.y,h.z) MUST be rounded towards -inf
	(h.x) = cast(int)p.x;
	(h.y) = cast(int)p.y;
	(h.z) = cast(int)p.z;
	if (cast(uint)(h.x|h.y) >= VSID) { (*ind) = null; (*dir) = -1; return; }
	ixi = ((((cast(int *)&d.x)[1])>>31)|1);
	iyi = ((((cast(int *)&d.y)[1])>>31)|1);
	izi = ((((cast(int *)&d.z)[1])>>31)|1);

	minz = min(h.z,0);

	f = 0x3fffffff/VSID; //Maximum delta value
	if ((fabs(d.x) >= fabs(d.y)) && (fabs(d.x) >= fabs(d.z)))
	{
		kx = 1024.0;
		if (d.y == 0) ky = f; else ky = min(cast(float)fabs(d.x/d.y)*1024.0f,f);
		if (d.z == 0) kz = f; else kz = min(cast(float)fabs(d.x/d.z)*1024.0f,f);
	}
	else if (fabs(d.y) >= fabs(d.z))
	{
		ky = 1024.0;
		if (d.x == 0) kx = f; else kx = min(cast(float)fabs(d.y/d.x)*1024.0f,f);
		if (d.z == 0) kz = f; else kz = min(cast(float)fabs(d.y/d.z)*1024.0f,f);
	}
	else
	{
		kz = 1024.0;
		if (d.x == 0) kx = f; else kx = min(cast(float)fabs(d.z/d.x)*1024.0f,f);
		if (d.y == 0) ky = f; else ky = min(cast(float)fabs(d.z/d.y)*1024.0f,f);
	}
	ftol(kx,&dxi); ftol((p.x-cast(float)h.x)*kx,&dx); if (ixi >= 0) dx = dxi-dx;
	ftol(ky,&dyi); ftol((p.y-cast(float)h.y)*ky,&dy); if (iyi >= 0) dy = dyi-dy;
	ftol(kz,&dzi); ftol((p.z-cast(float)h.z)*kz,&dz); if (izi >= 0) dz = dzi-dz;

	v = sptr[h.y*VSID+h.x];
	if (h.z >= v[1])
	{
		do
		{
			if (!v[0]) { (*ind) = null; (*dir) = -1; return; }
			v += v[0]*4;
		} while (h.z >= v[1]);
		z0 = v[3];
	} else z0 = minz;
	z1 = v[1];

	while (true)
	{
		//Check cube at: h.x,h.y,h.z

		if ((dz <= dx) && (dz <= dy))
		{
			h.z += izi; dz += dzi; (*dir) = 5-(izi>0);

				//Check if h.z ran into anything solid
			if (h.z < z0)
			{
				if (h.z < minz) (*ind) = null; else (*ind) = cast(int *)&v[-4];
				return;
			}
			if (h.z >= z1) { (*ind) = cast(int *)&v[4]; return; }
		}
		else
		{
			if (dx < dy)
			{
				h.x += ixi; dx += dxi; (*dir) = 1-(ixi>0);
				if (cast(uint)h.x >= VSID) { (*ind) = null; return; }
			}
			else
			{
				h.y += iyi; dy += dyi; (*dir) = 3-(iyi>0);
				if (cast(uint)h.y >= VSID) { (*ind) = null; return; }
			}

				//Check if (h.x, h.y) ran into anything solid
			v = sptr[h.y*VSID+h.x];
			while (true)
			{
				if (h.z < v[1])
				{
					if (v == sptr[h.y*VSID+h.x]) { z0 = minz; z1 = v[1]; break; }
					if (h.z < v[3]) { (*ind) = cast(int *)&v[(h.z-v[3])*4]; return; }
					z0 = v[3]; z1 = v[1]; break;
				}
				else if ((h.z <= v[2]) || (!v[0]))
					{ (*ind) = cast(int *)&v[(h.z-v[1])*4+4]; return; }

				v += v[0]*4;
			}
		}
	}
}

	// p0: start position
	// v0: direction
	//spr: pointer of sprite to test collision with
	//  h: coordinate of voxel hit in sprite coordinates (if any)
	//ind: pointer to voxel hit (kv6voxtype) (0 if none hit)
	//vsc:  input: max multiple/fraction of v0's length to scan (1.0 for |v0|)
	//     output: multiple/fraction of v0's length of hit point
void sprhitscan (dpoint3d *p0, dpoint3d *v0, vx5sprite *spr, lpoint3d *h, kv6voxtype **ind, float *vsc)
{
	kv6voxtype *vx[4];
	kv6data *kv;
	point3d t, u, v;
	lpoint3d a, d, p, q;
	float f, g;
	int i, x, y, xup, ix0, ix1;

	(*ind) = null;
	if (spr.flags&2)
	{
		kfatype *kf = spr.kfaptr;
			//This sets the sprite pointer to be the parent sprite (voxnum
			//   of the main sprite is invalid for KFA sprites!)
		spr = &kf.spr[kf.hingesort[(kf.numhin)-1]];
	}
	kv = spr.voxnum; if (!kv) return;

		//d transformed to spr space (0,0,0,kv.xsiz,kv.ysiz,kv.zsiz)
	v.x = v0.x*spr.s.x + v0.y*spr.s.y + v0.z*spr.s.z;
	v.y = v0.x*spr.h.x + v0.y*spr.h.y + v0.z*spr.h.z;
	v.z = v0.x*spr.f.x + v0.y*spr.f.y + v0.z*spr.f.z;

		//p transformed to spr space (0,0,0,kv.xsiz,kv.ysiz,kv.zsiz)
	t.x = p0.x-spr.p.x;
	t.y = p0.y-spr.p.y;
	t.z = p0.z-spr.p.z;
	u.x = t.x*spr.s.x + t.y*spr.s.y + t.z*spr.s.z;
	u.y = t.x*spr.h.x + t.y*spr.h.y + t.z*spr.h.z;
	u.z = t.x*spr.f.x + t.y*spr.f.y + t.z*spr.f.z;
	u.x /= (spr.s.x*spr.s.x + spr.s.y*spr.s.y + spr.s.z*spr.s.z);
	u.y /= (spr.h.x*spr.h.x + spr.h.y*spr.h.y + spr.h.z*spr.h.z);
	u.z /= (spr.f.x*spr.f.x + spr.f.y*spr.f.y + spr.f.z*spr.f.z);
	u.x += kv.xpiv; u.y += kv.ypiv; u.z += kv.zpiv;

	ix0 = max(vx5.xplanemin,0);
	ix1 = min(vx5.xplanemax,kv.xsiz);

		//Increment ray until it hits bounding box
		// (ix0,0,0,ix1-1ulp,kv.ysiz-1ulp,kv.zsiz-1ulp)
	g = cast(float)ix0;
	t.x = cast(float)ix1;      (*cast(int *)&t.x)--;
	t.y = cast(float)kv.ysiz; (*cast(int *)&t.y)--;
	t.z = cast(float)kv.zsiz; (*cast(int *)&t.z)--;
		  if (u.x <   g) { if (v.x <= 0) return; f = (  g-u.x)/v.x; u.x =   g; u.y += v.y*f; u.z += v.z*f; }
	else if (u.x > t.x) { if (v.x >= 0) return; f = (t.x-u.x)/v.x; u.x = t.x; u.y += v.y*f; u.z += v.z*f; }
		  if (u.y <   0) { if (v.y <= 0) return; f = (  0-u.y)/v.y; u.y =   0; u.x += v.x*f; u.z += v.z*f; }
	else if (u.y > t.y) { if (v.y >= 0) return; f = (t.y-u.y)/v.y; u.y = t.y; u.x += v.x*f; u.z += v.z*f; }
		  if (u.z <   0) { if (v.z <= 0) return; f = (  0-u.z)/v.z; u.z =   0; u.x += v.x*f; u.y += v.y*f; }
	else if (u.z > t.z) { if (v.z >= 0) return; f = (t.z-u.z)/v.z; u.z = t.z; u.x += v.x*f; u.y += v.y*f; }

	ix1 -= ix0;

	g = 262144.0 / sqrt(v.x*v.x + v.y*v.y + v.z*v.z);

		//Note: (a.x,a.y,a.z) MUST be rounded towards -inf
	ftol(u.x-.5,&a.x); if (cast(uint)(a.x-ix0) >= ix1) return;
	ftol(u.y-.5,&a.y); if (cast(uint)a.y >= kv.ysiz) return;
	ftol(u.z-.5,&a.z); if (cast(uint)a.z >= kv.zsiz) return;
	if (*cast(int *)&v.x < 0) { d.x = -1; u.x -= a.x;      v.x *= -g; }
							else { d.x =  1; u.x = a.x+1-u.x; v.x *=  g; }
	if (*cast(int *)&v.y < 0) { d.y = -1; u.y -= a.y;      v.y *= -g; }
							else { d.y =  1; u.y = a.y+1-u.y; v.y *=  g; }
	if (*cast(int *)&v.z < 0) { d.z = -1; u.z -= a.z;      v.z *= -g; }
							else { d.z =  1; u.z = a.z+1-u.z; v.z *=  g; }
	ftol(u.x*v.z - u.z*v.x,&p.x); ftol(v.x,&q.x);
	ftol(u.y*v.z - u.z*v.y,&p.y); ftol(v.y,&q.y);
	ftol(u.y*v.x - u.x*v.y,&p.z); ftol(v.z,&q.z);

		//Check if voxel at: (a.x,a.y,a.z) is solid
	vx[0] = kv.vox;
	for(x=0;x<a.x;x++) vx[0] += kv.xlen[x];
	vx[1] = vx[0]; xup = x*kv.ysiz;
	for(y=0;y<a.y;y++) vx[1] += kv.ylen[xup+y];
	vx[2] = vx[1]; vx[3] = &vx[1][kv.ylen[xup+y]];

	while (1)
	{
		//vs = kv.vox; //Brute force: remove all vx[?] code to enable this
		//for(x=0;x<a.x;x++) vs += kv.xlen[x];
		//for(y=0;y<a.y;y++) vs += kv.ylen[x*kv.ysiz+y];
		//for(ve=&vs[kv.ylen[x+y]];vs<ve;vs++) if (vs.z == a.z) break;

			//Check if voxel at: (a.x,a.y,a.z) is solid
		if (vx[1] < vx[3])
		{
			while ((a.z < vx[2].z) && (vx[2] > vx[1]  )) vx[2]--;
			while ((a.z > vx[2].z) && (vx[2] < vx[3]-1)) vx[2]++;
			if (a.z == vx[2].z) break;
		}

		if ((p.x|p.y) >= 0)
		{
			a.z += d.z; if (cast(uint)a.z >= kv.zsiz) return;
			p.x -= q.x; p.y -= q.y;
		}
		else if (p.z < 0)
		{
			a.y += d.y; if (cast(uint)a.y >= kv.ysiz) return;
			p.y += q.z; p.z += q.x;

			if (a.y < y) { y--; vx[1] -= kv.ylen[xup+y];      }
			if (a.y > y) {      vx[1] += kv.ylen[xup+y]; y++; }
			vx[2] = vx[1]; vx[3] = &vx[1][kv.ylen[xup+y]];
		}
		else
		{
			a.x += d.x; if (cast(uint)(a.x-ix0) >= ix1) return;
			p.x += q.z; p.z -= q.y;

			if (a.x < x) { x--; vx[0] -= kv.xlen[x];      xup -= kv.ysiz; }
			if (a.x > x) {      vx[0] += kv.xlen[x]; x++; xup += kv.ysiz; }
			if ((a.y<<1) < kv.ysiz) //Start y-slice search from closer side
			{
				vx[1] = vx[0];
				for(y=0;y<a.y;y++) vx[1] += kv.ylen[xup+y];
			}
			else
			{
				vx[1] = &vx[0][kv.xlen[x]];
				for(y=kv.ysiz;y>a.y;y--) vx[1] -= kv.ylen[xup+y-1];
			}
			vx[2] = vx[1]; vx[3] = &vx[1][kv.ylen[xup+y]];
		}
	}

		//given: a = kv6 coordinate, find: v = vxl coordinate
	u.x = cast(float)a.x-kv.xpiv;
	u.y = cast(float)a.y-kv.ypiv;
	u.z = cast(float)a.z-kv.zpiv;
	v.x = u.x*spr.s.x + u.y*spr.h.x + u.z*spr.f.x + spr.p.x;
	v.y = u.x*spr.s.y + u.y*spr.h.y + u.z*spr.f.y + spr.p.y;
	v.z = u.x*spr.s.z + u.y*spr.h.z + u.z*spr.f.z + spr.p.z;

		//Stupid dot product stuff...
	f = ((v.x-p0.x)*v0.x + (v.y-p0.y)*v0.y + (v.z-p0.z)*v0.z) /
		  (v0.x*v0.x + v0.y*v0.y + v0.z*v0.z);
	if (f >= (*vsc)) return;
	{ (*vsc) = f; (*h) = a; (*ind) = vx[2]; (*vsc) = f; }
}

uint calcglobalmass ()
{
	uint i, j;
	ubyte *v;

	j = VSID*VSID*256;
	for(i=0;i<VSID*VSID;i++)
	{
		v = sptr[i]; j -= v[1];
		while (v[0]) { v += v[0]*4; j += v[3]-v[1]; }
	}
	return(j);
}

void loadnul (dpoint3d *ipo, dpoint3d *ist, dpoint3d *ihe, dpoint3d *ifo)
{
	lpoint3d lp0, lp1;
	int i, x, y;
	ubyte *v;
	float f;

	if (!vbuf) { vbuf = cast(int *)malloc((VOXSIZ>>2)<<2); if (!vbuf) evilquit("vbuf malloc failed"); }
	if (!vbit) { vbit = cast(int *)malloc((VOXSIZ>>7)<<2); if (!vbit) evilquit("vbuf malloc failed"); }

	v = cast(ubyte *)(&vbuf[1]); //1st dword for voxalloc compare logic optimization

		//Completely re-compile vbuf
	for(x=0;x<VSID;x++)
		for(y=0;y<VSID;y++)
		{
			sptr[y*VSID+x] = v;
			i = 0; // + (rand()&1);  //i = default height of plain
			v[0] = 0;
			v[1] = cast(ubyte)i;
			v[2] = cast(ubyte)i;
			v[3] = 0;  //z0 (Dummy filler)
			//i = ((((x+y)>>3) + ((x^y)>>4)) % 231) + 16;
			//i = (i<<16)+(i<<8)+i;
			v += 4;
			(*cast(int *)v) = ((x^y)&15)*0x10101+0x807c7c7c; //colorjit(i,0x70707)|0x80000000;
			v += 4;
		}

	memset(&sptr[VSID*VSID],0,sptr.length*(ubyte*).sizeof-VSID*VSID*4);
	vbiti = ((cast(int)v-cast(int)vbuf)>>2); //# vbuf longs/vbit bits allocated
	clearbuf(cast(void *)vbit,vbiti>>5,-1);
	clearbuf(cast(void *)&vbit[vbiti>>5],(VOXSIZ>>7)-(vbiti>>5),0);
	vbit[vbiti>>5] = (1<<vbiti)-1;

		//Blow out sphere and stick you inside map
	vx5.colfunc = &jitcolfunc; vx5.curcol = 0x80704030;

	lp0.x = cast(int)(VSID*0.5-90); lp0.y = cast(int)(VSID*0.5-90); lp0.z = cast(int)(MAXZDIM*0.5-45);
	lp1.x = cast(int)(VSID*0.5+90); lp1.y = cast(int)(VSID*0.5+90); lp1.z = cast(int)(MAXZDIM*0.5+45);
	setrect(&lp0,&lp1,-1);
	//lp.x = VSID*.5; lp.y = VSID*.5; lp.z = MAXZDIM*.5; setsphere(&lp,64,-1);

	vx5.globalmass = calcglobalmass();

	ipo.x = VSID*.5; ipo.y = VSID*.5; ipo.z = MAXZDIM*.5; //ipo.z = -16;
	f = 0.0*PI/180.0;
	ist.x = cos(f); ist.y = sin(f); ist.z = 0;
	ihe.x = 0; ihe.y = 0; ihe.z = 1;
	ifo.x = sin(f); ifo.y = -cos(f); ifo.z = 0;

	gmipnum = 1; vx5.flstnum = 0;
	updatebbox(0,0,0,VSID,VSID,MAXZDIM,0);
}

int loaddta (const char *filename, dpoint3d *ipo, dpoint3d *ist, dpoint3d *ihe, dpoint3d *ifo)
{
	int i, j, p, leng, minz = 255, maxz = 0;
	int[5] h;
	int[256] longpal;

	ubyte dat;
	ubyte* dtahei, dtacol, v;
	char[MAX_PATH] dafilename;

	float f;
	FILE *fp;

	if (!vbuf) { vbuf = cast(int *)malloc((VOXSIZ>>2)<<2); if (!vbuf) evilquit("vbuf malloc failed"); }
	if (!vbit) { vbit = cast(int *)malloc((VOXSIZ>>7)<<2); if (!vbit) evilquit("vbuf malloc failed"); }

	if (VSID != 1024) return(0);
	v = cast(ubyte *)(&vbuf[1]); //1st dword for voxalloc compare logic optimization

	strcpy(&dafilename[0],filename);

	dtahei = cast(ubyte *)(&vbuf[(VOXSIZ-2097152)>>2]);
	dtacol = cast(ubyte *)(&vbuf[(VOXSIZ-1048576)>>2]);

	dafilename[0] = 'd';
	if (!kzopen(dafilename)) return(0);
	kzseek(128,SEEK_SET); p = 0;
	while (p < 1024*1024)
	{
		dat = kzgetc();
		if (dat >= 192) { leng = dat-192; dat = kzgetc(); }
					  else { leng = 1; }
		dat = 255-dat;
		if (dat < minz) minz = dat;
		if (dat > maxz) maxz = dat;
		while (leng-- > 0) dtahei[p++] = dat;
	}
	kzclose();

	dafilename[0] = 'c';
	if (!kzopen(dafilename)) return(0);
	kzseek(128,SEEK_SET);
	p = 0;
	while (p < 1024*1024)
	{
		dat = kzgetc();
		if (dat >= 192) { leng = dat-192; dat = kzgetc(); }
					  else { leng = 1; }
		while (leng-- > 0) dtacol[p++] = dat;
	}

	dat = kzgetc();
	if (dat == 0xc)
		for(i=0;i<256;i++)
		{
			longpal[i] = kzgetc();
			longpal[i] = (longpal[i]<<8)+kzgetc();
			longpal[i] = (longpal[i]<<8)+kzgetc() + 0x80000000;
		}

	kzclose();

		//Fill board data
	minz = lbound(128-((minz+maxz)>>1),-minz,255-maxz);
	for(p=0;p<1024*1024;p++)
	{
		h[0] = cast(int)dtahei[p];
		h[1] = cast(int)dtahei[((p-1)&0x3ff)+((p     )&0xffc00)];
		h[2] = cast(int)dtahei[((p+1)&0x3ff)+((p     )&0xffc00)];
		h[3] = cast(int)dtahei[((p  )&0x3ff)+((p-1024)&0xffc00)];
		h[4] = cast(int)dtahei[((p  )&0x3ff)+((p+1024)&0xffc00)];

		j = 1;
		for(i=4;i>0;i--) if (h[i]-h[0] > j) j = h[i]-h[0];

		sptr[p] = v;
		v[0] = 0;
		v[1] = cast(ubyte)(dtahei[p]+minz);
		v[2] = cast(ubyte)(dtahei[p]+minz+j-1);
		v[3] = 0; //dummy (z top)
		v += 4;
		for(;j;j--) { *cast(int *)v = colorjit(longpal[dtacol[p]],0x70707); v += 4; }
	}

	memset(&sptr[VSID*VSID],0,sptr.length*(ubyte*).sizeof-VSID*VSID*4);
	vbiti = ((cast(int)v-cast(int)vbuf)>>2); //# vbuf longs/vbit bits allocated
	clearbuf(cast(void *)vbit,vbiti>>5,-1);
	clearbuf(cast(void *)&vbit[vbiti>>5],(VOXSIZ>>7)-(vbiti>>5),0);
	vbit[vbiti>>5] = (1<<vbiti)-1;

	vx5.globalmass = calcglobalmass();

	ipo.x = VSID*.5; ipo.y = VSID*.5; ipo.z = 128;
	f = 0.0*PI/180.0;
	ist.x = cos(f); ist.y = sin(f); ist.z = 0;
	ihe.x = 0; ihe.y = 0; ihe.z = 1;
	ifo.x = sin(f); ifo.y = -cos(f); ifo.z = 0;

	gmipnum = 1; vx5.flstnum = 0;
	updatebbox(0,0,0,VSID,VSID,MAXZDIM,0);
	return(1);
}

int loadpng (const char *filename, dpoint3d *ipo, dpoint3d *ist, dpoint3d *ihe, dpoint3d *ifo)
{
	uint* pngdat; 
	uint[5] dat;
	int i, j, k, l, p, leng, minz = 255, maxz = 0;
	ubyte* v, buf;
	float f;
	FILE *fp;

	if (!vbuf) { vbuf = cast(int *)malloc((VOXSIZ>>2)<<2); if (!vbuf) evilquit("vbuf malloc failed"); }
	if (!vbit) { vbit = cast(int *)malloc((VOXSIZ>>7)<<2); if (!vbit) evilquit("vbuf malloc failed"); }

	if (VSID != 1024) return(0);
	v = cast(ubyte *)(&vbuf[1]); //1st dword for voxalloc compare logic optimization

	if (!kzopen(filename)) return(0);
	leng = kzfilelength();
	buf = cast(ubyte *)malloc(leng); if (!buf) { kzclose(); return(0); }
	kzread(buf,leng);
	kzclose();

	kpgetdim(buf,leng,cast(int *)&i,cast(int *)&j); if ((i != VSID) && (j != VSID)) { free(buf); return(0); }
	pngdat = cast(uint *)(&vbuf[(VOXSIZ-VSID*VSID*4)>>2]);
	if (kprender(buf,leng,cast(int)pngdat,VSID<<2,VSID,VSID,0,0) < 0) return(0);
	free(buf);

	for(i=0;i<VSID*VSID;i++)
	{
		if ((pngdat[i]>>24) < minz) minz = (pngdat[i]>>24);
		if ((pngdat[i]>>24) > maxz) maxz = (pngdat[i]>>24);
	}

		//Fill board data
	minz = lbound(128-((minz+maxz)>>1),-minz,255-maxz);
	for(p=0;p<VSID*VSID;p++)
	{
		dat[0] = pngdat[p];
		dat[1] = pngdat[((p-1)&(VSID-1))+((p     )&((VSID-1)*VSID))];
		dat[2] = pngdat[((p+1)&(VSID-1))+((p     )&((VSID-1)*VSID))];
		dat[3] = pngdat[((p  )&(VSID-1))+((p-VSID)&((VSID-1)*VSID))];
		dat[4] = pngdat[((p  )&(VSID-1))+((p+VSID)&((VSID-1)*VSID))];

		j = 1; l = dat[0];
		for(i=4;i>0;i--)
			if ((cast(int)((dat[i]>>24)-(dat[0]>>24))) > j)
				{ j = (dat[i]>>24)-(dat[0]>>24); l = dat[i]; }

		sptr[p] = v;
		v[0] = 0;
		v[1] = (pngdat[p]>>24)+minz;
		v[2] = (pngdat[p]>>24)+minz+j-1;
		v[3] = 0; //dummy (z top)
		v += 4;
		k = (pngdat[p]&0xffffff)|0x80000000;
		if (j == 2)
		{
			l = (((  l     &255)-( k     &255))>>1)      +
				 (((((l>> 8)&255)-((k>> 8)&255))>>1)<< 8) +
				 (((((l>>16)&255)-((k>>16)&255))>>1)<<16);
		}
		else if (j > 2)
		{
			l = (((  l     &255)-( k     &255))/j)      +
				 (((((l>> 8)&255)-((k>> 8)&255))/j)<< 8) +
				 (((((l>>16)&255)-((k>>16)&255))/j)<<16);
		}
		*cast(int *)v = k; v += 4; j--;
		while (j) { k += l; *cast(int *)v = colorjit(k,0x30303); v += 4; j--; }
	}

	memset(&sptr[VSID*VSID],0,sizeof(sptr)-VSID*VSID*4);
	vbiti = ((cast(int)v-cast(int)vbuf)>>2); //# vbuf longs/vbit bits allocated
	clearbuf(cast(void *)vbit,vbiti>>5,-1);
	clearbuf(cast(void *)&vbit[vbiti>>5],(VOXSIZ>>7)-(vbiti>>5),0);
	vbit[vbiti>>5] = (1<<vbiti)-1;

	vx5.globalmass = calcglobalmass();

	ipo.x = VSID*.5; ipo.y = VSID*.5; ipo.z = 128;
	f = 0.0*PI/180.0;
	ist.x = cos(f); ist.y = sin(f); ist.z = 0;
	ihe.x = 0; ihe.y = 0; ihe.z = 1;
	ifo.x = sin(f); ifo.y = -cos(f); ifo.z = 0;

	gmipnum = 1; vx5.flstnum = 0;
	updatebbox(0,0,0,VSID,VSID,MAXZDIM,0);
	return(1);
}

//Quake3 .BSP loading code begins --------------------------------------------
struct vlinerectyp 
{ 
	int c, i; 
	float z, z1; 
}

static point3d q3pln[5250];
static float q3pld[5250];
static float q3vz[256];
static int q3nod[4850][3];
static int q3lf[4850];

int vlinebsp (float x, float y, float z0, float z1, float *dvz)
{
	vlinerectyp vlrec[64];
	float z, t;
	int i, j, vcnt, vlcnt;
	char vt[256];

	vcnt = 1; i = 0; vlcnt = 0; vt[0] = 17;
	while (true)
	{
		if (i < 0)
		{
			if (vt[vcnt-1] != (q3lf[~i]&255))
				{ dvz[vcnt] = z0; vt[vcnt] = (q3lf[~i]&255); vcnt++; }
		}
		else
		{
			j = q3nod[i][0]; z = q3pld[j] - q3pln[j].x*x - q3pln[j].y*y;
			t = q3pln[j].z*z0-z;
			if ((t < 0) == (q3pln[j].z*z1 < z))
				{ vlrec[vlcnt].c = 0; i = q3nod[i][(t<0)+1]; }
			else
			{
				z /= q3pln[j].z; j = (q3pln[j].z<0)+1;
				vlrec[vlcnt].c = 1; vlrec[vlcnt].i = q3nod[i][j];
				vlrec[vlcnt].z = z; vlrec[vlcnt].z1 = z1;
				i = q3nod[i][3-j]; z1 = z;
			}
			vlcnt++; continue;
		}
		do { vlcnt--; if (vlcnt < 0) return(vcnt); } while (!vlrec[vlcnt].c);
		vlrec[vlcnt].c = 0; i = vlrec[vlcnt].i;
		z0 = vlrec[vlcnt].z; z1 = vlrec[vlcnt].z1;
		vlcnt++;
	}
	return(0);
}

	//Stupidly useless declarations:
//void delslab(int *b2, int y0, int y1);
//int *scum2(int x, int y);
//void scum2finish();

void loadbsp (const char* filnam, dpoint3d* ipo, dpoint3d* ist, dpoint3d* ihe, dpoint3d* ifo)
{
	FILE* fp;
	dpoint3d dp;
	float f, xof, yof, zof, sc, rsc;
	int numplanes, numnodes, numleafs;
	int[17] fpos, flng;
	int i, x, y, z, z0, z1, vcnt, minx, miny, minz, maxx, maxy, maxz;
	int* lptr;
	ubyte* v;

	if (!vbuf) { vbuf = cast(int *)malloc((VOXSIZ>>2)<<2); if (!vbuf) evilquit("vbuf malloc failed"); }
	if (!vbit) { vbit = cast(int *)malloc((VOXSIZ>>7)<<2); if (!vbit) evilquit("vbuf malloc failed"); }

		//Completely re-compile vbuf
	v = cast(ubyte *)(&vbuf[1]); //1st dword for voxalloc compare logic optimization
	for(x=0;x<VSID;x++)
		for(y=0;y<VSID;y++)
		{
			sptr[y*VSID+x] = v; v[0] = 0; v[1] = 0; v[2] = 0; v[3] = 0; v += 4;
			(*cast(int *)v) = ((x^y)&15)*0x10101+0x807c7c7c; v += 4;
		}

	memset(&sptr[VSID*VSID],0,sizeof(sptr)-VSID*VSID*4);
	vbiti = ((cast(int)v-cast(int)vbuf)>>2); //# vbuf longs/vbit bits allocated
	clearbuf(cast(void *)vbit,vbiti>>5,-1);
	clearbuf(cast(void *)&vbit[vbiti>>5],(VOXSIZ>>7)-(vbiti>>5),0);
	vbit[vbiti>>5] = (1<<vbiti)-1;

	if (!kzopen(filnam)) return;
	kzread(&i,4); if (i != 0x50534249) { kzclose(); return; }
	kzread(&i,4); if (i != 0x2e) { kzclose(); return; }
	for(i=0;i<17;i++) { kzread(&fpos[i],4); kzread(&flng[i],4); }
	kzseek(fpos[2],SEEK_SET); numplanes = flng[2]/16;
	for(i=0;i<numplanes;i++) { kzread(&q3pln[i].x,12); kzread(&q3pld[i],4); }
	kzseek(fpos[3],SEEK_SET); numnodes = flng[3]/36;
	minx = 0x7fffffff; miny = 0x7fffffff; minz = 0x7fffffff;
	maxx = 0x80000000; maxy = 0x80000000; maxz = 0x80000000;
	for(i=0;i<numnodes;i++)
	{
		kzread(&q3nod[i][0],12);
		kzread(&x,4); if (x < minx) minx = x;
		kzread(&x,4); if (x < miny) miny = x;
		kzread(&x,4); if (x < minz) minz = x;
		kzread(&x,4); if (x > maxx) maxx = x;
		kzread(&x,4); if (x > maxy) maxy = x;
		kzread(&x,4); if (x > maxz) maxz = x;
	}
	kzseek(fpos[4]+4,SEEK_SET); numleafs = flng[4]/48;
	for(i=0;i<numleafs;i++) { kzread(&q3lf[i],4); kzseek(44,SEEK_CUR); }
	kzclose();

	sc = cast(float)(VSID-2)/cast(float)(maxx-minx);
	rsc = cast(float)(VSID-2)/cast(float)(maxy-miny); if (rsc < sc) sc = rsc;
	rsc = cast(float)(MAXZDIM-2)/cast(float)(maxz-minz); if (rsc < sc) sc = rsc;
	//i = *(long *)sc; i &= 0xff800000; sc = *(float *)i;
	xof = (-cast(float)(minx+maxx)*sc + VSID   )*.5;
	yof = (+cast(float)(miny+maxy)*sc + VSID   )*.5;
	zof = (+cast(float)(minz+maxz)*sc + MAXZDIM)*.5;

	rsc = 1.0 / sc;
	vx5.colfunc = curcolfunc; //0<x0<x1<VSID, 0<y0<y1<VSID, 0<z0<z1<256,
	for(y=0;y<VSID;y++)
		for(x=0;x<VSID;x++)
		{
			lptr = scum2(x,y);

				//voxx = q3x*+sc + xof;
				//voxy = q3y*-sc + yof;
				//voxz = q3z*-sc + zof;
			vcnt = vlinebsp((cast(float)x-xof)*rsc,(cast(float)y-yof)*-rsc,-65536.0,65536.0,q3vz);
			for(i=vcnt-2;i>0;i-=2)
			{
				ftol(-q3vz[i+1]*sc+zof,&z0); if (z0 < 0) z0 = 0;
				ftol(-q3vz[i  ]*sc+zof,&z1); if (z1 > MAXZDIM) z1 = MAXZDIM;
				delslab(lptr,z0,z1);
			}
		}
	scum2finish();

	vx5.globalmass = calcglobalmass();

		//Find a spot that isn't too close to a wall
	sc = -1; ipo.x = VSID*.5; ipo.y = VSID*.5; ipo.z = -16;
	for(i=4096;i>=0;i--)
	{
		x = (rand()%VSID); y = (rand()%VSID); z = (rand()%MAXZDIM);
		if (!isvoxelsolid(x,y,z))
		{
			rsc = findmaxcr(cast(double)x+.5,cast(double)y+.5,cast(double)z+.5,5.0);
			if (rsc <= sc) continue;
			ipo.x = cast(double)x+.5; ipo.y = cast(double)x+.5; ipo.z = cast(double)x+.5;
			sc = rsc; if (sc >= 5.0) break;
		}
	}
	f = 0.0*PI/180.0;
	ist.x = cos(f); ist.y = sin(f); ist.z = 0;
	ihe.x = 0; ihe.y = 0; ihe.z = 1;
	ifo.x = sin(f); ifo.y = -cos(f); ifo.z = 0;

	gmipnum = 1; vx5.flstnum = 0;
	updatebbox(0,0,0,VSID,VSID,MAXZDIM,0);
}

//Quake3 .BSP loading code ends ----------------------------------------------

int loadvxl (const char* lodfilnam, dpoint3d* ipo, dpoint3d* ist, dpoint3d* ihe, dpoint3d* ifo)
{
	FILE* fil;
	int i, j, fsiz;
	ubyte* v, v2;

	if (!vbuf) { vbuf = cast(int *)malloc((VOXSIZ>>2)<<2); if (!vbuf) evilquit("vbuf malloc failed"); }
	if (!vbit) { vbit = cast(int *)malloc((VOXSIZ>>7)<<2); if (!vbit) evilquit("vbuf malloc failed"); }

	if (!kzopen(lodfilnam)) return(0);
	fsiz = kzfilelength();

	kzread(&i,4); if (i != 0x09072000) return(0);
	kzread(&i,4); if (i != VSID) return(0);
	kzread(&i,4); if (i != VSID) return(0);
	kzread(ipo,24);
	kzread(ist,24);
	kzread(ihe,24);
	kzread(ifo,24);

	v = cast(ubyte *)(&vbuf[1]); //1st dword for voxalloc compare logic optimization
	kzread(cast(void *)v,fsiz-kztell());

	for(i=0;i<VSID*VSID;i++)
	{
		sptr[i] = v;
		while (v[0]) v += ((cast(int)v[0])<<2);
		v += (((cast(int)v[2])-(cast(int)v[1])+2)<<2);
	}
	kzclose();

	memset(&sptr[VSID*VSID],0,sizeof(sptr)-VSID*VSID*4);
	vbiti = ((cast(int)v-cast(int)vbuf)>>2); //# vbuf longs/vbit bits allocated
	clearbuf(cast(void *)vbit,vbiti>>5,-1);
	clearbuf(cast(void *)&vbit[vbiti>>5],(VOXSIZ>>7)-(vbiti>>5),0);
	vbit[vbiti>>5] = (1<<vbiti)-1;

	vx5.globalmass = calcglobalmass();
	backedup = -1;

	gmipnum = 1; vx5.flstnum = 0;
	updatebbox(0,0,0,VSID,VSID,MAXZDIM,0);
	return(1);
}

int savevxl (const char* savfilnam, dpoint3d* ipo, dpoint3d* ist, dpoint3d* ihe, dpoint3d* ifo)
{
	FILE *fil;
	int i;

	if (!(fil = fopen(savfilnam,"wb"))) return(0);
	i = 0x09072000; fwrite(&i,4,1,fil);  //Version
	i = VSID; fwrite(&i,4,1,fil);
	i = VSID; fwrite(&i,4,1,fil);
	fwrite(ipo,24,1,fil);
	fwrite(ist,24,1,fil);
	fwrite(ihe,24,1,fil);
	fwrite(ifo,24,1,fil);
	for(i=0;i<VSID*VSID;i++) fwrite(cast(void *)sptr[i],slng(sptr[i]),1,fil);
	fclose(fil);
	return(1);
}

int loadsky (const char* skyfilnam)
{
	int x, y, xoff, yoff;
	float ang, f;

	if (skypic) { free(cast(int *)skypic); skypic = skyoff = 0; }
	xoff = yoff = 0;

	if (!stricmp(skyfilnam,"BLACK")) return(0);
	if (!stricmp(skyfilnam,"BLUE")) goto loadbluesky;

	kpzload(skyfilnam,cast(int *)&skypic,cast(int *)&skybpl,cast(int *)&skyxsiz,cast(int *)&skyysiz);
	if (!skypic)
	{
		int r, g, b;
		int* p;
loadbluesky:;
			//Load default sky
		skyxsiz = 512; skyysiz = 1; skybpl = skyxsiz*4;
		if (!(skypic = cast(int)malloc(skyysiz*skybpl))) return(-1);

		p = cast(int *)skypic; y = skyxsiz*skyxsiz;
		for(x=0;x<=(skyxsiz>>1);x++)
		{
			p[x] = ((((x*1081 - skyxsiz*252)*x)/y + 35)<<16)+
					 ((((x* 950 - skyxsiz*198)*x)/y + 53)<<8)+
					  (((x* 439 - skyxsiz* 21)*x)/y + 98);
		}
		p[skyxsiz-1] = 0x50903c;
		r = ((p[skyxsiz>>1]>>16)&255);
		g = ((p[skyxsiz>>1]>>8)&255);
		b = ((p[skyxsiz>>1])&255);
		for(x=(skyxsiz>>1)+1;x<skyxsiz;x++)
		{
			p[x] = ((((0x50-r)*(x-(skyxsiz>>1)))/(skyxsiz-1-(skyxsiz>>1))+r)<<16)+
					 ((((0x90-g)*(x-(skyxsiz>>1)))/(skyxsiz-1-(skyxsiz>>1))+g)<<8)+
					 ((((0x3c-b)*(x-(skyxsiz>>1)))/(skyxsiz-1-(skyxsiz>>1))+b));
		}
		y = skyxsiz*skyysiz;
		for(x=skyxsiz;x<y;x++) p[x] = p[x-skyxsiz];
	}

		//Initialize look-up table for longitudes
	if (skylng) free(cast(void *)skylng);
	if (!(skylng = cast(point2d *)malloc(skyysiz*8))) return(-1);
	f = PI*2.0 / (cast(float)skyysiz);
	for(y=skyysiz-1;y>=0;y--)
		fcossin(cast(float)y*f+PI,&skylng[y].x,&skylng[y].y);
	skylngmul = cast(float)skyysiz/(PI*2);
		//This makes those while loops in gline() not lockup when skyysiz==1
	if (skyysiz == 1) { skylng[0].x = 0; skylng[0].y = 0; }

		//Initialize look-up table for latitudes
	if (skylat) free(cast(void *)skylat);
	if (!(skylat = cast(int *)malloc(skyxsiz*4))) return(-1);
	f = PI*.5 / (cast(float)skyxsiz);
	for(x=skyxsiz-1;x;x--)
	{
		ang = cast(float)((x<<1)-skyxsiz)*f;
		ftol(cos(ang)*32767.0,&xoff);
		ftol(sin(ang)*32767.0,&yoff);
		skylat[x] = (xoff<<16)+((-yoff)&65535);
	}
	skylat[0] = 0; //Hack to make sure assembly index never goes < 0
	skyxsiz--; //Hack for assembly code

	return(0);
}

void orthonormalize (point3d *v0, point3d *v1, point3d *v2)
{
	float t;

	t = 1.0 / sqrt((v0.x)*(v0.x) + (v0.y)*(v0.y) + (v0.z)*(v0.z));
	(v0.x) *= t; (v0.y) *= t; (v0.z) *= t;
	t = (v1.x)*(v0.x) + (v1.y)*(v0.y) + (v1.z)*(v0.z);
	(v1.x) -= t*(v0.x); (v1.y) -= t*(v0.y); (v1.z) -= t*(v0.z);
	t = 1.0 / sqrt((v1.x)*(v1.x) + (v1.y)*(v1.y) + (v1.z)*(v1.z));
	(v1.x) *= t; (v1.y) *= t; (v1.z) *= t;
	(v2.x) = (v0.y)*(v1.z) - (v0.z)*(v1.y);
	(v2.y) = (v0.z)*(v1.x) - (v0.x)*(v1.z);
	(v2.z) = (v0.x)*(v1.y) - (v0.y)*(v1.x);
}

void dorthonormalize (dpoint3d *v0, dpoint3d *v1, dpoint3d *v2)
{
	double t;

	t = 1.0 / sqrt((v0.x)*(v0.x) + (v0.y)*(v0.y) + (v0.z)*(v0.z));
	(v0.x) *= t; (v0.y) *= t; (v0.z) *= t;
	t = (v1.x)*(v0.x) + (v1.y)*(v0.y) + (v1.z)*(v0.z);
	(v1.x) -= t*(v0.x); (v1.y) -= t*(v0.y); (v1.z) -= t*(v0.z);
	t = 1.0 / sqrt((v1.x)*(v1.x) + (v1.y)*(v1.y) + (v1.z)*(v1.z));
	(v1.x) *= t; (v1.y) *= t; (v1.z) *= t;
	(v2.x) = (v0.y)*(v1.z) - (v0.z)*(v1.y);
	(v2.y) = (v0.z)*(v1.x) - (v0.x)*(v1.z);
	(v2.z) = (v0.x)*(v1.y) - (v0.y)*(v1.x);
}

void orthorotate (float ox, float oy, float oz, point3d *ist, point3d *ihe, point3d *ifo)
{
	float f, t, dx, dy, dz;
	float rr[9];

	fcossin(ox,&ox,&dx);
	fcossin(oy,&oy,&dy);
	fcossin(oz,&oz,&dz);
	f = ox*oz; t = dx*dz; rr[0] =  t*dy + f; rr[7] = -f*dy - t;
	f = ox*dz; t = dx*oz; rr[1] = -f*dy + t; rr[6] =  t*dy - f;
	rr[2] = dz*oy; rr[3] = -dx*oy; rr[4] = ox*oy; rr[8] = oz*oy; rr[5] = dy;
	ox = ist.x; oy = ihe.x; oz = ifo.x;
	ist.x = ox*rr[0] + oy*rr[3] + oz*rr[6];
	ihe.x = ox*rr[1] + oy*rr[4] + oz*rr[7];
	ifo.x = ox*rr[2] + oy*rr[5] + oz*rr[8];
	ox = ist.y; oy = ihe.y; oz = ifo.y;
	ist.y = ox*rr[0] + oy*rr[3] + oz*rr[6];
	ihe.y = ox*rr[1] + oy*rr[4] + oz*rr[7];
	ifo.y = ox*rr[2] + oy*rr[5] + oz*rr[8];
	ox = ist.z; oy = ihe.z; oz = ifo.z;
	ist.z = ox*rr[0] + oy*rr[3] + oz*rr[6];
	ihe.z = ox*rr[1] + oy*rr[4] + oz*rr[7];
	ifo.z = ox*rr[2] + oy*rr[5] + oz*rr[8];
	//orthonormalize(ist,ihe,ifo);
}

void dorthorotate (double ox, double oy, double oz, dpoint3d *ist, dpoint3d *ihe, dpoint3d *ifo)
{
	double f, t, dx, dy, dz;
	double rr[9];

	dcossin(ox,&ox,&dx);
	dcossin(oy,&oy,&dy);
	dcossin(oz,&oz,&dz);
	f = ox*oz; t = dx*dz; rr[0] =  t*dy + f; rr[7] = -f*dy - t;
	f = ox*dz; t = dx*oz; rr[1] = -f*dy + t; rr[6] =  t*dy - f;
	rr[2] = dz*oy; rr[3] = -dx*oy; rr[4] = ox*oy; rr[8] = oz*oy; rr[5] = dy;
	ox = ist.x; oy = ihe.x; oz = ifo.x;
	ist.x = ox*rr[0] + oy*rr[3] + oz*rr[6];
	ihe.x = ox*rr[1] + oy*rr[4] + oz*rr[7];
	ifo.x = ox*rr[2] + oy*rr[5] + oz*rr[8];
	ox = ist.y; oy = ihe.y; oz = ifo.y;
	ist.y = ox*rr[0] + oy*rr[3] + oz*rr[6];
	ihe.y = ox*rr[1] + oy*rr[4] + oz*rr[7];
	ifo.y = ox*rr[2] + oy*rr[5] + oz*rr[8];
	ox = ist.z; oy = ihe.z; oz = ifo.z;
	ist.z = ox*rr[0] + oy*rr[3] + oz*rr[6];
	ihe.z = ox*rr[1] + oy*rr[4] + oz*rr[7];
	ifo.z = ox*rr[2] + oy*rr[5] + oz*rr[8];
	//dorthonormalize(ist,ihe,ifo);
}

void axisrotate (point3d *p, point3d *axis, float w)
{
	point3d ax;
	float t, c, s, ox, oy, oz;
	float k[9];

	fcossin(w,&c,&s);
	t = axis.x*axis.x + axis.y*axis.y + axis.z*axis.z; if (t == 0) return;
	t = 1.0 / sqrt(t); ax.x = axis.x*t; ax.y = axis.y*t; ax.z = axis.z*t;

	t = 1.0-c;
	k[0] = ax.x*t; k[7] = ax.x*s; oz = ax.y*k[0];
	k[4] = ax.y*t; k[2] = ax.y*s; oy = ax.z*k[0];
	k[8] = ax.z*t; k[3] = ax.z*s; ox = ax.z*k[4];
	k[0] = ax.x*k[0] + c; k[5] = ox - k[7]; k[7] += ox;
	k[4] = ax.y*k[4] + c; k[6] = oy - k[2]; k[2] += oy;
	k[8] = ax.z*k[8] + c; k[1] = oz - k[3]; k[3] += oz;

	ox = p.x; oy = p.y; oz = p.z;
	p.x = ox*k[0] + oy*k[1] + oz*k[2];
	p.y = ox*k[3] + oy*k[4] + oz*k[5];
	p.z = ox*k[6] + oy*k[7] + oz*k[8];
}

void slerp (point3d *istr, point3d *ihei, point3d *ifor,
				point3d *istr2, point3d *ihei2, point3d *ifor2,
				point3d *ist, point3d *ihe, point3d *ifo, float rat)
{
	point3d ax;
	float c, s, t, ox, oy, oz;
	float k[9];

	ist.x = istr.x; ist.y = istr.y; ist.z = istr.z;
	ihe.x = ihei.x; ihe.y = ihei.y; ihe.z = ihei.z;
	ifo.x = ifor.x; ifo.y = ifor.y; ifo.z = ifor.z;

	ax.x = istr.y*istr2.z - istr.z*istr2.y + ihei.y*ihei2.z - ihei.z*ihei2.y + ifor.y*ifor2.z - ifor.z*ifor2.y;
	ax.y = istr.z*istr2.x - istr.x*istr2.z + ihei.z*ihei2.x - ihei.x*ihei2.z + ifor.z*ifor2.x - ifor.x*ifor2.z;
	ax.z = istr.x*istr2.y - istr.y*istr2.x + ihei.x*ihei2.y - ihei.y*ihei2.x + ifor.x*ifor2.y - ifor.y*ifor2.x;
	t = ax.x*ax.x + ax.y*ax.y + ax.z*ax.z; if (t == 0) return;

		//Based on the vector suck-out method (see ROTATE2.BAS)
	ox = istr.x*ax.x + istr.y*ax.y + istr.z*ax.z;
	oy = ihei.x*ax.x + ihei.y*ax.y + ihei.z*ax.z;
	if (fabs(ox) < fabs(oy))
		{ c = istr.x*istr2.x + istr.y*istr2.y + istr.z*istr2.z; s = ox*ox; }
	else
		{ c = ihei.x*ihei2.x + ihei.y*ihei2.y + ihei.z*ihei2.z; s = oy*oy; }
	if (t == s) return;
	c = (c*t - s) / (t-s);
	if (c < -1) c = -1;
	if (c > 1) c = 1;
	fcossin(acos(c)*rat,&c,&s);

	t = 1.0 / sqrt(t); ax.x *= t; ax.y *= t; ax.z *= t;

	t = 1.0f-c;
	k[0] = ax.x*t; k[7] = ax.x*s; oz = ax.y*k[0];
	k[4] = ax.y*t; k[2] = ax.y*s; oy = ax.z*k[0];
	k[8] = ax.z*t; k[3] = ax.z*s; ox = ax.z*k[4];
	k[0] = ax.x*k[0] + c; k[5] = ox - k[7]; k[7] += ox;
	k[4] = ax.y*k[4] + c; k[6] = oy - k[2]; k[2] += oy;
	k[8] = ax.z*k[8] + c; k[1] = oz - k[3]; k[3] += oz;

	ox = ist.x; oy = ist.y; oz = ist.z;
	ist.x = ox*k[0] + oy*k[1] + oz*k[2];
	ist.y = ox*k[3] + oy*k[4] + oz*k[5];
	ist.z = ox*k[6] + oy*k[7] + oz*k[8];

	ox = ihe.x; oy = ihe.y; oz = ihe.z;
	ihe.x = ox*k[0] + oy*k[1] + oz*k[2];
	ihe.y = ox*k[3] + oy*k[4] + oz*k[5];
	ihe.z = ox*k[6] + oy*k[7] + oz*k[8];

	ox = ifo.x; oy = ifo.y; oz = ifo.z;
	ifo.x = ox*k[0] + oy*k[1] + oz*k[2];
	ifo.y = ox*k[3] + oy*k[4] + oz*k[5];
	ifo.z = ox*k[6] + oy*k[7] + oz*k[8];
}

void expandrle (int x, int y, int *uind)
{
	int i;
	ubyte *v;

	if ((x|y)&(~(VSID-1))) { uind[0] = 0; uind[1] = MAXZDIM; return; }

	v = sptr[y*VSID+x]; uind[0] = v[1]; i = 2;
	while (v[0])
	{
		v += v[0]*4; if (v[3] >= v[1]) continue;
		uind[i-1] = v[3]; uind[i] = v[1]; i += 2;
	}
	uind[i-1] = MAXZDIM;
}

/**
*	Inputs:  n0[<=MAXZDIM]: rle buffer of column to compress
*	         n1-4[<=MAXZDIM]: neighboring rle buffers
*	         top,bot,top,bot,... (ends when bot == MAXZDIM)
*	         px,py: takes color from original column (before modification)
*	            If originally unexposed, calls vx5.colfunc(.)
*	Outputs: cbuf[MAXCSIZ]: compressed output buffer
*	Returns: n: length of compressed buffer (in bytes)
*/
int compilerle (int *n0, int *n1, int *n2, int *n3, int *n4, ubyte *cbuf, int px, int py)
{
	int i, ia, ze, zend, onext, dacnt, n;
	int* ic;
	lpoint3d p;
	ubyte *v;

	p.x = px; p.y = py;

		//Generate pointers to color slabs in this format:
		//   0:z0,  1:z1,  2:(pointer to z0's color)-z0
	v = sptr[py*VSID+px]; ic = tbuf2;
	while (true)
	{
		ia = v[1]; p.z = v[2];
		ic[0] = ia; ic[1] = p.z+1; ic[2] = (cast(int)v)-(ia<<2)+4; ic += 3;
		i = v[0]; if (!i) break;
		v += i*4; ze = v[3];
		ic[0] = ze+p.z-ia-i+2; ic[1] = ze; ic[2] = (cast(int)v)-(ze<<2); ic += 3;
	}
	ic[0] = MAXZDIM; ic[1] = MAXZDIM;

	p.z = n0[0]; cbuf[1] = n0[0];
	ze = n0[1]; cbuf[2] = ze-1;
	cbuf[3] = 0;
	i = onext = 0; ic = tbuf2; ia = 15; n = 4;
	if (ze != MAXZDIM) zend = ze-1; else zend = -1;
	while (true)
	{
		dacnt = 0;
		while (true)
		{
			do
			{
				while (p.z >= ic[1]) ic += 3;
				if (p.z >= ic[0]) *cast(int *)&cbuf[n] = *cast(int *)(ic[2]+(p.z<<2));
								 else *cast(int *)&cbuf[n] = vx5.colfunc(&p);
				n += 4; p.z++; if (p.z >= ze) goto rlendit2;
				while (p.z >= n1[0]) { n1++; ia ^= 1; }
				while (p.z >= n2[0]) { n2++; ia ^= 2; }
				while (p.z >= n3[0]) { n3++; ia ^= 4; }
				while (p.z >= n4[0]) { n4++; ia ^= 8; }
			} while ((ia) || (p.z == zend));

			if (!dacnt) { cbuf[onext+2] = p.z-1; dacnt = 1; }
			else
			{
				cbuf[onext] = ((n-onext)>>2); onext = n;
				cbuf[n+1] = p.z; cbuf[n+2] = p.z-1; cbuf[n+3] = p.z; n += 4;
			}

			if ((n1[0] < n2[0]) && (n1[0] < n3[0]) && (n1[0] < n4[0]))
				{ if (n1[0] >= ze) { p.z = ze-1; } else { p.z = *n1++; ia ^= 1; } }
			else if ((n2[0] < n3[0]) && (n2[0] < n4[0]))
				{ if (n2[0] >= ze) { p.z = ze-1; } else { p.z = *n2++; ia ^= 2; } }
			else if (n3[0] < n4[0])
				{ if (n3[0] >= ze) { p.z = ze-1; } else { p.z = *n3++; ia ^= 4; } }
			else
				{ if (n4[0] >= ze) { p.z = ze-1; } else { p.z = *n4++; ia ^= 8; } }

			if (p.z == MAXZDIM-1) goto rlenditall;
		}
rlendit2:;
		if (ze >= MAXZDIM) break;

		i += 2;
		cbuf[onext] = ((n-onext)>>2); onext = n;
		p.z = n0[i]; cbuf[n+1] = n0[i]; cbuf[n+3] = ze;
		ze = n0[i+1]; cbuf[n+2] = ze-1;
		n += 4;
	}
rlenditall:;
	cbuf[onext] = 0;
	return(n);
}

	//Delete everything on b2() in y0<=y<y1
void delslab (int *b2, int y0, int y1)
{
	int i, j, z;

	if (y1 >= MAXZDIM) y1 = MAXZDIM-1;
	if ((y0 >= y1) || (!b2)) return;
	for(z=0;y0>=b2[z+1];z+=2) {}
	if (y0 > b2[z])
	{
		if (y1 < b2[z+1])
		{
			for(i=z;b2[i+1]<MAXZDIM;i+=2) {}
			while (i > z) { b2[i+3] = b2[i+1]; b2[i+2] = b2[i]; i -= 2; }
			b2[z+3] = b2[z+1]; b2[z+1] = y0; b2[z+2] = y1; return;
		}
		b2[z+1] = y0; z += 2;
	}
	if (y1 >= b2[z+1])
	{
		for(i=z+2;y1>=b2[i+1];i+=2) {}
		j = z-i; b2[z] = b2[i]; b2[z+1] = b2[i+1];
		while (b2[i+1] < MAXZDIM)
			{ i += 2; b2[i+j] = b2[i]; b2[i+j+1] = b2[i+1]; }
	}
	if (y1 > b2[z]) b2[z] = y1;
}

	//Insert everything on b2() in y0<=y<y1
void insslab (int *b2, int y0, int y1)
{
	int i, j, z;

	if ((y0 >= y1) || (!b2)) return;
	for(z=0;y0>b2[z+1];z+=2) {}
	if (y1 < b2[z])
	{
		for(i=z;b2[i+1]<MAXZDIM;i+=2){}
		do { b2[i+3] = b2[i+1]; b2[i+2] = b2[i]; i -= 2; } while (i >= z);
		b2[z+1] = y1; b2[z] = y0; return;
	}
	if (y0 < b2[z]) b2[z] = y0;
	if ((y1 >= b2[z+2]) && (b2[z+1] < MAXZDIM))
	{
		for(i=z+2;(y1 >= b2[i+2]) && (b2[i+1] < MAXZDIM);i+=2) {}
		j = z-i; b2[z+1] = b2[i+1];
		while (b2[i+1] < MAXZDIM)
			{ i += 2; b2[i+j] = b2[i]; b2[i+j+1] = b2[i+1]; }
	}
	if (y1 > b2[z+1]) b2[z+1] = y1;
}

//------------------------ SETCOLUMN CODE BEGINS ----------------------------

static int scx0, scx1, scox0, scox1, scoox0, scoox1;
static int scex0, scex1, sceox0, sceox1, scoy = 0x80000000; 
static int* scoym3;

void scumline ()
{
	int i, j, k, x, y, x0, x1;
	int* mptr, uptr;
	ubyte* v;

	x0 = min(scox0-1,min(scx0,scoox0)); scoox0 = scox0; scox0 = scx0;
	x1 = max(scox1+1,max(scx1,scoox1)); scoox1 = scox1; scox1 = scx1;

	uptr = &scoym3[SCPITCH]; if (uptr == &radar[SCPITCH*9]) uptr = &radar[SCPITCH*6];
	mptr = &uptr[SCPITCH];   if (mptr == &radar[SCPITCH*9]) mptr = &radar[SCPITCH*6];

	if ((x1 < sceox0) || (x0 > sceox1))
	{
		for(x=x0;x<=x1;x++) expandstack(x,scoy-2,&uptr[x*SCPITCH*3]);
	}
	else
	{
		for(x=x0;x<sceox0;x++) expandstack(x,scoy-2,&uptr[x*SCPITCH*3]);
		for(x=x1;x>sceox1;x--) expandstack(x,scoy-2,&uptr[x*SCPITCH*3]);
	}

	if ((scex1|x1) >= 0)
	{
		for(x=x1+2;x<scex0;x++) expandstack(x,scoy-1,&mptr[x*SCPITCH*3]);
		for(x=x0-2;x>scex1;x--) expandstack(x,scoy-1,&mptr[x*SCPITCH*3]);
	}
	if ((x1+1 < scex0) || (x0-1 > scex1))
	{
		for(x=x0-1;x<=x1+1;x++) expandstack(x,scoy-1,&mptr[x*SCPITCH*3]);
	}
	else
	{
		for(x=x0-1;x<scex0;x++) expandstack(x,scoy-1,&mptr[x*SCPITCH*3]);
		for(x=x1+1;x>scex1;x--) expandstack(x,scoy-1,&mptr[x*SCPITCH*3]);
	}
	sceox0 = min(x0-1,scex0);
	sceox1 = max(x1+1,scex1);

	if ((x1 < scx0) || (x0 > scx1))
	{
		for(x=x0;x<=x1;x++) expandstack(x,scoy,&scoym3[x*SCPITCH*3]);
	}
	else
	{
		for(x=x0;x<scx0;x++) expandstack(x,scoy,&scoym3[x*SCPITCH*3]);
		for(x=x1;x>scx1;x--) expandstack(x,scoy,&scoym3[x*SCPITCH*3]);
	}
	scex0 = x0;
	scex1 = x1;

	y = scoy-1; if (y&(~(VSID-1))) return;
	if (x0 < 0) x0 = 0;
	if (x1 >= VSID) x1 = VSID-1;
	i = y*VSID+x0; k = x0*SCPITCH*3;
	for(x=x0;x<=x1;x++,i++,k+=SCPITCH*3)
	{
		v = sptr[i]; vx5.globalmass += v[1];
		while (v[0]) { v += v[0]*4; vx5.globalmass += v[1]-v[3]; }

			//De-allocate column (x,y)
		voxdealloc(sptr[i]);

		j = compilestack(&mptr[k],&mptr[k-SCPITCH*3],&mptr[k+SCPITCH*3],&uptr[k],&scoym3[k],
							  tbuf,x,y);

			//Allocate & copy to new column (x,y)
		sptr[i] = v = voxalloc(j); copybuf(cast(void *)tbuf.ptr,cast(void *)v,j>>2);

		vx5.globalmass -= v[1];
		while (v[0]) { v += v[0]*4; vx5.globalmass += v[3]-v[1]; }
	}
}

/**
*	x: x on voxel map
*	y: y on voxel map
*	z0: highest z on column
*	z1: lowest z(+1) on column
*	nbuf: buffer of color data from nbuf[z0] to nbuf[z1-1];
*	           -3: don't modify voxel
*	           -2: solid voxel (unexposed): to be calculated in compilestack
*	           -1: write air voxel
*	   0-16777215: write solid voxel (exposed)
*
*	Example of how to use this code:
*	vx5.colfunc = curcolfunc; //0 < x0 < x1 < VSID, 0 < y0 < y1< VSID, 0 < z0 < z1 < 256,
*	clearbuf((void *)&templongbuf[z0],z1-z0,-1); //Ex: set all voxels to air
*	for(y=y0;y < y1;y++) //MUST iterate x&y in this order, but can skip around
*	   for(x=x0;x < x1;x++)
*	      if (rand()&8) scum(x,y,z0,z1,templongbuf));
*	scumfinish(); //MUST call this when done!
*/	
void scum (int x, int y, int z0, int z1, int *nbuf)
{
	int z; 
	int* mptr;

	if ((x|y)&(~(VSID-1))) return;

	if (y != scoy)
	{
		if (scoy >= 0)
		{
			scumline();
			while (scoy < y-1)
			{
				scx0 = 0x7fffffff; scx1 = 0x80000000;
				scoy++; scoym3 += SCPITCH; if (scoym3 == &radar[SCPITCH*9]) scoym3 = &radar[SCPITCH*6];
				scumline();
			}
			scoy++; scoym3 += SCPITCH; if (scoym3 == &radar[SCPITCH*9]) scoym3 = &radar[SCPITCH*6];
		}
		else
		{
			scoox0 = scox0 = 0x7fffffff;
			sceox0 = scex0 = x+1;
			sceox1 = scex1 = x;
			scoy = y; scoym3 = &radar[SCPITCH*6];
		}
		scx0 = x;
	}
	else
	{
		while (scx1 < x-1) { scx1++; expandstack(scx1,y,&scoym3[scx1*SCPITCH*3]); }
	}

	mptr = &scoym3[x*SCPITCH*3]; scx1 = x; expandstack(x,y,mptr);

		//Modify column (x,y):
	if (nbuf[MAXZDIM-1] == -1) nbuf[MAXZDIM-1] = -2; //Bottom voxel must be solid
	for(z=z0;z<z1;z++)
		if (nbuf[z] != -3) mptr[z] = nbuf[z];
}

void scumfinish ()
{
	int i;

	if (scoy == 0x80000000) return;
	for(i=2;i;i--)
	{
		scumline(); scx0 = 0x7fffffff; scx1 = 0x80000000;
		scoy++; scoym3 += SCPITCH; if (scoym3 == &radar[SCPITCH*9]) scoym3 = &radar[SCPITCH*6];
	}
	scumline(); scoy = 0x80000000;
}

void scum2line ()
{
	int i, j, k, x, y, x0, x1;
	int* mptr, uptr;
	ubyte* v;

	x0 = min(scox0-1,min(scx0,scoox0)); scoox0 = scox0; scox0 = scx0;
	x1 = max(scox1+1,max(scx1,scoox1)); scoox1 = scox1; scox1 = scx1;

	uptr = &scoym3[SCPITCH]; if (uptr == &radar[SCPITCH*9]) uptr = &radar[SCPITCH*6];
	mptr = &uptr[SCPITCH];   if (mptr == &radar[SCPITCH*9]) mptr = &radar[SCPITCH*6];

	if ((x1 < sceox0) || (x0 > sceox1))
	{
		for(x=x0;x<=x1;x++) expandrle(x,scoy-2,&uptr[x*SCPITCH*3]);
	}
	else
	{
		for(x=x0;x<sceox0;x++) expandrle(x,scoy-2,&uptr[x*SCPITCH*3]);
		for(x=x1;x>sceox1;x--) expandrle(x,scoy-2,&uptr[x*SCPITCH*3]);
	}

	if ((scex1|x1) >= 0)
	{
		for(x=x1+2;x<scex0;x++) expandrle(x,scoy-1,&mptr[x*SCPITCH*3]);
		for(x=x0-2;x>scex1;x--) expandrle(x,scoy-1,&mptr[x*SCPITCH*3]);
	}
	if ((x1+1 < scex0) || (x0-1 > scex1))
	{
		for(x=x0-1;x<=x1+1;x++) expandrle(x,scoy-1,&mptr[x*SCPITCH*3]);
	}
	else
	{
		for(x=x0-1;x<scex0;x++) expandrle(x,scoy-1,&mptr[x*SCPITCH*3]);
		for(x=x1+1;x>scex1;x--) expandrle(x,scoy-1,&mptr[x*SCPITCH*3]);
	}
	sceox0 = min(x0-1,scex0);
	sceox1 = max(x1+1,scex1);

	if ((x1 < scx0) || (x0 > scx1))
	{
		for(x=x0;x<=x1;x++) expandrle(x,scoy,&scoym3[x*SCPITCH*3]);
	}
	else
	{
		for(x=x0;x<scx0;x++) expandrle(x,scoy,&scoym3[x*SCPITCH*3]);
		for(x=x1;x>scx1;x--) expandrle(x,scoy,&scoym3[x*SCPITCH*3]);
	}
	scex0 = x0;
	scex1 = x1;

	y = scoy-1; if (y&(~(VSID-1))) return;
	if (x0 < 0) x0 = 0;
	if (x1 >= VSID) x1 = VSID-1;
	i = y*VSID+x0; k = x0*SCPITCH*3;
	for(x=x0;x<=x1;x++,i++,k+=SCPITCH*3)
	{
		j = compilerle(&mptr[k],&mptr[k-SCPITCH*3],&mptr[k+SCPITCH*3],&uptr[k],&scoym3[k],
							tbuf,x,y);

		v = sptr[i]; vx5.globalmass += v[1];
		while (v[0]) { v += v[0]*4; vx5.globalmass += v[1]-v[3]; }

			//De-allocate column (x,y)  Note: Must be AFTER compilerle!
		voxdealloc(sptr[i]);

			//Allocate & copy to new column (x,y)
		sptr[i] = v = voxalloc(j); copybuf(cast(void *)tbuf.ptr,cast(void *)v,j>>2);

		vx5.globalmass -= v[1];
		while (v[0]) { v += v[0]*4; vx5.globalmass += v[3]-v[1]; }
	}
}

/**
*	x: x on voxel map
*	y: y on voxel map
*	Returns pointer to rle column (x,y)
*/
int* scum2 (int x, int y)
{
	int *mptr;

	if ((x|y)&(~(VSID-1))) return(0);

	if (y != scoy)
	{
		if (scoy >= 0)
		{
			scum2line();
			while (scoy < y-1)
			{
				scx0 = 0x7fffffff; scx1 = 0x80000000;
				scoy++; scoym3 += SCPITCH; if (scoym3 == &radar[SCPITCH*9]) scoym3 = &radar[SCPITCH*6];
				scum2line();
			}
			scoy++; scoym3 += SCPITCH; if (scoym3 == &radar[SCPITCH*9]) scoym3 = &radar[SCPITCH*6];
		}
		else
		{
			scoox0 = scox0 = 0x7fffffff;
			sceox0 = scex0 = x+1;
			sceox1 = scex1 = x;
			scoy = y; scoym3 = &radar[SCPITCH*6];
		}
		scx0 = x;
	}
	else
	{
		while (scx1 < x-1) { scx1++; expandrle(scx1,y,&scoym3[scx1*SCPITCH*3]); }
	}

	mptr = &scoym3[x*SCPITCH*3]; scx1 = x; expandrle(x,y,mptr);
	return(mptr);
}

void scum2finish ()
{
	int i;

	if (scoy == 0x80000000) return;
	for(i=2;i;i--)
	{
		scum2line(); scx0 = 0x7fffffff; scx1 = 0x80000000;
		scoy++; scoym3 += SCPITCH; if (scoym3 == &radar[SCPITCH*9]) scoym3 = &radar[SCPITCH*6];
	}
	scum2line(); scoy = 0x80000000;
}

//------------------- editing backup / restore begins ------------------------

void voxdontrestore ()
{
	int i;

	if (backedup == 1)
	{
		for(i=(bacx1-bacx0)*(bacy1-bacy0)-1;i>=0;i--) voxdealloc(bacsptr[i]);
	}
	backedup = -1;
}

void voxrestore ()
{
	int i, j, x, y;
	ubyte* v, daptr;

	if (backedup == 1)
	{
		i = 0;
		for(y=bacy0;y<bacy1;y++)
		{
			j = y*VSID;
			for(x=bacx0;x<bacx1;x++)
			{
				v = sptr[j+x]; vx5.globalmass += v[1];
				while (v[0]) { v += v[0]*4; vx5.globalmass += v[1]-v[3]; }

				voxdealloc(sptr[j+x]);
				sptr[j+x] = bacsptr[i]; i++;

				v = sptr[j+x]; vx5.globalmass -= v[1];
				while (v[0]) { v += v[0]*4; vx5.globalmass += v[3]-v[1]; }
			}
		}
		if (vx5.vxlmipuse > 1) genmipvxl(bacx0,bacy0,bacx1,bacy1);
	}
	else if (backedup == 2)
	{
		daptr = cast(ubyte *)bacsptr;
		for(y=bacy0;y<bacy1;y++)
		{
			j = y*VSID;
			for(x=bacx0;x<bacx1;x++)
			{
				for(v=sptr[j+x];v[0];v+=v[0]*4)
					for(i=1;i<v[0];i++) v[(i<<2)+3] = *daptr++;
				for(i=1;i<=v[2]-v[1]+1;i++) v[(i<<2)+3] = *daptr++;
			}
		}
		if (vx5.vxlmipuse > 1) genmipvxl(bacx0,bacy0,bacx1,bacy1);
	}
	backedup = -1;
}

void voxbackup (int x0, int y0, int x1, int y1, int tag)
{
	int i, j, n, x, y;
	ubyte* v, daptr;

	voxdontrestore();

	x0 = max(x0-2,0); y0 = max(y0-2,0);
	x1 = min(x1+2,VSID); y1 = min(y1+2,VSID);
	if ((x1-x0)*(y1-y0) > 262144) return;

	bacx0 = x0; bacy0 = y0; bacx1 = x1; bacy1 = y1; backtag = tag;

	if (tag&0x10000)
	{
		backedup = 1;
		i = 0;
		for(y=bacy0;y<bacy1;y++)
		{
			j = y*VSID;
			for(x=bacx0;x<bacx1;x++)
			{
				bacsptr[i] = v = sptr[j+x]; i++;
				n = slng(v); sptr[j+x] = voxalloc(n);

				copybuf(cast(void *)v,cast(void *)sptr[j+x],n>>2);
			}
		}
	}
	else if (tag&0x20000)
	{
		backedup = 2;
			//WARNING!!! Right now, function will crash if saving more than
			//   1<<20 colors :( This needs to be addressed!!!
		daptr = cast(ubyte *)bacsptr;
		for(y=bacy0;y<bacy1;y++)
		{
			j = y*VSID;
			for(x=bacx0;x<bacx1;x++)
			{
				for(v=sptr[j+x];v[0];v+=v[0]*4)
					for(i=1;i<v[0];i++) *daptr++ = v[(i<<2)+3];
				for(i=1;i<=v[2]-v[1]+1;i++) *daptr++ = v[(i<<2)+3];
			}
		}
	}
	else backedup = 0;
}

//-------------------- editing backup / restore ends -------------------------

/**
*	WARNING! Make sure to set vx5.colfunc before calling this function!
*	This function is here for simplicity only - it is NOT optimal.
*	
*	   -1: set air
*	   -2: use vx5.colfunc
*/
void setcube (int px, int py, int pz, int col)
{
	int bakcol;
	alias int function(lpoint3d*) da_bakcolfunc;
	da_bakcolfunc bakcolfunc;
	int *lptr;

	vx5.minx = px; vx5.maxx = px+1;
	vx5.miny = py; vx5.maxy = py+1;
	vx5.minz = pz; vx5.maxz = pz+1;
	if (cast(uint)pz >= MAXZDIM) return;
	if (cast(uint)col >= cast(uint)0xfffffffe) //-1 or -2
	{
		lptr = scum2(px,py);
		if (col == -1) delslab(lptr,pz,pz+1); else insslab(lptr,pz,pz+1);
		scum2finish();
		updatebbox(vx5.minx,vx5.miny,vx5.minz,vx5.maxx,vx5.maxy,vx5.maxz,col);
		return;
	}

	bakcol = getcube(px,py,pz);
	if (bakcol == 1) return; //Unexposed solid
	if (bakcol != 0) //Not 0 (air)
		*cast(int *)bakcol = col;
	else
	{
		bakcolfunc = vx5.colfunc; bakcol = vx5.curcol;
		vx5.colfunc = curcolfunc; vx5.curcol = col;
		insslab(scum2(px,py),pz,pz+1); scum2finish();
		vx5.colfunc = bakcolfunc; vx5.curcol = bakcol;
	}
	updatebbox(vx5.minx,vx5.miny,vx5.minz,vx5.maxx,vx5.maxy,vx5.maxz,0);
}

//-------------------------- SETCOLUMN CODE ENDS ----------------------------
static long qmulmip[8] =
[
	0x7fff7fff7fff7fff,0x4000400040004000,0x2aaa2aaa2aaa2aaa,0x2000200020002000,
	0x1999199919991999,0x1555155515551555,0x1249124912491249,0x1000100010001000
];
static int mixc[MAXZDIM>>1][8]; //4K
static int mixn[MAXZDIM>>1];    //0.5K
void genmipvxl (int x0, int y0, int x1, int y1)
{
	int i, n, oldn, x, y, z, xsiz, ysiz, zsiz, oxsiz, oysiz;
	int cz, oz, nz, zz, besti, cstat, mipnum, mipmax;
	int curz[4];
	int curzn[4][4];
	ubyte* v[4];
	ubyte* tv;
	ubyte** sr, sw, ssr, ssw;

	if ((!(x0|y0)) && (x1 == VSID) && (y1 == VSID)) mipmax = vx5.vxlmipuse;
															 else mipmax = gmipnum;
	if (mipmax <= 0) return;
	mipnum = 1;

	vx5.colfunc = curcolfunc;
	xsiz = VSID; ysiz = VSID; zsiz = MAXZDIM;
	ssr = sptr; ssw = sptr+xsiz*ysiz;
	while ((xsiz > 1) && (ysiz > 1) && (zsiz > 1) && (mipnum < mipmax))
	{
		oxsiz = xsiz; xsiz >>= 1;
		oysiz = ysiz; ysiz >>= 1;
						  zsiz >>= 1;

		x0--; if (x0 < 0) x0 = 0;
		y0--; if (y0 < 0) y0 = 0;
		x1++; if (x1 > VSID) x1 = VSID;
		y1++; if (y1 > VSID) y1 = VSID;

		x0 >>= 1; x1 = ((x1+1)>>1);
		y0 >>= 1; y1 = ((y1+1)>>1);
		for(y=y0;y<y1;y++)
		{
			sr = ssr+oxsiz*(y<<1)+(x0<<1);
			sw = ssw+xsiz*y+x0;
			for(x=x0;x<x1;x++)
			{
					//ÚÄÄÄÂÄÄÄÂÄÄÄÂÄÄÄ¿
					//³npt³z1 ³z1c³dum³
					//³ b ³ g ³ r ³ i ³
					//³ b ³ g ³ r ³ i ³
					//³npt³z1 ³z1c³z0 ³
					//³ b ³ g ³ r ³ i ³
					//ÀÄÄÄÁÄÄÄÁÄÄÄÁÄÄÄÙ
				v[0] = sr[      0];
				v[1] = sr[      1];
				v[2] = sr[oysiz  ];
				v[3] = sr[oysiz+1];
				for(i=3;i>=0;i--)
				{
					curz[i] = curzn[i][0] = cast(int)v[i][1];
					curzn[i][1] = (cast(int)v[i][2])+1;

					tv = v[i];
					while (1)
					{
						oz = cast(int)tv[1];
						for(z=oz;z<=(cast(int)tv[2]);z++)
						{
							nz = (z>>1);
							mixc[nz][mixn[nz]++] = *cast(int *)(&tv[((z-oz)<<2)+4]);
						}
						z = (z-oz) - ((cast(int)tv[0])-1);
						if (!tv[0]) break;
						tv += ((cast(int)tv[0])<<2);
						oz = cast(int)tv[3];
						for(;z<0;z++)
						{
							nz = ((z+oz)>>1);
							mixc[nz][mixn[nz]++] = *cast(int *)(&tv[z<<2]);
						}
					}
				}
				cstat = 0; oldn = 0; n = 4; tbuf[3] = 0; z = 0x80000000;
				while (true)
				{
					oz = z;

						//z,besti = min,argmin(curz[0],curz[1],curz[2],curz[3])
					besti = ((cast(uint)(curz[1]-curz[    0]))>>31);
						 i = ((cast(uint)(curz[3]-curz[    2]))>>31)+2;
					besti +=(((cast(int)(curz[i]-curz[besti]))>>31)&(i-besti));
					z = curz[besti]; if (z >= MAXZDIM) break;

					if ((!cstat) && ((z>>1) >= ((oz+1)>>1)))
					{
						if (oz >= 0)
						{
							tbuf[oldn] = ((n-oldn)>>2);
							tbuf[oldn+2]--;
							tbuf[n+3] = ((oz+1)>>1);
							oldn = n; n += 4;
						}
						tbuf[oldn] = 0;
						tbuf[oldn+1] = tbuf[oldn+2] = (z>>1); cz = -1;
					}
					if (cstat&0x1111)
					{
						if ((((cast(int)tbuf[oldn+2])<<1)+1 >= oz) && (cz < 0))
						{
							while (((cast(int)tbuf[oldn+2])<<1) < z)
							{
								zz = cast(int)tbuf[oldn+2];

								asm //*cast(int *)&tbuf[n] = mixc[zz][rand()%mixn[zz]];
								{    //mixn[zz] = 0;
									mov EAX, zz;
									mov ECX, mixn[EAX*4];
									mov mixn[EAX*4], 0;
									shl EAX, 5;
									pxor MM0, MM0;
									movq MM2, qmulmip[ECX*8-8];
									pcmpeqb MM6, MM6;
									movq MM7, MM0;
					 				
					 				vxlmipbeg0:
					 				movd MM1, mixc[EAX+ECX*4-4];
									punpcklbw MM1, MM7;
									paddw MM0, MM1;
									dec ECX;
									jnz short vxlmipbeg0;
									paddw MM0, MM0;
									psubw MM0, MM6; //rounding bias
									pmulhw MM0, MM2;
									packuswb MM0, MM0;
									mov EAX, n;
									movd tbuf[EAX], MM0;
								}

								tbuf[oldn+2]++; n += 4;
							}
						}
						else
						{
							if (cz < 0) cz = (oz>>1);
							else if ((cz<<1)+1 < oz)
							{
									//Insert fake slab
								tbuf[oldn] = ((n-oldn)>>2);
								tbuf[oldn+2]--;
								tbuf[n] = 0;
								tbuf[n+1] = tbuf[n+2] = tbuf[n+3] = cz;
								oldn = n; n += 4;
								cz = (oz>>1);
							}
							while ((cz<<1) < z)
							{
								asm //*cast(int *)&tbuf[n] = mixc[cz][rand()%mixn[cz]];
								{    //mixn[cz] = 0;
									mov EAX, cz;
									mov ECX, mixn[EAX*4];
									mov mixn[EAX*4], 0;
									shl EAX, 5;
									pxor MM0, MM0;
									movq MM2, qmulmip[ECX*8-8];
									pcmpeqb MM6, MM6;
									movq MM7, MM0;

					 				vxlmipbeg1:
					 				movd MM1, mixc[EAX+ECX*4-4];
									punpcklbw MM1, MM7;
									paddw MM0, MM1;
									dec ECX;
									jnz short vxlmipbeg1;
									paddw MM0, MM0;
									psubw MM0, MM6; //rounding bias
									pmulhw MM0, MM2;
									packuswb MM0, MM0;
									mov EAX, n;
									movd tbuf[EAX], MM0;
								}

								cz++; n += 4;
							}
						}
					}

					i = (besti<<2);
					cstat = (((1<<i)+cstat)&0x3333); //--33--22--11--00
					switch ((cstat>>i)&3)
					{
						case 0: curz[besti] = curzn[besti][0]; break;
						case 1: curz[besti] = curzn[besti][1]; break;
						case 2:
							if (!(v[besti][0])) { curz[besti] = MAXZDIM; }
							else
							{
								tv = v[besti]; i = ((cast(int)tv[2])-(cast(int)tv[1])+1)-((cast(int)tv[0])-1);
								tv += ((cast(int)tv[0])<<2);
								curz[besti] = (cast(int)(tv[3])) + i;
								curzn[besti][3] = cast(int)(tv[3]);
								curzn[besti][0] = cast(int)(tv[1]);
								curzn[besti][1] = (cast(int)tv[2])+1;
								v[besti] = tv;
							}
							break;
						case 3: curz[besti] = curzn[besti][3]; break;
						//default: __assume(0); //tells MSVC default can't be reached
					}
				}
				tbuf[oldn+2]--;
				if (cz >= 0)
				{
					tbuf[oldn] = ((n-oldn)>>2);
					tbuf[n] = 0;
					tbuf[n+1] = tbuf[n+3] = cz;
					tbuf[n+2] = cz-1;
					n += 4;
				}

					//De-allocate column (x,y) if it exists
				if (sw[0]) voxdealloc(sw[0]);

					//Allocate & copy to new column (x,y)
				sw[0] = voxalloc(n);
				copybuf(cast(void *)tbuf.ptr,cast(void *)sw[0],n>>2);
				sw++; sr += 2;
			}
			sr += ysiz*2;
		}
		ssr = ssw; ssw += xsiz*ysiz;
		mipnum++; if (mipnum > gmipnum) gmipnum = mipnum;
	}

		//Remove extra mips (bbox must be 0,0,VSID,VSID to get inside this)
	while ((xsiz > 1) && (ysiz > 1) && (zsiz > 1) && (mipnum < gmipnum))
	{
		xsiz >>= 1; ysiz >>= 1; zsiz >>= 1;
		for(i=xsiz*ysiz;i>0;i--)
		{
			if (ssw[0]) voxdealloc(ssw[0]); //De-allocate column if it exists
			ssw++;
		}
		gmipnum--;
	}

	asm {emms;}

	static if(false) //TEMP HACK!!!
	{
		{
			FILE *fil;
			dpoint3d dp;
			if (!(fil = fopen("temp512.vxl","wb"))) return;
			i = 0x09072000; fwrite(&i,4,1,fil);  //Version
			i = (VSID>>1); fwrite(&i,4,1,fil);
			i = (VSID>>1); fwrite(&i,4,1,fil);
			dp.x = cast(double)i*.5; dp.y = cast(double)i*.5; dp.z = cast(double)i*.5;
			fwrite(&dp,24,1,fil);
			dp.x = 1.0; dp.y = 0.0; dp.z = 0.0; fwrite(&dp,24,1,fil);
			dp.x = 0.0; dp.y = 0.0; dp.z = 1.0; fwrite(&dp,24,1,fil);
			dp.x = 0.0; dp.y =-1.0; dp.z = 0.0; fwrite(&dp,24,1,fil);
			for(i=0;i<(VSID>>1)*(VSID>>1);i++)
				fwrite(cast(void *)sptr[i+VSID*VSID],slng(sptr[i+VSID*VSID]),1,fil);
			fclose(fil);
		}
		gmipnum = 1;
	}

}

void setsphere (lpoint3d *hit, int hitrad, int dacol)
{
	alias void function(int*, int, int) da_modslab;
	da_modslab modslab;
	int i, x, y, xs, ys, zs, xe, ye, ze, sq;
	float f, ff;

	xs = max(hit.x-hitrad,0); xe = min(hit.x+hitrad,VSID-1);
	ys = max(hit.y-hitrad,0); ye = min(hit.y+hitrad,VSID-1);
	zs = max(hit.z-hitrad,0); ze = min(hit.z+hitrad,MAXZDIM-1);
	vx5.minx = xs; vx5.maxx = xe+1;
	vx5.miny = ys; vx5.maxy = ye+1;
	vx5.minz = zs; vx5.maxz = ze+1;
	if ((xs > xe) || (ys > ye) || (zs > ze)) return;

	if (vx5.colfunc == sphcolfunc)
	{
		vx5.cen = hit.x+hit.y+hit.z;
		vx5.daf = 1.f/(hitrad*sqrt(3.f));
	}

	if (hitrad >= SETSPHMAXRAD-1) hitrad = SETSPHMAXRAD-2;
	if (dacol == -1) modslab = delslab; else modslab = insslab;

	tempfloatbuf[0] = 0.0f;
	static if(false)
	{
			//Totally unoptimized
		for(i=1;i<=hitrad;i++) tempfloatbuf[i] = pow(cast(float)i,vx5.curpow);
	} else
	{
		tempfloatbuf[1] = 1.0f;
		for(i=2;i<=hitrad;i++)
		{
			if (!factr[i][0]) tempfloatbuf[i] = exp(logint[i]*vx5.curpow);
			else tempfloatbuf[i] = tempfloatbuf[factr[i][0]]*tempfloatbuf[factr[i][1]];
		}
	}
	*cast(int *)&tempfloatbuf[hitrad+1] = 0x7f7fffff; //3.4028235e38f; //Highest float

	sq = 0; //pow(fabs(x-hit.x),vx5.curpow) + "y + "z < pow(vx5.currad,vx5.curpow)
	for(y=ys;y<=ye;y++)
	{
		ff = tempfloatbuf[hitrad]-tempfloatbuf[labs(y-hit.y)];
		if (*cast(int *)&ff <= 0) continue;
		for(x=xs;x<=xe;x++)
		{
			f = ff-tempfloatbuf[labs(x-hit.x)]; if (*cast(int *)&f <= 0) continue;
			while (*cast(int *)&tempfloatbuf[sq] <  *cast(int *)&f) sq++;
			while (*cast(int *)&tempfloatbuf[sq] >= *cast(int *)&f) sq--;
			modslab(scum2(x,y),max(hit.z-sq,zs),min(hit.z+sq+1,ze));
		}
	}
	scum2finish();
	updatebbox(vx5.minx,vx5.miny,vx5.minz,vx5.maxx,vx5.maxy,vx5.maxz,dacol);
}

void setellipsoid (lpoint3d *hit, lpoint3d *hit2, int hitrad, int dacol, int bakit)
{
	alias void function(int*, int, int) da_modslab;
	da_modslab modslab;
	int x, y, xs, ys, zs, xe, ye, ze;
	float a, b, c, d, e, f, g, h, r, t, u, Za, Zb, fx0, fy0, fz0, fx1, fy1, fz1;

	xs = min(hit.x,hit2.x)-hitrad; xs = max(xs,0);
	ys = min(hit.y,hit2.y)-hitrad; ys = max(ys,0);
	zs = min(hit.z,hit2.z)-hitrad; zs = max(zs,0);
	xe = max(hit.x,hit2.x)+hitrad; xe = min(xe,VSID-1);
	ye = max(hit.y,hit2.y)+hitrad; ye = min(ye,VSID-1);
	ze = max(hit.z,hit2.z)+hitrad; ze = min(ze,MAXZDIM-1);
	vx5.minx = xs; vx5.maxx = xe+1;
	vx5.miny = ys; vx5.maxy = ye+1;
	vx5.minz = zs; vx5.maxz = ze+1;
	if ((xs > xe) || (ys > ye) || (zs > ze))
		{ if (bakit) voxbackup(xs,ys,xs,ys,bakit); return; }

	fx0 = cast(float)hit.x; fy0 = cast(float)hit.y; fz0 = cast(float)hit.z;
	fx1 = cast(float)hit2.x; fy1 = cast(float)hit2.y; fz1 = cast(float)hit2.z;

	r = (fx1-fx0)*(fx1-fx0) + (fy1-fy0)*(fy1-fy0) + (fz1-fz0)*(fz1-fz0);
	r = sqrt(cast(float)hitrad*cast(float)hitrad + r*.25);
	c = fz0*fz0 - fz1*fz1; d = r*r*-4; e = d*4;
	f = c*c + fz1*fz1 * e; g = c + c; h = (fz1-fz0)*2; c = c*h - fz1*e;
	Za = -h*h - e; if (Za <= 0) { if (bakit) voxbackup(xs,ys,xs,ys,bakit); return; }
	u = 1 / Za;

	if (vx5.colfunc == jitcolfunc) vx5.amount = 0x70707;

	if (dacol == -1) modslab = delslab; else modslab = insslab;

	if (bakit) voxbackup(xs,ys,xe+1,ye+1,bakit);

	for(y=ys;y<=ye;y++)
		for(x=xs;x<=xe;x++)
		{
			a = (x-fx0)*(x-fx0) + (y-fy0)*(y-fy0);
			b = (x-fx1)*(x-fx1) + (y-fy1)*(y-fy1);
			t = a-b+d; Zb = t*h + c;
			t = ((t+g)*t + b*e + f)*Za + Zb*Zb; if (t <= 0) continue;
			t = sqrt(t);
			ftol((Zb - t)*u,&zs); if (zs < 0) zs = 0;
			ftol((Zb + t)*u,&ze); if (ze > MAXZDIM) ze = MAXZDIM;
			modslab(scum2(x,y),zs,ze);
		}
	scum2finish();
	updatebbox(vx5.minx,vx5.miny,vx5.minz,vx5.maxx,vx5.maxy,vx5.maxz,dacol);
}

/**
*	Draws a cylinder, given: 2 points, a radius, and a color
*	Code mostly optimized - original code from CYLINDER.BAS:drawcylinder
*/
void setcylinder (lpoint3d* p0, lpoint3d* p1, int cr, int dacol, int bakit)
{
	alias void function(int*, int, int) da_modslab;
	da_modslab modslab;

	float t, ax, ay, az, bx, by, bz, cx, cy, cz, ux, uy, uz, vx, vy, vz;
	float Za, Zb, Zc, tcr, xxyy, rcz, rZa;
	float fx, fxi, xof, vx0, vy0, vz0, vz0i, vxo, vyo, vzo;
	int i, j, ix, iy, ix0, ix1, iz0, iz1, minx, maxx, miny, maxy;
	int x0, y0, z0, x1, y1, z1;

		//Map generic cylinder into unit space:  (0,0,0), (0,0,1), cr = 1
		//   x*x + y*y < 1, z >= 0, z < 1
	if (p0.z > p1.z)
	{
		x0 = p1.x; y0 = p1.y; z0 = p1.z;
		x1 = p0.x; y1 = p0.y; z1 = p0.z;
	}
	else
	{
		x0 = p0.x; y0 = p0.y; z0 = p0.z;
		x1 = p1.x; y1 = p1.y; z1 = p1.z;
	}

	xxyy = cast(float)((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0));
	t = xxyy + cast(float)(z1-z0)*(z1-z0);
	if ((t == 0) || (cr == 0))
	{
		vx5.minx = x0; vx5.maxx = x0+1;
		vx5.miny = y0; vx5.maxy = y0+1;
		vx5.minz = z0; vx5.maxz = z0+1;
		if (bakit) voxbackup(x0,y0,x0,y0,bakit);
		return;
	}
	t = 1 / t; cx = (cast(float)(x1-x0))*t; cy = (cast(float)(y1-y0))*t; cz = (cast(float)(z1-z0))*t;
	t = sqrt(t); ux = (cast(float)(x1-x0))*t; uy = (cast(float)(y1-y0))*t; uz = (cast(float)(z1-z0))*t;

	if (vx5.colfunc == jitcolfunc) vx5.amount = 0x70707;

	if (dacol == -1) modslab = delslab; else modslab = insslab;

	if (xxyy == 0)
	{
		iz0 = max(z0,0); iz1 = min(z1,MAXZDIM);
		minx = max(x0-cr,0); maxx = min(x0+cr,VSID-1);
		miny = max(y0-cr,0); maxy = min(y0+cr,VSID-1);

		vx5.minx = minx; vx5.maxx = maxx+1;
		vx5.miny = miny; vx5.maxy = maxy+1;
		vx5.minz = iz0; vx5.maxz = iz1;
		if (bakit) voxbackup(minx,miny,maxx+1,maxy+1,bakit);

		j = cr*cr;
		for(iy=miny;iy<=maxy;iy++)
		{
			i = j-(iy-y0)*(iy-y0);
			for(ix=minx;ix<=maxx;ix++)
				if ((ix-x0)*(ix-x0) < i) modslab(scum2(ix,iy),iz0,iz1);
		}
		scum2finish();
		updatebbox(vx5.minx,vx5.miny,vx5.minz,vx5.maxx,vx5.maxy,vx5.maxz,dacol);
		return;
	}

	if (x0 < x1) { minx = x0; maxx = x1; } else { minx = x1; maxx = x0; }
	if (y0 < y1) { miny = y0; maxy = y1; } else { miny = y1; maxy = y0; }
	tcr = cr / sqrt(xxyy); vx = fabs(cast(float)(x1-x0))*tcr; vy = fabs(cast(float)(y1-y0))*tcr;
	t = vx*uz + vy;
	ftol(cast(float)minx-t,&minx); if (minx < 0) minx = 0;
	ftol(cast(float)maxx+t,&maxx); if (maxx >= VSID) maxx = VSID-1;
	t = vy*uz + vx;
	ftol(cast(float)miny-t,&miny); if (miny < 0) miny = 0;
	ftol(cast(float)maxy+t,&maxy); if (maxy >= VSID) maxy = VSID-1;

	vx5.minx = minx; vx5.maxx = maxx+1;
	vx5.miny = miny; vx5.maxy = maxy+1;
	vx5.minz = z0-cr; vx5.maxz = z1+cr+1;
	if (bakit) voxbackup(minx,miny,maxx+1,maxy+1,bakit);

	vx = (fabs(ux) < fabs(uy)); vy = 1.0f-vx; vz = 0;
	ax = uy*vz - uz*vy; ay = uz*vx - ux*vz; az = ux*vy - uy*vx;
	t = 1.0 / (sqrt(ax*ax + ay*ay + az*az)*cr);
	ax *= t; ay *= t; az *= t;
	bx = ay*uz - az*uy; by = az*ux - ax*uz; bz = ax*uy - ay*ux;

	Za = az*az + bz*bz; rZa = 1.0f / Za;
	if (cz != 0) { rcz = 1.0f / cz; vz0i = -rcz*cx; }
	if (y0 != y1)
	{
		t = 1.0f / (cast(float)(y1-y0)); fxi = (cast(float)(x1-x0))*t;
		fx = (cast(float)miny-y0)*fxi + x0; xof = fabs(tcr*xxyy*t);
	}
	else { fx = cast(float)minx; fxi = 0.0; xof = cast(float)(maxx-minx); }

	vy = cast(float)(miny-y0);
	vxo = vy*ay - z0*az;
	vyo = vy*by - z0*bz;
	vzo = vy*cy - z0*cz;
	for(iy=miny;iy<=maxy;iy++)
	{
		ftol(fx-xof,&ix0); if (ix0 < minx) ix0 = minx;
		ftol(fx+xof,&ix1); if (ix1 > maxx) ix1 = maxx;
		fx += fxi;

		vx = cast(float)(ix0-x0);
		vx0 = vx*ax + vxo; vxo += ay;
		vy0 = vx*bx + vyo; vyo += by;
		vz0 = vx*cx + vzo; vzo += cy;

		if (cz != 0)   //(vx0 + vx1*t)ý + (vy0 + vy1*t)ý = 1
		{
			vz0 *= -rcz;
			for(ix=ix0;ix<=ix1;ix++,vx0+=ax,vy0+=bx,vz0+=vz0i)
			{
				Zb = vx0*az + vy0*bz; Zc = vx0*vx0 + vy0*vy0 - 1;
				t = Zb*Zb - Za*Zc; if (*cast(int *)&t <= 0) continue; t = sqrt(t);
				ftol(max((-Zb-t)*rZa,vz0    ),&iz0); if (iz0 < 0) iz0 = 0;
				ftol(min((-Zb+t)*rZa,vz0+rcz),&iz1); if (iz1 > MAXZDIM) iz1 = MAXZDIM;
				modslab(scum2(ix,iy),iz0,iz1);
			}
		}
		else
		{
			for(ix=ix0;ix<=ix1;ix++,vx0+=ax,vy0+=bx,vz0+=cx)
			{
				if (*cast(uint *)&vz0 >= 0x3f800000) continue; //vz0<0||vz0>=1
				Zb = vx0*az + vy0*bz; Zc = vx0*vx0 + vy0*vy0 - 1;
				t = Zb*Zb - Za*Zc; if (*cast(int *)&t <= 0) continue; t = sqrt(t);
				ftol((-Zb-t)*rZa,&iz0); if (iz0 < 0) iz0 = 0;
				ftol((-Zb+t)*rZa,&iz1); if (iz1 > MAXZDIM) iz1 = MAXZDIM;
				modslab(scum2(ix,iy),iz0,iz1);
			}
		}
	}
	scum2finish();
	updatebbox(vx5.minx,vx5.miny,vx5.minz,vx5.maxx,vx5.maxy,vx5.maxz,dacol);
}

/**
*	Draws a rectangle, given: 2 points as opposite corners, and a color
*/
void setrect (lpoint3d *hit, lpoint3d *hit2, int dacol)
{
	int x, y, xs, ys, zs, xe, ye, ze;

		//WARNING: do NOT use lbound because 'c' not guaranteed to be >= 'b'
	xs = max(min(hit.x,hit2.x),0); xe = min(max(hit.x,hit2.x),VSID-1);
	ys = max(min(hit.y,hit2.y),0); ye = min(max(hit.y,hit2.y),VSID-1);
	zs = max(min(hit.z,hit2.z),0); ze = min(max(hit.z,hit2.z),MAXZDIM-1);
	vx5.minx = xs; vx5.maxx = xe+1;
	vx5.miny = ys; vx5.maxy = ye+1;
	vx5.minz = zs; vx5.maxz = ze+1;
	if ((xs > xe) || (ys > ye) || (zs > ze)) return;

	if (vx5.colfunc == jitcolfunc) vx5.amount = 0x70707;

	ze++;
	if (dacol == -1)
	{
		for(y=ys;y<=ye;y++)
			for(x=xs;x<=xe;x++)
				delslab(scum2(x,y),zs,ze);
	}
	else
	{
		for(y=ys;y<=ye;y++)
			for(x=xs;x<=xe;x++)
				insslab(scum2(x,y),zs,ze);
	}
	scum2finish();
	updatebbox(vx5.minx,vx5.miny,vx5.minz,vx5.maxx,vx5.maxy,vx5.maxz,dacol);
}

/**
*	Does CSG using pre-sorted spanlist
*/
void setspans (vspans* lst, int lstnum, lpoint3d* offs, int dacol)
{
	alias void function(int*, int, int) da_modslab;
	da_modslab modslab;

	int i, j, x, y, z0, z1;
	int *lptr;
	ubyte ox, oy;

	if (lstnum <= 0) return;
	if (dacol == -1) modslab = delslab; else modslab = insslab;
	vx5.minx = vx5.maxx = (cast(int)lst[0].x)+offs.x;
	vx5.miny = (cast(int)lst[       0].y)+offs.y;
	vx5.maxy = (cast(int)lst[lstnum-1].y)+offs.y+1;
	vx5.minz = vx5.maxz = (cast(int)lst[0].z0)+offs.z;

	i = 0; goto in2setlist;
	do
	{
		if ((ox != lst[i].x) || (oy != lst[i].y))
		{
in2setlist:;
			ox = lst[i].x; oy = lst[i].y;
			x = (cast(int)lst[i].x)+offs.x;
			y = (cast(int)lst[i].y)+offs.y;
				  if (x < vx5.minx) vx5.minx = x;
			else if (x > vx5.maxx) vx5.maxx = x;
			lptr = scum2(x,y);
		}
		if ((x|y)&(~(VSID-1))) { i++; continue; }
		z0 = (cast(int)lst[i].z0)+offs.z;   if (z0 < 0) z0 = 0;
		z1 = (cast(int)lst[i].z1)+offs.z+1; if (z1 > MAXZDIM) z1 = MAXZDIM;
		if (z0 < vx5.minz) vx5.minz = z0;
		if (z1 > vx5.maxz) vx5.maxz = z1;
		modslab(lptr,z0,z1);
		i++;
	} while (i < lstnum);
	vx5.maxx++; vx5.maxz++;
	if (vx5.minx < 0) vx5.minx = 0;
	if (vx5.miny < 0) vx5.miny = 0;
	if (vx5.maxx > VSID) vx5.maxx = VSID;
	if (vx5.maxy > VSID) vx5.maxy = VSID;

	scum2finish();
	updatebbox(vx5.minx,vx5.miny,vx5.minz,vx5.maxx,vx5.maxy,vx5.maxz,dacol);
}

void setheightmap (const ubyte* hptr, int hpitch, int hxdim, int hydim,
						 int x0, int y0, int x1, int y1)
{
	int x, y, su, sv, u, v;

	if (x0 < 0) x0 = 0;
	if (y0 < 0) y0 = 0;
	if (x1 > VSID) x1 = VSID;
	if (y1 > VSID) y1 = VSID;
	vx5.minx = x0; vx5.maxx = x1;
	vx5.miny = y0; vx5.maxy = y1;
	vx5.minz = 0; vx5.maxz = MAXZDIM;
	if ((x0 >= x1) || (y0 >= y1)) return;

	su = x0%hxdim; sv = y0%hydim;
	for(y=y0,v=sv;y<y1;y++)
	{
		for(x=x0,u=su;x<x1;x++)
		{
			insslab(scum2(x,y),hptr[v*hpitch+u],MAXZDIM);
			u++; if (u >= hxdim) u = 0;
		}
		v++; if (v >= hydim) v = 0;
	}
	scum2finish();
	updatebbox(vx5.minx,vx5.miny,vx5.minz,vx5.maxx,vx5.maxy,vx5.maxz,0);
}

static int[VSID] min0, max0; //MAXY
static int[VSID] min1, max1; //MAXX
static int[VSID] min2, max2; //MAXY

static void canseerange (point3d* p0, point3d* p1)
{
	lpoint3d a, c, d, p, i;
	point3d f, g;
	int cnt, j;

	ftol(p0.x-0.5,&a.x); ftol(p0.y-0.5,&a.y); ftol(p0.z-0.5,&a.z);
	ftol(p1.x-0.5,&c.x); ftol(p1.y-0.5,&c.y); ftol(p1.z-0.5,&c.z);
	cnt = 0;

		  if (c.x <  a.x) { d.x = -1; f.x = p0.x-a.x;   g.x = (p0.x-p1.x)*1024; cnt += a.x-c.x; }
	else if (c.x != a.x) { d.x =  1; f.x = a.x+1-p0.x; g.x = (p1.x-p0.x)*1024; cnt += c.x-a.x; }
	else f.x = g.x = 0;
		  if (c.y <  a.y) { d.y = -1; f.y = p0.y-a.y;   g.y = (p0.y-p1.y)*1024; cnt += a.y-c.y; }
	else if (c.y != a.y) { d.y =  1; f.y = a.y+1-p0.y; g.y = (p1.y-p0.y)*1024; cnt += c.y-a.y; }
	else f.y = g.y = 0;
		  if (c.z <  a.z) { d.z = -1; f.z = p0.z-a.z;   g.z = (p0.z-p1.z)*1024; cnt += a.z-c.z; }
	else if (c.z != a.z) { d.z =  1; f.z = a.z+1-p0.z; g.z = (p1.z-p0.z)*1024; cnt += c.z-a.z; }
	else f.z = g.z = 0;

	ftol(f.x*g.z - f.z*g.x,&p.x); ftol(g.x,&i.x);
	ftol(f.y*g.z - f.z*g.y,&p.y); ftol(g.y,&i.y);
	ftol(f.y*g.x - f.x*g.y,&p.z); ftol(g.z,&i.z);
	for(;cnt;cnt--)
	{
			//use a.x, a.y, a.z
		if (a.x < min0[a.y]) min0[a.y] = a.x;
		if (a.x > max0[a.y]) max0[a.y] = a.x;
		if (a.z < min1[a.x]) min1[a.x] = a.z;
		if (a.z > max1[a.x]) max1[a.x] = a.z;
		if (a.z < min2[a.y]) min2[a.y] = a.z;
		if (a.z > max2[a.y]) max2[a.y] = a.z;

		if (((p.x|p.y) >= 0) && (a.z != c.z)) { a.z += d.z; p.x -= i.x; p.y -= i.y; }
		else if ((p.z >= 0) && (a.x != c.x))  { a.x += d.x; p.x += i.z; p.z -= i.y; }
		else                                  { a.y += d.y; p.y += i.z; p.z += i.x; }
	}
}

void settri (point3d* p0, point3d* p1, point3d* p2, int bakit)
{
	point3d n;
	float f, x0, y0, z0, x1, y1, z1, rx, ry, k0, k1;
	int i, x, y, z, iz0, iz1, minx, maxx, miny, maxy;

	if (p0.x < p1.x) { x0 = p0.x; x1 = p1.x; } else { x0 = p1.x; x1 = p0.x; }
	if (p2.x < x0) x0 = p2.x;
	if (p2.x > x1) x1 = p2.x;
	if (p0.y < p1.y) { y0 = p0.y; y1 = p1.y; } else { y0 = p1.y; y1 = p0.y; }
	if (p2.y < y0) y0 = p2.y;
	if (p2.y > y1) y1 = p2.y;
	if (p0.z < p1.z) { z0 = p0.z; z1 = p1.z; } else { z0 = p1.z; z1 = p0.z; }
	if (p2.z < z0) z0 = p2.z;
	if (p2.z > z1) z1 = p2.z;

	ftol(x0-.5,&minx); ftol(y0-.5,&miny);
	ftol(x1-.5,&maxx); ftol(y1-.5,&maxy);
	vx5.minx = minx; vx5.maxx = maxx+1;
	vx5.miny = miny; vx5.maxy = maxy+1;
	ftol(z0-.5,&vx5.minz); ftol(z1+.5,&vx5.maxz);
	if (bakit) voxbackup(minx,miny,maxx+1,maxy+1,bakit);

	for(i=miny;i<=maxy;i++) { min0[i] = 0x7fffffff; max0[i] = 0x80000000; }
	for(i=minx;i<=maxx;i++) { min1[i] = 0x7fffffff; max1[i] = 0x80000000; }
	for(i=miny;i<=maxy;i++) { min2[i] = 0x7fffffff; max2[i] = 0x80000000; }

	canseerange(p0,p1);
	canseerange(p1,p2);
	canseerange(p2,p0);

	n.x = (p1.z-p0.z)*(p2.y-p1.y) - (p1.y-p0.y) * (p2.z-p1.z);
	n.y = (p1.x-p0.x)*(p2.z-p1.z) - (p1.z-p0.z) * (p2.x-p1.x);
	n.z = (p1.y-p0.y)*(p2.x-p1.x) - (p1.x-p0.x) * (p2.y-p1.y);
	f = 1.0 / sqrt(n.x*n.x + n.y*n.y + n.z*n.z); if (n.z < 0) f = -f;
	n.x *= f; n.y *= f; n.z *= f;

	if (n.z > .01)
	{
		f = -1.0 / n.z; rx = n.x*f; ry = n.y*f;
		k0 = ((n.x>=0)-p0.x)*rx + ((n.y>=0)-p0.y)*ry - ((n.z>=0)-p0.z) + .5;
		k1 = ((n.x< 0)-p0.x)*rx + ((n.y< 0)-p0.y)*ry - ((n.z< 0)-p0.z) - .5;
	}
	else { rx = 0; ry = 0; k0 = -2147000000.0; k1 = 2147000000.0; }

	for(y=miny;y<=maxy;y++)
		for(x=min0[y];x<=max0[y];x++)
		{
			f = cast(float)x*rx + cast(float)y*ry; ftol(f+k0,&iz0); ftol(f+k1,&iz1);
			if (iz0 < min1[x]) iz0 = min1[x];
			if (iz1 > max1[x]) iz1 = max1[x];
			if (iz0 < min2[y]) iz0 = min2[y];
			if (iz1 > max2[y]) iz1 = max2[y];

				//set: (x,y,iz0) to (x,y,iz1) (inclusive)
			insslab(scum2(x,y),iz0,iz1+1);
	}
	scum2finish();
	updatebbox(vx5.minx,vx5.miny,vx5.minz,vx5.maxx,vx5.maxy,vx5.maxz,0);
}

static int tripind[24] = [0,4,1,5,2,6,3,7,0,2,1,3,4,6,5,7,0,1,2,3,4,5,6,7];
/**
*	Known problems:
*	1. Need to test faces for intersections on p1<.p2 line (not just edges)
*	2. Doesn't guarantee that hit point/line is purely air (but very close)
*	3. Piescan is more useful for parts of rope code :/
*/
int triscan (point3d* p0, point3d* p1, point3d* p2, point3d* hit, lpoint3d* lhit)
{
	point3d n, cp2;
	point3d d[8];
	float f, g, x0, x1, y0, y1, rx, ry, k0, k1, fx, fy, fz;
	float pval[8];
	int i, j, k, x, y, z, iz0, iz1, minx, maxx, miny, maxy, didhit;

	didhit = 0;

	if (p0.x < p1.x) { x0 = p0.x; x1 = p1.x; } else { x0 = p1.x; x1 = p0.x; }
	if (p2.x < x0) x0 = p2.x;
	if (p2.x > x1) x1 = p2.x;
	if (p0.y < p1.y) { y0 = p0.y; y1 = p1.y; } else { y0 = p1.y; y1 = p0.y; }
	if (p2.y < y0) y0 = p2.y;
	if (p2.y > y1) y1 = p2.y;
	ftol(x0-.5,&minx); ftol(y0-.5,&miny);
	ftol(x1-.5,&maxx); ftol(y1-.5,&maxy);
	for(i=miny;i<=maxy;i++) { min0[i] = 0x7fffffff; max0[i] = 0x80000000; }
	for(i=minx;i<=maxx;i++) { min1[i] = 0x7fffffff; max1[i] = 0x80000000; }
	for(i=miny;i<=maxy;i++) { min2[i] = 0x7fffffff; max2[i] = 0x80000000; }

	canseerange(p0,p1);
	canseerange(p1,p2);
	canseerange(p2,p0);

	n.x = (p1.z-p0.z)*(p2.y-p1.y) - (p1.y-p0.y) * (p2.z-p1.z);
	n.y = (p1.x-p0.x)*(p2.z-p1.z) - (p1.z-p0.z) * (p2.x-p1.x);
	n.z = (p1.y-p0.y)*(p2.x-p1.x) - (p1.x-p0.x) * (p2.y-p1.y);
	f = 1.0 / sqrt(n.x*n.x + n.y*n.y + n.z*n.z); if (n.z < 0) f = -f;
	n.x *= f; n.y *= f; n.z *= f;

	if (n.z > .01)
	{
		f = -1.0 / n.z; rx = n.x*f; ry = n.y*f;
		k0 = ((n.x>=0)-p0.x)*rx + ((n.y>=0)-p0.y)*ry - ((n.z>=0)-p0.z) + .5;
		k1 = ((n.x< 0)-p0.x)*rx + ((n.y< 0)-p0.y)*ry - ((n.z< 0)-p0.z) - .5;
	}
	else { rx = 0; ry = 0; k0 = -2147000000.0; k1 = 2147000000.0; }

	cp2.x = p2.x; cp2.y = p2.y; cp2.z = p2.z;

	for(y=miny;y<=maxy;y++)
		for(x=min0[y];x<=max0[y];x++)
		{
			f = cast(float)x*rx + cast(float)y*ry; ftol(f+k0,&iz0); ftol(f+k1,&iz1);
			if (iz0 < min1[x]) iz0 = min1[x];
			if (iz1 > max1[x]) iz1 = max1[x];
			if (iz0 < min2[y]) iz0 = min2[y];
			if (iz1 > max2[y]) iz1 = max2[y];
			for(z=iz0;z<=iz1;z++)
			{
				if (!isvoxelsolid(x,y,z)) continue;

				for(i=0;i<8;i++)
				{
					d[i].x = cast(float)(( i    &1)+x);
					d[i].y = cast(float)(((i>>1)&1)+y);
					d[i].z = cast(float)(((i>>2)&1)+z);
					pval[i] = (d[i].x-p0.x)*n.x + (d[i].y-p0.y)*n.y + (d[i].z-p0.z)*n.z;
				}
				for(i=0;i<24;i+=2)
				{
					j = tripind[i+0];
					k = tripind[i+1];
					if (((*cast(int *)&pval[j])^(*cast(int *)&pval[k])) < 0)
					{
						f = pval[j]/(pval[j]-pval[k]);
						fx = (d[k].x-d[j].x)*f + d[j].x;
						fy = (d[k].y-d[j].y)*f + d[j].y;
						fz = (d[k].z-d[j].z)*f + d[j].z;

							//         (p0.x,p0.y,p0.z)
							//             _|     |_
							//           _|     .   |_
							//         _|  (fx,fy,fz) |_
							//       _|                 |_
							//(p1.x,p1.y,p1.z)-.----(cp2.x,cp2.y,cp2.z)
	
						if ((fabs(n.z) > fabs(n.x)) && (fabs(n.z) > fabs(n.y)))
						{ //x,y
						  // ix = p1.x + (cp2.x-p1.x)*t;
						  // iy = p1.y + (cp2.y-p1.y)*t;
						  //(iz = p1.z + (cp2.z-p1.z)*t;)
						  // ix = p0.x + (fx-p0.x)*u;
						  // iy = p0.y + (fy-p0.y)*u;
						  // (p1.x-cp2.x)*t + (fx-p0.x)*u = p1.x-p0.x;
						  // (p1.y-cp2.y)*t + (fy-p0.y)*u = p1.y-p0.y;
	
							f = (p1.x-cp2.x)*(fy-p0.y) - (p1.y-cp2.y)*(fx-p0.x);
							if ((*cast(int *)&f) == 0) continue;
							f = 1.0 / f;
							g = ((p1.x-cp2.x)*(p1.y-p0.y) - (p1.y-cp2.y)*(p1.x-p0.x))*f;
							//NOTE: The following trick assumes g not * or / by f!
							//if (((*cast(int *)&g)-(*cast(int *)&f))^(*cast(int *)&f)) >= 0) continue;
							if ((*cast(int *)&g) < 0x3f800000) continue;
							g = ((p1.x-p0.x)*(fy-p0.y) - (p1.y-p0.y)*(fx-p0.x))*f;
						}
						else if (fabs(n.y) > fabs(n.x))
						{ //x,z
							f = (p1.x-cp2.x)*(fz-p0.z) - (p1.z-cp2.z)*(fx-p0.x);
							if ((*cast(int *)&f) == 0) continue;
							f = 1.0 / f;
							g = ((p1.x-cp2.x)*(p1.z-p0.z) - (p1.z-cp2.z)*(p1.x-p0.x))*f;
							if ((*cast(int *)&g) < 0x3f800000) continue;
							g = ((p1.x-p0.x)*(fz-p0.z) - (p1.z-p0.z)*(fx-p0.x))*f;
						}
						else
						{ //y,z
							f = (p1.y-cp2.y)*(fz-p0.z) - (p1.z-cp2.z)*(fy-p0.y);
							if ((*cast(int *)&f) == 0) continue;
							f = 1.0 / f;
							g = ((p1.y-cp2.y)*(p1.z-p0.z) - (p1.z-cp2.z)*(p1.y-p0.y))*f;
							if ((*cast(int *)&g) < 0x3f800000) continue;
							g = ((p1.y-p0.y)*(fz-p0.z) - (p1.z-p0.z)*(fy-p0.y))*f;
						}
						if ((*cast(uint *)&g) >= 0x3f800000) continue;
						(hit.x) = fx; (hit.y) = fy; (hit.z) = fz;
						(lhit.x) = x; (lhit.y) = y; (lhit.z) = z; didhit = 1;
						(cp2.x) = (cp2.x-p1.x)*g + p1.x;
						(cp2.y) = (cp2.y-p1.y)*g + p1.y;
						(cp2.z) = (cp2.z-p1.z)*g + p1.z;
					}
				}
			}
		}
	return(didhit);
}

// ------------------------ CONVEX 3D HULL CODE BEGINS ------------------------

enum MAXPOINTS = 256*2; //Leave the *2 here for safety!
point3d nm[MAXPOINTS*2+2];
float nmc[MAXPOINTS*2+2];
int tri[MAXPOINTS*8+8];
int lnk[MAXPOINTS*8+8];
int tricnt;
ubyte umost[VSID*VSID];
ubyte dmost[VSID*VSID];

// line 6700 initetrasid