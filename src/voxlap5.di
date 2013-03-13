//based on VOXLAP engine by Ken Silverman (http://advsys.net/ken)
module voxlap5;

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
	long x, y, z;
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
	long col;
	ushort z;
	char vis, dir;
}

align(1) struct kv6data
{
	long leng, xsiz, ysiz, zsiz;
	float xpiv, ypiv, zpiv;
	ulong numvoxs;
	long namoff;
	align(4) kv6data *lowermip; // to avoid gc problems
	align(4) kv6voxtype *vox; // numvoxs*sizeof(kv6voxtype)
	ulong *xlen; // xsiz*sizeof(long)
	ushort *ylen; // xsiz*ysiz*sizeof(short)
}

align(1) struct hingertype
{
	/// index to parent sprite (-1 = none)
	long parent;
	/// "velcro" point of each object
	point3d p[2];
	/// axis of rotation for each object
	point3d v[2];
	/// min value / max value
	short vmin, vmax;
	char htype;
	char filler[7];
}

align(1) struct seqtyp
{
	long tim, frm;
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
	long flags;
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
	long kfatim;
	/// kv6data.zsiz direction in VXL coordinates
	static union {point3d f, z; };
	/// make vx5sprite exactly 64 bytes :)
	long okfatim;
}

align(1) struct kfatype
{
	long numspr, numhin, numfrm, seqnum;
	long namoff;
	/// Points to original unconnected KV6 (maybe helpful?)
	align(4) kv6data *basekv6;
	align(4) vx5sprite *spr; 		// [numspr]
	align(4) hingertype *hinge;		// [numhin]
	align(4) long *hingesort;		// [numhin]
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
	long i0, i1;
	/// bounding box, written bt startfalls
	long x0, y0, z0, x1, y1, z1;
	/// mass of piece, written by startfalls (1 unit per voxel)
	long mass;
	/// centroid of piece, written by startfalls
	point3d centroid;

	/**
	*	userval is set to -1 when a new piece is spawned. Voxlap does not
	*	read or write these values after that point. You should use these to
	*	play an initial sound and track velocity
	*/
	long userval, userval2;
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
	char z1, z0, x, y;
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
	long cliphitnum;

	/// Bounding box written by last set* VXL writing call
	long minx, miny, minz, maxx, maxy, maxz;

	/// Falling voxels shared data:
	long flstnum;
	flstboxtype flscnt[FLPIECES];

	/// Total count of solid voxels in .VXL map (included unexposed voxels)
	long globalmass;

	/**
	*	Temp workspace for KFA animation (hinge angles)
	*	Animsprite writes these values and you modify them before drawsprite
	*/
	short kfaval[MAXFRM];

	//------------------------ DATA provided to VOXLAP5 ------------------------

	/// Opticast variables:
	long anginc, sideshademode, mipscandist, maxscandist, vxlminpuse, fogcol;

	/// Drawsprite variables:
	long kv6mipfactor, kv6col;

	/**
	*	Drawsprite x-plane clipping (reset to 0, (high int) after use!)
	*	For example min=8, max=12 permits only planes 8,9,10,11 to draw
	*/
	long xplanemin, xplanemax;

	/// Map modification function data:
	long curcol, currad, curhei;
	float curpow;

	/// Procedural texture function data:
	alias long function(lpoint3d *) colfunc_t;
	colfunc_t colfunc;
	long cen, amount, bpl, xsiz, ysiz, xoru, xorv, picmode;
	long *pic;
	point3d fpico, fpicu, fpicv, fpicw;
	lpoint3d pico, picu, picv;
	float daf;

	/**
	*	Lighting variables: (used by updatelighting)
	*	0 (default), 1:simple lighting, 2:lightsrc lighting
	*/
	long lightmode;
	/// (?,?,?),128*128,262144
	lightsrctype lightsrc[MAXLIGHTS];
	long numlights;

	long fallcheck;
} 
vx5_type vx5;

/// Initialization functions:
extern(C) long initvoxlap();

extern(C) void uninitvoxlap();

/// File related functions:
extern(C) long loadsxl(const char* , char**, char**, char**);
extern(C) char* parspr(vx5sprite*, char**);
extern(C) void loadnul(dpoint3d *, dpoint3d *, dpoint3d *, dpoint3d *);
extern(C) long loaddta(const char*, dpoint3d*, dpoint3d*, dpoint3d*, dpoint3d*);
extern(C) long loadpng(const char*, dpoint3d*, dpoint3d*, dpoint3d*, dpoint3d*);
extern(C) void loadbsp(const char*, dpoint3d*, dpoint3d*, dpoint3d*, dpoint3d*);
extern(C) long loadvxl(const char*, dpoint3d*, dpoint3d*, dpoint3d*, dpoint3d*);
extern(C) long savevxl(const char*, dpoint3d*, dpoint3d*, dpoint3d*, dpoint3d*);
extern(C) long loadsky(const char*);

/// Screen related functions:
extern(C) void voxsetframebuffer(long, long, long, long);
extern(C) void setsideshades(char, char, char, char, char, char);
extern(C) void setcamera(dpoint3d*, dpoint3d*, dpoint3d*, dpoint3d*, float, float, float);
extern(C) void opticast();
extern(C) void drawpoint2d(long, long, long);
extern(C) void drawpoint3d(float, float, float, long);
extern(C) void drawline2d(float, float, float, float, long);
extern(C) void drawline3d(float, float, float, float, float, float, long);
extern(C) long project2d(float, float, float, float*, float*, float*);
extern(C) void drawspherefill(float, float, float, float, long);
extern(C) void drawpicinquad(long, long, long, long, long, long, long, long, float, float, float, float, float, float, float, float);
extern(C) void drawpolyquad (long, long, long, long, float, float, float, float, float, float, float, float, float, float, float, float, float, float, float, float, float, float);
extern(C) void print4x6 (long, long, long, long, const char *, ...);
extern(C) void print6x8 (long, long, long, long, const char *, ...);
extern(C) void drawtile (long, long, long, long, long, long, long, long, long, long, long, long);
extern(C) long screencapture32bit (const char *);
extern(C) long surroundcapture32bit (dpoint3d *, const char *, long);

/// Sprite related functions:
extern(C) kv6data *getkv6 (const char *);
extern(C) kfatype *getkfa (const char *);
extern(C) void freekv6 (kv6data *kv6);
extern(C) void savekv6 (const char *, kv6data *);
extern(C) void getspr (vx5sprite *, const char *);
extern(C) kv6data *genmipkv6 (kv6data *);
extern(C) char *getkfilname (long);
extern(C) void animsprite (vx5sprite *, long);
extern(C) void drawsprite (vx5sprite *);
extern(C) long meltsphere (vx5sprite *, lpoint3d *, long);
extern(C) long meltspans (vx5sprite *, vspans *, long, lpoint3d *);

	//Physics helper functions:
extern(C) void orthonormalize (point3d *, point3d *, point3d *);
extern(C) void dorthonormalize (dpoint3d *, dpoint3d *, dpoint3d *);
extern(C) void orthorotate (float, float, float, point3d *, point3d *, point3d *);
extern(C) void dorthorotate (double, double, double, dpoint3d *, dpoint3d *, dpoint3d *);
extern(C) void axisrotate (point3d *, point3d *, float);
extern(C) void slerp (point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, point3d *, float);
extern(C) long cansee (point3d *, point3d *, lpoint3d *);
extern(C) void hitscan (dpoint3d *, dpoint3d *, lpoint3d *, long **, long *);
extern(C) void sprhitscan (dpoint3d *, dpoint3d *, vx5sprite *, lpoint3d *, kv6voxtype **, float *vsc);
extern(C) double findmaxcr (double, double, double, double);
extern(C) void clipmove (dpoint3d *, dpoint3d *, double);
extern(C) long triscan (point3d *, point3d *, point3d *, point3d *, lpoint3d *);
extern(C) void estnorm (long, long, long, point3d *);

	//VXL reading functions (fast!):
extern(C) long isvoxelsolid (long, long, long);
extern(C) long anyvoxelsolid (long, long, long, long);
extern(C) long anyvoxelempty (long, long, long, long);
extern(C) long getfloorz (long, long, long);
extern(C) long getcube (long, long, long);

	//VXL writing functions (optimized & bug-free):
extern(C) void setcube (long, long, long, long);
extern(C) void setsphere (lpoint3d *, long, long);
extern(C) void setellipsoid (lpoint3d *, lpoint3d *, long, long, long);
extern(C) void setcylinder (lpoint3d *, lpoint3d *, long, long, long);
extern(C) void setrect (lpoint3d *, lpoint3d *, long);
extern(C) void settri (point3d *, point3d *, point3d *, long);
extern(C) void setsector (point3d *, long *, long, float, long, long);
extern(C) void setspans (vspans *, long, lpoint3d *, long);
extern(C) void setheightmap (const ubyte *, long, long, long, long, long, long, long);
extern(C) void setkv6 (vx5sprite *, long);

	//VXL writing functions (slow or buggy):
extern(C) void sethull3d (point3d *, long, long, long);
extern(C) void setlathe (point3d *, long, long, long);
extern(C) void setblobs (point3d *, long, long, long);
extern(C) void setfloodfill3d (long, long, long, long, long, long, long, long, long);
extern(C) void sethollowfill ();
extern(C) void setkvx (const char *, long, long, long, long, long);
extern(C) void setflash (float, float, float, long, long, long);
extern(C) void setnormflash (float, float, float, long, long);

	//VXL MISC functions:
extern(C) void updatebbox (long, long, long, long, long, long, long);
extern(C) void updatevxl ();
extern(C) void genmipvxl (long, long, long, long);
extern(C) void updatelighting (long, long, long, long, long, long);

	//Falling voxels functions:
extern(C) void checkfloatinbox (long, long, long, long, long, long);
extern(C) void startfalls ();
extern(C) void dofall (long);
extern(C) long meltfall (vx5sprite *, long, long);
extern(C) void finishfalls ();

	//Procedural texture functions:
extern(C) long curcolfunc (lpoint3d *);
extern(C) long floorcolfunc (lpoint3d *);
extern(C) long jitcolfunc (lpoint3d *);
extern(C) long manycolfunc (lpoint3d *);
extern(C) long sphcolfunc (lpoint3d *);
extern(C) long woodcolfunc (lpoint3d *);
extern(C) long pngcolfunc (lpoint3d *);
extern(C) long kv6colfunc (lpoint3d *);

	//Editing backup/restore functions
extern(C) void voxbackup (long, long, long, long, long);
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
