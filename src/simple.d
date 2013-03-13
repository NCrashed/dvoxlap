/*
To compile, first compile game.c to generate the OBJ files, then type: "nmake simple.c"
simple.exe : simple.obj voxlap5.obj v5.obj kplib.obj winmain.obj
	link simple voxlap5 v5 kplib winmain ddraw.lib dinput.lib ole32.lib dxguid.lib user32.lib gdi32.lib /opt:nowin98
simple.obj : simple.c voxlap5.h sysmain.h
	cl /c /J /TP simple.c /Ox /Ob2 /G6Fy /Gs /MD /QIfist
*/

import sysmain;
import voxlap5;

dpoint3d ipos, istr, ihei, ifor;

//vx5sprite desklamp;

long initapp(long argc, char **argv)
{
	xres = 1920; yres = 1080; colbits = 32; fullscreen = 0;
	initvoxlap();
	kzaddstack("voxdata.zip");
	loadvxl("vxl/untitled.vxl", &ipos, &istr, &ihei, &ifor);

	/*desklamp.voxnum = getkv6("kv6\\desklamp.kv6");
	desklamp.p.x = 652; desklamp.p.y = 620; desklamp.p.z = 103.5;
	desklamp.s.x = 0.4; desklamp.s.y = 0;   desklamp.s.z = 0;
	desklamp.h.x = 0;   desklamp.h.y = 0.4; desklamp.h.z = 0;
	desklamp.f.x = 0;	desklamp.f.y = 0;	desklamp.f.z = 0.4;
	desklamp.kfatim = 0; desklamp.okfatim = 0; desklamp.flags = 0;*/

	return 0;
}

void doframe()
{
	long frameptr, pitch, xdim, ydim;
	startdirectdraw(&frameptr, &pitch, &xdim, &ydim);
	voxsetframebuffer(frameptr, pitch, xdim, ydim);
	setcamera(&ipos, &istr, &ihei, &ifor, xdim*0.5, ydim*0.5, xdim*0.5);
	opticast();

	//drawsprite(&desklamp);

	stopdirectdraw();
	nextpage();
	readkeyboard(); if (keystatus[1]) quitloop();
}

void uninitapp()
{
	//freekv6(desklamp.voxnum);
	uninitvoxlap();
	kzuninit();
}