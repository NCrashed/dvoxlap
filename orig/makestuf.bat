@rem SETUP Visual C 6.0 (SP5) environment for command line
@rem Fix directory name if incorrect!
call "C:\Program Files (x86)\Microsoft Visual Studio 9.0\VC\bin\vcvars32.bat"

@rem SETUP DirectX 8.x environment for command line
@rem Fix directory name if incorrect!
set include=C:\Program Files (x86)\Microsoft DirectX SDK (June 2007)\Include;%include%
set lib=C:\Program Files (x86)\Microsoft DirectX SDK (June 2007)\Lib\x86;%lib%

@rem compile game
nmake game.c

@rem compile simple (must do this after game.c)
nmake simple.c

if exist winmain.obj del winmain.obj

@rem compile voxed
nmake voxed.c

@rem compile kwalk
nmake kwalk.c

pause