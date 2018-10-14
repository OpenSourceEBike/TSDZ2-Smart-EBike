PATH = %PATH%;C:\SDCC\usr\local\bin;%~dp0..\..\tools\cygwin\bin

make -f Makefile_windows clean

:: pass batch file parameters, e.g. THROTTLE=0
make -f Makefile_windows %*