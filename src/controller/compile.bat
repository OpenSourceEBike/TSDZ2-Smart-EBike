PATH = %PATH%;C:\SDCC\usr\local\bin;%~dp0..\..\tools\cygwin\bin

make -f Makefile_windows clean
make -f Makefile_windows

PAUSE