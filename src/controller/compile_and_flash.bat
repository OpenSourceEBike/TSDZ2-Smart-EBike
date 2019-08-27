PATH = %PATH%;C:\Program Files (x86)\STMicroelectronics\st_toolset\stvp

CALL compile.bat

STVP_CmdLine -BoardName=ST-LINK -ProgMode=SWIM -Port=USB -Device=STM8S105x6 -FileProg=main.ihx -verbose -no_loop

CALL clean.bat

PAUSE
EXIT