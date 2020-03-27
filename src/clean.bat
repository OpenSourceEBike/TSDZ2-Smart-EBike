@ECHO OFF

del /q main.hex >NUL 2>NUL
del /q main.ihx >NUL 2>NUL

del /s /q *.asm >NUL 2>NUL
del /s /q *.rel >NUL 2>NUL
del /s /q *.lk >NUL 2>NUL
del /s /q *.lst >NUL 2>NUL
del /s /q *.rst >NUL 2>NUL
del /s /q *.sym >NUL 2>NUL
del /s /q *.cdb >NUL 2>NUL
del /s /q *.map >NUL 2>NUL
del /s /q *.elf >NUL 2>NUL
del /s /q *.adb >NUL 2>NUL

@ECHO ON