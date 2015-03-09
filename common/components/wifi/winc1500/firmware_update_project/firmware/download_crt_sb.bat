@echo off

set TOOL=%1
set MCU=%2
set IMAGE_FILE=%3
set GAIN_MODE=%4
echo %MCU% flashing script: please connect %TOOL% and power up the board.
"C:\Program Files (x86)\Atmel\Atmel Studio 6.2\atbackend\atprogram.exe" -t %TOOL% -i SWD -d %MCU% chiperase
IF %ERRORLEVEL% NEQ 0 ( echo Fail
echo     #######################################################################
echo     ##                                                                   ##
echo     ##                    ########    ###     ####  ##                   ##
echo     ##                    ##         ## ##     ##   ##                   ##
echo     ##                    ##        ##   ##    ##   ##                   ##
echo     ##                    ######   ##     ##   ##   ##                   ##
echo     ##                    ##       #########   ##   ##                   ##
echo     ##                    ##       ##     ##   ##   ##                   ##
echo     ##                    ##       ##     ##  ####  ########             ##
echo     ##                                                                   ##
echo     #######################################################################
pause
exit
)
"C:\Program Files (x86)\Atmel\Atmel Studio 6.2\atbackend\atprogram.exe" -t %TOOL% -i SWD -d %MCU% program -f %IMAGE_FILE%
IF %ERRORLEVEL% NEQ 0 ( echo Fail
echo     #######################################################################
echo     ##                                                                   ##
echo     ##                    ########    ###     ####  ##                   ##
echo     ##                    ##         ## ##     ##   ##                   ##
echo     ##                    ##        ##   ##    ##   ##                   ##
echo     ##                    ######   ##     ##   ##   ##                   ##
echo     ##                    ##       #########   ##   ##                   ##
echo     ##                    ##       ##     ##   ##   ##                   ##
echo     ##                    ##       ##     ##  ####  ########             ##
echo     ##                                                                   ##
echo     #######################################################################
pause
exit
)

echo Please wait...
ping 192.0.0.1 -w 500 > NUL

cd Tools\root_certificate_downloader\debug_uart
RootCertDownload.bat
