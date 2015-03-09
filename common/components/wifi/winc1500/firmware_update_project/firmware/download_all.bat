@ECHO off
SET PORT_NUM = 0
SET FIRMWARE_PATH = firmware/m2m_aio.bin
echo Mode %1
if "%1" == "UART"  Goto contine_UART
if "%1" == "I2C"   Goto contine_I2C
if "%1" == "OTA"   Goto contine_OTA
	echo Define the mode (I2C/UART/OTA)
	pause
	exit

:contine_I2C
Set  FMode=debug_i2c
set Mode=I2C
goto START

:contine_OTA
Set  FMode=debug_ota
set Mode=OTA
goto START

:contine_UART
Set  FMode=debug_uart
set Mode=UART
goto START

:START
IF %FMode%==debug_ota (echo Creating Image...
cd Tools\image_builder\debug\
image_builder.exe -OTA_IMG -no_wait
echo OK
goto END
)
cd Tools\image_downloader\%FMode%
echo Downloading Image...
image_downloader.exe -no_wait -port %PORT_NUM% -fw_path %FIRMWARE_PATH%
IF %ERRORLEVEL% NEQ  0 goto END
cd ..\..\gain_builder\%FMode%
echo Downloading Gain Values...
if "%2" == "SAMW25" ( Set GAIN_FILE=samw25_gain_setting.csv
goto DOWNLOAD_GAIN
)
Set GAIN_FILE=samd21_gain_setting.csv
:DOWNLOAD_GAIN
gain_builder.exe -fw_path ../gain_sheets/%GAIN_FILE% -no_wait
:END
IF %ERRORLEVEL% NEQ  0 ( echo Fail
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
)

echo OK
echo     #######################################################################
echo     ##                                                                   ##
echo     ##                 ########     ###     ######   ######              ##
echo     ##                 ##     ##   ## ##   ##    ## ##    ##             ##
echo     ##                 ##     ##  ##   ##  ##       ##                   ##
echo     ##                 ########  ##     ##  ######   ######              ##
echo     ##                 ##        #########       ##       ##             ##
echo     ##                 ##        ##     ## ##    ## ##    ##             ##
echo     ##                 ##        ##     ##  ######   ######              ##
echo     ##                                                                   ##
echo     #######################################################################

echo Downloading ends successfully
pause
