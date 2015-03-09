@ECHO off
copy ..\crt\*.cer*
root_certificate_downloader -n 4 NMA_Root.cer PROWL_Root.cer RADIUS_Root.cer FreeRADIUS_Root.cer -no_wait -port 0
del   "*.cer"