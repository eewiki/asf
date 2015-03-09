@ECHO off
@TITLE	Root Certificate Downloader - Atmel Corporation
copy ..\crt\*.cer*
root_certificate_downloader -n 4 FreeRADIUS_Root.cer NMA_Root.cer PROWL_Root.cer RADIUS_Root.cer -no_wait -port 0
del   "*.cer"