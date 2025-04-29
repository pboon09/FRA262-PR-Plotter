..\nrfutil.exe keys generate private.key
..\nrfutil.exe keys display --key pk --format hex private.key --out_file public.key
..\nrfutil.exe keys display --key pk --format code private.key --out_file public.c