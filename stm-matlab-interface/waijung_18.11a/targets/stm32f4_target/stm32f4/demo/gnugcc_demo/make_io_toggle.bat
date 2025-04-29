echo off

REM Always run this script file from the folder where the makefile resides.
REM Do not forget to update "WaijungRoot" to Waijung's root directory.

set WaijungRoot=D:\subversion\waijung\trunk\waijung

set PATH=%PATH%; %WaijungRoot%\utils\gnu_tools_arm_embedded\bin

echo on

%WaijungRoot%\utils\gnumake382-w32\gnumake382-w32

REM %WaijungRoot%\utils\gnumake382-w32\gnumake382-w32 clean-obj


