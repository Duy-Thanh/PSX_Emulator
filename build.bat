@ECHO OFF

set CURRENT_DIRECTORY=%CD%

mkdir build
cd build

cmake -G "MinGW Makefiles" ..
make

cd %CURRENT_DIRECTORY%