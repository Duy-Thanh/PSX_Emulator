@ECHO OFF

set CURRENT_DIRECTORY=%CD%

mkdir build
cd build

cmake -G "MinGW Makefiles" ..
make

copy /Y deps\SDL2\SDL2.dll .
copy /Y mingw\mingw64\bin\libgcc_s_seh-1.dll .
copy /Y "mingw\mingw64\bin\libstdc++-6.dll" .
copy /Y mingw\mingw64\bin\libwinpthread-1.dll .

cd %CURRENT_DIRECTORY%