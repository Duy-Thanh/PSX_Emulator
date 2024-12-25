#!/bin/bash

export CURRENT_DIRECTORY=$PWD

mkdir build
cd build

cmake ..
make

cd $CURRENT_DIRECTORY