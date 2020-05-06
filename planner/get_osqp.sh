#!/bin/bash

cd ../..

git clone --recursive https://github.com/oxfordcontrol/osqp

cd osqp/
mkdir build
cd build/

cmake -G "Unix Makefiles" ..

cmake --build .

cmake --build . --target install

cd ../..

git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen

mkdir build && cd build/
cmake ../
make

make install
