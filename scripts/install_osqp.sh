#!/bin/bash
set -e

# Install osqp
cd /project/packages/osqp
mkdir build && cd build
cmake .. -DBUILD_SHARED_LIBS=ON
make -j$(nproc)
make install

# clean up
cd /project/packages/osqp
rm -rf build

# Install osqp-eigen
cd /project/packages/osqp-eigen
mkdir build && cd build
cmake ..
make -j$(nproc)
make install

# clean up
cd /project/packages/osqp-eigen
rm -rf build
