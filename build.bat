REM git checkout --recurse-submodules
REM git submodule update --init --recursive
cmake -B build .
cmake --build ./build
