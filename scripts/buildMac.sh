# Remove all contents in the build folder before building
rm -rf build/*

cmake -S . -B build -DCMAKE_PREFIX_PATH="$(brew --prefix qt)"

cmake --build build -j
