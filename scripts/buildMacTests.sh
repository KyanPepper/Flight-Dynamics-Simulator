# Remove all contents in the build folder before building
rm -rf build/*

# Configure the project with CMake, specifying the Qt prefix path
cmake -S . -B build -DBUILD_TESTS=ON 


# Build the project
cmake --build build -j

