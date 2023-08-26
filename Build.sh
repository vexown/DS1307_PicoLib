#!/bin/bash

echo "########## Build.sh - start ##########"
echo "Building all components with CMake"

if [ ! -d "build" ]; then
    mkdir build
    echo "Created build directory"
else
    echo "Build directory already exists"
fi

#configure build directory
echo Configuring the build directory...
cd build
cmake ..

#build the project
echo Building...
cmake --build .

mv DS1307/libDS1307_LIB.a ../libDS1307_LIB.a

echo "Press 'X' to exit the script."

while true; do
  read -n 1 key  # wait for user to press a key
  if [[ $key == "X" || $key == "x" ]]; then  # check if key pressed is 'X' or 'x'
    echo "Exiting the script."
    exit 0
  fi
done


