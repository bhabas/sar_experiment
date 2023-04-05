#!/bin/bash

# Check if the number of arguments provided is less than 1
if [ $# -lt 1 ]; then
    echo "Please provide the app directory as the first argument."
    exit 1


# Check if the number of arguments provided is 1
elif [ $# -eq 1 ]; then
    SRC_DIR="$1"
    # Run the docker command with only the app directory argument
    docker run --rm -v ${PWD}:/module --device /dev/ttyUSB0 --privileged -P bitcraze/aideck tools/build/make-example ${SRC_DIR} image flash


# Check if the number of arguments provided is 2
elif [ $# -eq 2 ]; then
    SRC_DIR="$1"
    CFLAGS="$2"
    # Run the docker command with both the app directory and compiler flags arguments
    docker run --rm -v ${PWD}:/module bitcraze/aideck tools/build/make-example ${SRC_DIR} "$CFLAGS" flash


# If an invalid number of arguments is provided, display an error message and exit
else
    echo "Invalid number of arguments provided. Please provide the app directory as the first argument and optionally the compiler flags as the second argument."
    exit 1
fi
