#!/bin/bash

if [ $# -lt 1 ]; then
    echo "Please provide the app directory as the first argument."
    exit 1
elif [ $# -lt 2 ]; then
    echo "Please provide the compiler flags as the second argument. (clean, all, image)"
    exit 1
fi

SRC_DIR="$1"
shift 1
CFLAGS="$*"

docker run --rm -v ${PWD}:/module bitcraze/aideck tools/build/make-example "$SRC_DIR" "$CFLAGS"
