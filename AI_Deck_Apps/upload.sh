#!/bin/bash

if [ $# -lt 1 ]; then
    echo "Please provide the app to build as the first argument."
    exit 1
fi

cfloader flash $1/BUILD/GAP8_V2/GCC_RISCV_FREERTOS/target.board.devices.flash.img deck-bcAI:gap8-fw -w radio://0/80/2M/E7E7E7E701
