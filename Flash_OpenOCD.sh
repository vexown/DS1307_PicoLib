#!/bin/bash

echo "Flashing the board using Picoprobe and OpenOCD"

openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "program build/DS1307/DS1307.elf verify reset exit" 

echo "Press 'X' to exit the script."

while true; do
  read -n 1 key  # wait for user to press a key
  if [[ $key == "X" || $key == "x" ]]; then  # check if key pressed is 'X' or 'x'
    echo "Exiting the script."
    exit 0
  fi
done