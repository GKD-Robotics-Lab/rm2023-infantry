#!/bin/sh
cd build
make -j8
cd ..
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg -c "program build/standard_robot.bin 0x08000000 verify exit"
