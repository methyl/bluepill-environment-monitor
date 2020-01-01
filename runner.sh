openocd &
PID1=$?
arm-none-eabi-gdb -q -x openocd.gdb "$1"

trap "exit" INT TERM
trap "kill $PID1" EXIT
