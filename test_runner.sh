#!/usr/bin/env zsh

# Check arguments
if [[ $# -ne 1 ]]; then
    echo "Usage: $0 <test-number>"
    echo "\nAvailable tests:"
    echo "1: 01-special.gb"
    echo "2: 02-interrupts.gb"
    echo "3: 03-op sp,hl.gb"
    echo "4: 04-op r,imm.gb"
    echo "5: 05-op rp.gb"
    echo "6: 06-ld r,r.gb"
    echo "7: 07-jr,jp,call,ret,rst.gb"
    echo "8: 08-misc instrs.gb"
    echo "9: 09-op r,r.gb"
    echo "10: 10-bit ops.gb"
    echo "11: 11-op a,(hl).gb"
    exit 1
fi

TEST_NUM=$1
DOCTOR_PATH="./gameboy-doctor"
TEST_ROM="tests/cpu_instrs/individual/$(printf "%02d" $TEST_NUM)-"

# Find the matching test ROM
TEST_FILE=$(ls ${TEST_ROM}*.gb 2>/dev/null)

if [[ ! -f $TEST_FILE ]]; then
    echo "Error: Test ROM not found for test number $TEST_NUM"
    exit 1
fi

if [[ ! -f $DOCTOR_PATH/gameboy-doctor ]]; then
    echo "Error: gameboy-doctor not found at $DOCTOR_PATH/gameboy-doctor"
    exit 1
fi

# Run the emulator and pipe through sed to format the output
# then pipe to gameboy-doctor
zig build run -- $TEST_FILE 2>&1 | \
    sed -n 's/^info: \(A:.*\)/\1/p' | \
    python3 $DOCTOR_PATH/gameboy-doctor - cpu_instrs $TEST_NUM
