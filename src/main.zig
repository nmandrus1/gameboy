const std = @import("std");
const gameboy = @import("gameboy");

// Launch the emulator
pub fn main() !void {
    try gameboy.run();
}
