const std = @import("std");

// use @bitcast to convert a byte to a field and then back again
const Flags = packed struct { _dead_bits: u4, carry: u1, half_carry: u1, subtraction: u1, zero: u1 };

pub const CPU = struct {
    // Assuming DMG startup state  7 6 5 4 3 2 1 0   and  3  2  1  0
    // Registers are indexed as    L H E D C B F A        LH ED CB FA
    registers: [8]u8 = [_]u8{0} ** 8,

    pc: u16 = 0x0100,
    sp: u16 = 0xFFFE,

    // Accessors

    // get a pointer to register r8
    pub fn register8(self: *CPU, r8: u4) *u8 {
        return &self.registers[r8];
    }

    // get a pointer to register r16
    pub fn register16(self: *CPU, r16: u2) *u16 {
        return @ptrCast(@alignCast(&self.registers[r16 * 2]));
    }
};

const testing = std.testing;

test "get_register8s" {
    var cpu = CPU{};
    cpu.registers[0] = 0x69;

    try testing.expectEqual(0x69, cpu.register8(0).*);
}

test "get_register16s" {
    var cpu = CPU{};
    cpu.registers[0] = 0x69;
    cpu.registers[1] = 0x96;

    try testing.expectEqual(0x6996, cpu.register16(0).*);
}
