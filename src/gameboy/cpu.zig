const std = @import("std");

// use @bitcast to convert a byte to a field and then back again
const Flags = packed struct { _dead_bits: u4, carry: u1, half_carry: u1, subtraction: u1, zero: u1 };

// Operand for an 8 bit instruction
const Operand8 = union {
    r8: Register8,
    im: void,
    addr: void,
};

// NOTE: returns byte at address HL -----------v
const Register8 = enum(u4) { l, h, e, d, c, b, hl, a };
const Register16 = enum(u2) { hl, de, bc, af };

pub const CPU = struct {
    // Assuming DMG startup state  0 1 2 3 4 5 6 7   and   0  1  2  3
    // Registers are indexed as    L H E D C B F A        HL DE BC AF
    registers: [8]u8 = [_]u8{0} ** 8,
    pc: u16 = 0x0100,
    sp: u16 = 0xFFFE,

    // TODO: Rework Memory
    memory: [0xFFFF]u8 = [_]u8{0} ** 0xFFFF,

    // Accessors
    // read a word/byte from memory
    pub fn mem(self: *CPU, T: type, addr: u16) *T {
        return @ptrCast(@alignCast(&self.memory[addr]));
    }

    // get a pointer to register r8
    pub fn fetch_byte(self: *CPU, r8: Register8) *u8 {
        return switch (r8) {
            // read byte at the address pointed to by HL register
            .hl => self.mem(u8, self.fetch_word(.hl).*),
            // otherwise read register
            else => &self.registers[@intFromEnum(r8)],
        };
    }

    // get a pointer to register r16
    pub fn fetch_word(self: *CPU, r16: Register16) *u16 {
        const idx: u3 = @intCast(@intFromEnum(r16));
        return @ptrCast(@alignCast(&self.registers[idx * 2]));
    }
};

const testing = std.testing;

test "register8_intFromEnum" {
    try testing.expectEqual(0, @intFromEnum(CPU.Register8.l));
    try testing.expectEqual(1, @intFromEnum(CPU.Register8.h));
    try testing.expectEqual(2, @intFromEnum(CPU.Register8.e));
    try testing.expectEqual(3, @intFromEnum(CPU.Register8.d));
    try testing.expectEqual(4, @intFromEnum(CPU.Register8.c));
    try testing.expectEqual(5, @intFromEnum(CPU.Register8.b));
    try testing.expectEqual(6, @intFromEnum(CPU.Register8.hl));
    try testing.expectEqual(7, @intFromEnum(CPU.Register8.a));
}

test "register16_intFromEnum" {
    try testing.expectEqual(0, @intFromEnum(CPU.Register16.hl));
    try testing.expectEqual(1, @intFromEnum(CPU.Register16.de));
    try testing.expectEqual(2, @intFromEnum(CPU.Register16.bc));
    try testing.expectEqual(3, @intFromEnum(CPU.Register16.af));
}

test "get_register8" {
    var cpu = CPU{};
    // register[7] = A
    cpu.registers[7] = 0x0A;
    cpu.registers[6] = 0x0F;
    cpu.registers[5] = 0x0B;
    cpu.registers[4] = 0x0C;
    cpu.registers[3] = 0x0D;
    cpu.registers[2] = 0x0E;
    cpu.registers[1] = 0x06;
    cpu.registers[0] = 0x09;

    try testing.expectEqual(0x0A, cpu.fetch_byte(.a).*);
    try testing.expectEqual(0x0B, cpu.fetch_byte(.b).*);
    try testing.expectEqual(0x0C, cpu.fetch_byte(.c).*);
    try testing.expectEqual(0x0D, cpu.fetch_byte(.d).*);
    try testing.expectEqual(0x0E, cpu.fetch_byte(.e).*);
    try testing.expectEqual(0x06, cpu.fetch_byte(.h).*);
    try testing.expectEqual(0x09, cpu.fetch_byte(.l).*);
}

test "get_register16" {
    var cpu = CPU{};
    cpu.registers[7] = 0x0A;
    cpu.registers[6] = 0x0F;
    cpu.registers[5] = 0x0B;
    cpu.registers[4] = 0x0C;
    cpu.registers[3] = 0x0D;
    cpu.registers[2] = 0x0E;
    cpu.registers[1] = 0x06;
    cpu.registers[0] = 0x09;

    try testing.expectEqual(0x0A0F, cpu.fetch_word(.af).*);
    try testing.expectEqual(0x0B0C, cpu.fetch_word(.bc).*);
    try testing.expectEqual(0x0D0E, cpu.fetch_word(.de).*);
    try testing.expectEqual(0x0609, cpu.fetch_word(.hl).*);
}

test "memory" {
    var cpu = CPU{};

    // Little endian says LSB is in the lowest addr
    cpu.memory[0x200] = 0xEF;
    cpu.memory[0x201] = 0xBE;

    cpu.memory[0x202] = 0x69;

    const word = cpu.mem(u16, 0x200);
    const byte = cpu.mem(u8, 0x202);

    try testing.expectEqual(0xBEEF, word.*);
    try testing.expectEqual(0x69, byte.*);
}
