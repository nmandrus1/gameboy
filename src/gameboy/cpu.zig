const std = @import("std");
const opcodes = @import("opcodes.zig");
const Load = opcodes.Load;
const LoadOperand = opcodes.LoadOperand;

pub const CPUError = error{
    InvalidRegisterName,
    InvalidOperand,
};

// use @bitcast to convert a byte to a field and then back again
const Flags = packed struct { _dead_bits: u4, carry: u1, half_carry: u1, subtraction: u1, zero: u1 };

// Operand for an 8 bit instruction
pub const Register8 = enum(u4) {
    l,
    h,
    e,
    d,
    c,
    b,
    // NOTE: returns byte at address HL
    hl,
    a,

    pub fn from_str(str: []const u8) CPUError!Register8 {
        return std.meta.stringToEnum(Register8, str) orelse return CPUError.InvalidRegisterName;
    }
};

pub const Register16 = enum(u3) {
    hl,
    de,
    bc,
    af,
    sp,

    pub fn from_str(str: []const u8) CPUError!Register16 {
        return std.meta.stringToEnum(Register16, str) orelse return CPUError.InvalidRegisterName;
    }
};

pub const CPU = struct {
    const State = enum {
        Running,
    };

    // Assuming DMG startup state  0 1 2 3 4 5 6 7   and   0  1  2  3  4
    // Registers are indexed as    L H E D C B F A        HL DE BC AF SP
    registers: [8]u8 = [_]u8{0} ** 8,
    pc: u16 = 0x0100,
    sp: u16 = 0xFFFE,

    // TODO: Rework Memory
    memory: [0xFFFF]u8 = [_]u8{0} ** 0xFFFF,

    state: State = .Running,

    // Accessors
    // read a word/byte from memory
    pub fn read(self: *CPU, comptime T: type, addr: u16) *T {
        return @ptrCast(@alignCast(&self.memory[addr]));
    }

    /// Given an address struct this returns a u16 pointer to
    /// somewhere in memory, use the read() function to read
    /// a byte/word from that address
    pub fn address_ptr(self: *CPU, addr: opcodes.Address) u16 {
        return switch (addr) {
            .bc, .de, .hl => ret: {
                const r16 = Register16.from_str(@tagName(addr)) catch unreachable;
                break :ret self.get_r16(r16).*;
            },
            .hli => ret: {
                const val = self.get_r16(.hl).*;
                self.get_r16(.hl).* += 1;
                break :ret val;
            },
            .hld => ret: {
                const val = self.get_r16(.hl).*;
                self.get_r16(.hl).* -= 1;
                break :ret val;
            },
            .a16 => self.pc_read(u16),
            .c => 0xFF00 + @as(u16, @intCast(self.get_r8(.c).*)),
            .a8 => 0xFF00 + @as(u16, @intCast(self.pc_read(u8))),
        };
    }

    // get a pointer to register r8
    pub fn get_r8(self: *CPU, r8: Register8) *u8 {
        return switch (r8) {
            // read byte at the address pointed to by HL register
            .hl => self.read(u8, self.get_r16(.hl).*),
            // otherwise read register
            else => &self.registers[@intFromEnum(r8)],
        };
    }

    // get a pointer to register r16
    pub fn get_r16(self: *CPU, r16: Register16) *u16 {
        switch (r16) {
            .sp => return &self.sp,
            else => {
                const idx: u3 = @intCast(@intFromEnum(r16));
                return @ptrCast(@alignCast(&self.registers[idx * 2]));
            },
        }
    }

    // TODO: generic register funciton??
    // pub fn register(self: *CPU, comptime T: type, register: anytype) *T {
    //     comptime if (!(T == u8 or T == u16)) @compileError("Invalid type -- expecting u8 or u16");
    //     switch (T) {
    //         u8 => {
    //             comptime info = @typeInfo(@TypeOf(register)).Enum.;
    //         }
    //     }
    // }

    // read a byte or word from the program counter
    // TODO: test this
    pub fn pc_read(self: *CPU, comptime T: type) T {
        comptime if (!(T == u8 or T == u16)) @compileError("Invalid type -- expecting u8 or u16");

        defer self.pc += @divExact(@typeInfo(T).Int.bits, 8);
        return std.mem.readInt(T, @ptrCast(&self.memory[self.pc]), .little);
    }

    // step the CPU forward by one instruction
    pub fn step(self: *CPU) usize {
        switch (self.state) {
            .Running => {
                _ = self.memory[self.pc];
            },
        }
    }

    // push a value to the stack
    pub fn stack_push(self: *CPU, r16: Register16) void {
        self.sp -= 2;
        // push is just a load operation
        self.load(u16, &self.memory[self.sp .. self.sp + 2], self.get_r16(r16).*);
    }

    // pop a value from the stack
    pub fn stack_pop(self: *CPU, r16: Register16) void {
        defer self.sp += 2;
        // pop is just a load operation
        self.load(u16, &self.sp, self.read(u16, self.get_r16(r16).*));
    }

    pub fn ld(self: *CPU, op: Load) usize {
        switch (op.src) {
            // 8 bit load
            .r8, .imm8, .address => {
                const src = self.fetch_data(u8, op.src);
                const dest = switch (op.dest) {
                    .r8 => |r8| self.get_r8(r8),
                    .address => |addr| self.read(u8, self.address_ptr(addr)),
                    else => unreachable,
                };
                CPU.load(u8, dest, src);
            },
            // 16 bit load
            .r16, .imm16, .sp_offset => {
                const src = self.fetch_data(u16, op.src);
                const dest = switch (op.dest) {
                    .r16 => |r16| self.get_r16(r16),
                    .address => |addr| self.read(u16, self.address_ptr(addr)),
                    else => unreachable,
                };
                CPU.load(u16, dest, src);
            },
        }

        return op.cycles;
    }

    // fetch data for a load operand, can be a word or byte
    pub fn fetch_data(self: *CPU, comptime T: type, operand: LoadOperand) T {
        comptime if (!(T == u8 or T == u16)) @compileError("Invalid Type -- expected u8 or u16");
        return switch (T) {
            u8 => switch (operand) {
                .r8 => |r8| self.get_r8(r8).*,
                // TODO: find a way to better express the posibilty of an addr pointing to a u16
                .address => |addr| self.read(u8, self.address_ptr(addr)).*,
                .imm8 => self.pc_read(u8),
                else => unreachable,
            },
            u16 => switch (operand) {
                .r16 => |r16| self.get_r16(r16).*,
                .imm16 => self.pc_read(u16),
                .sp_offset => blk: {
                    std.log.warn("CPU Flags are NOT handled for this instruction", .{});
                    break :blk @as(u16, self.pc_read(u8)) + self.sp;
                },
                else => unreachable,
            },
            else => unreachable,
        };
    }

    // Simple load funciton, ultimately any load operation boils down to this
    // my goal is to have any "load" instruction eventually call this
    pub fn load(comptime T: type, dest: *T, src: T) void {
        comptime if (!(T == u8 or T == u16)) @compileError("Invalid type -- expected u8 or u16");
        std.mem.writeInt(T, @ptrCast(@alignCast(dest)), src, .little);
    }

    // pub fn load(comptime T: type, dest: *[@divExact(@typeInfo(T).Int.bits, 8)]u8, src: T) void {
    //     comptime if (!(T == u8 or T == u16)) @compileError("Invalid type -- expected u8 or u16");
    //     std.mem.writeInt(T, dest, src, .little);
    // }
};

const testing = std.testing;

test "register8_intFromEnum" {
    try testing.expectEqual(0, @intFromEnum(Register8.l));
    try testing.expectEqual(1, @intFromEnum(Register8.h));
    try testing.expectEqual(2, @intFromEnum(Register8.e));
    try testing.expectEqual(3, @intFromEnum(Register8.d));
    try testing.expectEqual(4, @intFromEnum(Register8.c));
    try testing.expectEqual(5, @intFromEnum(Register8.b));
    try testing.expectEqual(6, @intFromEnum(Register8.hl));
    try testing.expectEqual(7, @intFromEnum(Register8.a));
}

test "register16_intFromEnum" {
    try testing.expectEqual(0, @intFromEnum(Register16.hl));
    try testing.expectEqual(1, @intFromEnum(Register16.de));
    try testing.expectEqual(2, @intFromEnum(Register16.bc));
    try testing.expectEqual(3, @intFromEnum(Register16.af));
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

    try testing.expectEqual(0x0A, cpu.get_r8(.a).*);
    try testing.expectEqual(0x0B, cpu.get_r8(.b).*);
    try testing.expectEqual(0x0C, cpu.get_r8(.c).*);
    try testing.expectEqual(0x0D, cpu.get_r8(.d).*);
    try testing.expectEqual(0x0E, cpu.get_r8(.e).*);
    try testing.expectEqual(0x06, cpu.get_r8(.h).*);
    try testing.expectEqual(0x09, cpu.get_r8(.l).*);
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

    try testing.expectEqual(0x0A0F, cpu.get_r16(.af).*);
    try testing.expectEqual(0x0B0C, cpu.get_r16(.bc).*);
    try testing.expectEqual(0x0D0E, cpu.get_r16(.de).*);
    try testing.expectEqual(0x0609, cpu.get_r16(.hl).*);
}

test "memory" {
    var cpu = CPU{};

    // Little endian says LSB is in the lowest addr
    cpu.memory[0x200] = 0xEF;
    cpu.memory[0x201] = 0xBE;

    cpu.memory[0x202] = 0x69;

    const word = cpu.read(u16, 0x200);
    const byte = cpu.read(u8, 0x202);

    try testing.expectEqual(0xBEEF, word.*);
    try testing.expectEqual(0x69, byte.*);
}

test "parse register8" {
    const r8 = try Register8.from_str("a");
    try testing.expectEqual(r8, Register8.a);

    var err = Register8.from_str("A");
    try testing.expectError(CPUError.InvalidRegisterName, err);

    err = Register8.from_str("Z");
    try testing.expectError(CPUError.InvalidRegisterName, err);
}

test "basic load8" {
    var cpu = CPU{};
    cpu.memory[cpu.pc] = 0x42;
    const old_pc = cpu.pc;

    const load_a_imm8 = Load{ .dest = .{ .r8 = .a }, .src = .{ .imm8 = {} }, .bytes = 2, .cycles = 2 };

    const cycles = cpu.ld(load_a_imm8);

    try testing.expectEqual(cpu.get_r8(.a).*, 0x42);
    try testing.expectEqual(cycles, 2);
    // only + 1 since the opcode is included in the instruction byte count
    try testing.expectEqual(old_pc + 1, cpu.pc);
}
