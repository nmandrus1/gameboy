const std = @import("std");
const opcodes = @import("opcodes.zig");
const Load = opcodes.Load;
const LoadOperand = opcodes.LoadOperand;
const Add = opcodes.Add;
const AddOperand = opcodes.AddOperand;

pub const CPUError = error{
    InvalidRegisterName,
    InvalidOperand,
};

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

    // use @bitcast to convert a byte to a field and then back again
    const Flags = packed struct { _dead_bits: u4 = 0, carry: u1 = 0, half_carry: u1 = 0, subtraction: u1 = 0, zero: u1 = 0 };
    const Flag = union(enum) { carry: u1, half_carry: u1, subtraction: u1, zero: u1 };

    // Assuming DMG startup state  0 1 2 3 4 5 6 7   and   0  1  2  3  4
    // Registers are indexed as    L H E D C B F A        HL DE BC AF SP
    registers: [8]u8 = [_]u8{0} ** 8,
    pc: u16 = 0x0100,
    sp: u16 = 0xFFFE,

    // TODO: Rework Memory
    memory: [0xFFFF]u8 = [_]u8{0} ** 0xFFFF,

    state: State = .Running,

    /// Set the CPU Flag register to the passed value
    pub fn set_flag(self: *CPU, flag: Flag) void {
        var f: *Flags = @ptrCast(&self.registers[6]);
        switch (flag) {
            .carry => |bit| f.carry = bit,
            .half_carry => |bit| f.half_carry = bit,
            .subtraction => |bit| f.subtraction = bit,
            .zero => |bit| f.zero = bit,
        }
    }

    pub fn get_flag_state(self: *CPU) *Flags {
        return @ptrCast(&self.registers[6]);
    }

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
        comptime if (@typeInfo(T) != .Int) @compileError("Invalid type -- expecting integer");
        comptime if (@typeInfo(T).bits > 16) @compileError("Invalid type -- 8 or 16 bit integer");

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

    // ########################################
    // ###      INSTRUCTION EXECUTION       ###
    // ########################################

    // push a value to the stack
    pub fn stackPush(self: *CPU, r16: Register16) void {
        self.sp -= 2;
        // push is just a load operation
        self.load(u16, &self.memory[self.sp .. self.sp + 2], self.get_r16(r16).*);
    }

    // pop a value from the stack
    pub fn stackPop(self: *CPU, r16: Register16) void {
        defer self.sp += 2;
        // pop is just a load operation
        self.load(u16, &self.sp, self.read(u16, self.get_r16(r16).*));
    }

    pub fn executeLoad(self: *CPU, op: Load) usize {
        switch (op.src) {
            // 8 bit load
            .r8, .imm8, .address => {
                const src = self.fetchLoadData(u8, op.src);
                const dest = switch (op.dest) {
                    .r8 => |r8| self.get_r8(r8),
                    .address => |addr| self.read(u8, self.address_ptr(addr)),
                    else => unreachable,
                };
                CPU.load(u8, dest, src);
            },
            // 16 bit load
            .r16, .imm16, .sp_offset => {
                const src = self.fetchLoadData(u16, op.src);
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
    pub fn fetchLoadData(self: *CPU, comptime T: type, operand: LoadOperand) T {
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
                    self.add_sp_e8();
                    break :blk self.sp;
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

    pub fn executeAdd(self: *CPU, add: Add) usize {
        const rhs = switch (add.src) {
            // 8 bit add
            .r8 => |r8| self.get_r8(r8).*,
            .imm8 => self.pc_read(u8),
            .hl_addr => self.read(u8, self.get_r16(.hl).*),
            .r16 => |r16| {
                self.genericAdd(u16, self.get_r16(.hl), self.get_r16(r16).*);
                return add.cycles;
            },
            .e8 => { // go ahead and execute special function
                self.add_sp_e8();
                return add.cycles;
            },
        };

        // if we are here then its an 8 bit load
        if (add.with_carry) self.adc(rhs) else self.genericAdd(u8, self.get_r8(.a), rhs);
        return add.cycles;
    }

    /// For most operations, the Zero, Carry, and Half carry flags are set, and
    /// this function automatically does that, for more control over flag state call
    /// the regular CPU.add() function
    pub fn genericAdd(self: *CPU, comptime T: type, dest: *T, src: anytype) void {
        if (T == u8) self.get_flag_state().* = CPU.add(T, dest, src) else if (T == u16) {
            // 16 bit add functions leave zero flag untouched, so the state is saved and rewritten
            const zero = self.get_flag_state().zero;
            self.get_flag_state().* = CPU.add(T, dest, src);
            self.set_flag(.{ .zero = zero });
        }
    }

    /// Adds the sum of dest and rhs to the memory location pointed to by dest
    /// return the Zero, Carry, Half-Carry, and Substraction flags, and it is up
    /// to the caller to ensure that the proper flags based on the return value
    pub fn add_inner(comptime T: type, dest: *T, rhs: T) Flags {
        const lhs = dest.*;

        const int_type = @Type(.{ .Int = .{ .signedness = .unsigned, .bits = @typeInfo(T).Int.bits - 4 } });

        const lhs_lower: int_type = @truncate(lhs);
        const rhs_lower: int_type = @truncate(rhs);

        const half_carry = @addWithOverflow(lhs_lower, rhs_lower).@"1";

        const result = @addWithOverflow(lhs, rhs);
        const zero: u1 = if (result.@"0" == 0) 1 else 0;
        return Flags{ .carry = result.@"1", .zero = zero, .half_carry = half_carry, .subtraction = 0 };
    }

    /// Function to handle specific instructions where immediate data is added to SP
    pub fn add_sp_e8(self: *CPU) void {
        const lsb: u8 = @truncate(self.sp >> 8);
        const e8 = self.pc_read(u8);
        const flags = CPU.add(u8, &lsb, e8);
        self.set_flag(.{ .carry = flags.carry });
        self.set_flag(.{ .half_carry = flags.half_carry });
        self.sp += e8;
    }

    /// Add to the A register, the carry flag, and the associated data
    pub fn adc(self: *CPU, rhs: u8) void {
        const partial = CPU.add(u8, self.get_r8(.a), rhs);
        const partial2 = CPU.add(u8, self.get_r8(.a), @as(u8, self.get_flag_state().carry));
        var flags = self.get_flag_state();
        flags.zero = partial.zero | partial2.zero;
        flags.carry = partial.carry | partial2.carry;
        flags.half_carry = partial.half_carry | partial2.half_carry;
        flags.subtraction = 0;
    }

    pub fn inc(comptime T: type, dest: *T) void {
        dest.* += 1;
    }
    pub fn dec(comptime T: type, dest: *T) void {
        dest.* -= 1;
    }
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

test "flag register" {
    var cpu = CPU{};
    cpu.get_flag_state().zero = 1;

    const actual: u1 = @truncate((cpu.registers[6]) >> 7);
    try testing.expectEqual(1, actual);
}
