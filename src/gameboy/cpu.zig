const std = @import("std");
const opcodes = @import("opcodes.zig");
const Load = opcodes.Load;
const LoadOperand = opcodes.LoadOperand;
const StackOp = opcodes.StackOperation;
const Add = opcodes.Add;
const AddOperand = opcodes.AddOperand;
const Increment = opcodes.Increment;
const Decrement = opcodes.Decrement;
const ByteArithmetic = opcodes.ByteArithmetic;
const Jump = opcodes.Jump;
const Call = opcodes.Call;
const Return = opcodes.Return;
const Instruction = opcodes.Instruction;

pub const CPUError = error{
    InvalidRegisterName,
    InvalidOperand,
};

// Operand for an 8 bit instruction
pub const Register8 = enum(u4) {
    // a,
    // hl,
    // b,
    // c,
    // d,
    // e,
    // h,
    // l,
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

    // af,
    // bc,
    // de,
    // hl,
    // sp,

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
    cycles: usize = 0,

    table: ?opcodes.UnprefixedOpcodes = null,

    pub fn init(allocator: std.mem.Allocator) !CPU {
        return CPU{
            .table = try opcodes.UnprefixedOpcodes.init(allocator),
        };
    }

    pub fn loadROM(self: *CPU, rom: []const u8) void {
        @memcpy(&self.memory, rom[0..self.memory.len]);
    }

    pub fn deinit(self: *CPU) void {
        self.table.?.deinit();
    }

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

    pub fn flag_state(self: *CPU) *Flags {
        return @ptrCast(&self.registers[6]);
    }

    // Accessors
    // read a word/byte from memory
    pub fn mem_read(self: *CPU, comptime T: type, addr: u16) *T {
        return switch (T) {
            u8 => &self.memory[addr],
            u16 => @ptrCast(@alignCast(&self.memory[addr])),
            else => @compileError("Unsupported type for read"),
        };
    }

    // Simple load funciton, ultimately any load operation boils down to this
    // my goal is to have any "load" instruction eventually call this
    pub fn write(comptime T: type, dest: *T, src: T) void {
        switch (T) {
            u8 => dest.* = src,
            u16 => std.mem.writeInt(u16, @ptrCast(@alignCast(dest)), src, .little),
            else => @compileError("Invliad type for write"),
        }
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

    /// Get a pointer to the value stored in the 8 bit register
    /// if the variaant is .hl, get a pointer to the value in memory
    /// pointed to by the HL register
    pub fn get_r8(self: *CPU, r8: Register8) *u8 {
        return switch (r8) {
            // read byte at the address pointed to by HL register
            .hl => self.mem_read(u8, self.get_r16(.hl).*),
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
        comptime if (@typeInfo(T).Int.bits > 16) @compileError("Invalid type -- 8 or 16 bit integer");

        defer self.pc += @divExact(@typeInfo(T).Int.bits, 8);
        // return std.mem.readInt(T, @ptrCast(&self.memory[self.pc]), .little);
        return self.mem_read(T, self.pc).*;
    }

    // step the CPU forward by one instruction
    pub fn step(self: *CPU) usize {
        switch (self.state) {
            .Running => {
                const opcode = self.pc_read(u8);
                const instr = self.decode(opcode);
                return self.execute(instr);
            },
        }
    }

    pub fn decode(self: CPU, opcode: u8) Instruction {
        // TODO: make opcode table better
        return Instruction.from_json_opcode(self.table.?.table[opcode]) catch |err| {
            std.log.err("Error getting instruction from JsonOpcode - {!}", .{err});
            return Instruction{ .Nop = {} };
        };
    }

    pub fn execute(self: *CPU, instr: Instruction) usize {
        return switch (instr) {
            .Nop => 4,
            .Load => |ld| self.executeLoad(ld),
            // Push/
            .StackOp => |stack_op| self.executeStackOp(stack_op),
            // ADD/
            .Add => |add_instr| self.executeAdd(add_instr),
            .Inc => |inc_instr| self.executeInc(inc_instr),
            .Dec => |dec_instr| self.executeDec(dec_instr),
            .ALUOp => |byte_op| self.executeALUOp(byte_op),
            .Jump => |jmp| self.executeJump(jmp),
            .Call => |call_fn| self.executeCall(call_fn),
            .Return => |ret| self.executeReturn(ret),
        };
    }

    // ########################################
    // ###      INSTRUCTION EXECUTION       ###
    // ########################################

    // push a value to the stack
    pub fn stackPush(self: *CPU, value: u16) void {
        self.sp -= 2;
        // push is just a load operation
        // convince zig that this pointer to a byte is a pointer to a u16
        CPU.write(u16, @ptrCast(@alignCast(&self.memory[self.sp])), value);
    }

    // pop a value from the stack
    pub fn stackPop(self: *CPU, ptr: *u16) void {
        defer self.sp += 2;
        // pop is just a load operation
        CPU.write(u16, ptr, std.mem.readInt(u16, @ptrCast(self.memory[self.sp .. self.sp + 2]), .little));
    }

    pub fn executeStackOp(self: *CPU, op: StackOp) usize {
        switch (op.op) {
            .push => |r16| self.stackPush(self.get_r16(r16).*),
            .pop => |r16| self.stackPop(self.get_r16(r16)),
        }
        return op.cycles;
    }

    pub fn executeLoad(self: *CPU, op: Load) usize {
        switch (op.src) {
            // 8 bit load
            .r8, .imm8, .address => {
                const src = self.fetchLoadData(u8, op.src);
                const dest = switch (op.dest) {
                    .r8 => |r8| self.get_r8(r8),
                    .address => |addr| self.mem_read(u8, self.address_ptr(addr)),
                    else => unreachable,
                };
                CPU.write(u8, dest, src);
            },
            // 16 bit load
            .r16, .imm16, .sp_offset => {
                const src = self.fetchLoadData(u16, op.src);
                const dest = switch (op.dest) {
                    .r16 => |r16| self.get_r16(r16),
                    .address => |addr| self.mem_read(u16, self.address_ptr(addr)),
                    else => unreachable,
                };
                CPU.write(u16, dest, src);
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
                .address => |addr| self.mem_read(u8, self.address_ptr(addr)).*,
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

    pub fn executeAdd(self: *CPU, add_instr: Add) usize {
        const rhs = switch (add_instr.src) {
            // 8 bit add
            .r8 => |r8| self.get_r8(r8).*,
            .imm8 => self.pc_read(u8),
            .r16 => |r16| {
                self.genericAdd(u16, self.get_r16(.hl), self.get_r16(r16).*);
                return add_instr.cycles;
            },
            .e8 => { // go ahead and execute special function
                self.add_sp_e8();
                return add_instr.cycles;
            },
        };

        // if we are here then its an 8 bit load
        if (add_instr.with_carry) self.adc(rhs) else self.genericAdd(u8, self.get_r8(.a), rhs);
        return add_instr.cycles;
    }

    // Handles all 8 bit ALU Operations except for Adding
    fn executeALUOp(self: *CPU, instr: ByteArithmetic) usize {
        const rhs = switch (instr.operation) {
            .SUB,
            .SBC,
            .AND,
            .OR,
            .XOR,
            .CP,
            => |operand| switch (operand) {
                .r8 => |r8| self.get_r8(r8).*,
                .imm8 => self.pc_read(u8),
            },
            else => undefined,
        };

        self.flag_state().* = switch (instr.operation) {
            .SUB => CPU.sub(self.get_r8(.a), rhs),
            .SBC => self.sbc(rhs),
            .AND => self.acc_and(rhs),
            .OR => self.acc_or(rhs),
            .XOR => self.acc_xor(rhs),
            .CP => blk: {
                //Subtracts from the 8-bit A register, the 8-bit register r, and updates flags based on the result.
                // This instruction is basically identical to SUB r, but does not update the A register.
                // store and write back the oringal A register value
                const a = self.get_r8(.a).*;
                defer self.get_r8(.a).* = a;
                break :blk CPU.sub(self.get_r8(.a), rhs);
            },

            // toggles carry, sets HC, and N to 0, doesnt touch Z
            .CCF => .{ .carry = ~self.flag_state().carry, .zero = self.flag_state().zero },
            .SCF => .{ .carry = 1, .zero = self.flag_state().zero },
            .DAA => blk: {
                std.log.warn("DAA Instruction not Implemented!", .{});
                break :blk self.flag_state().*;
            },
            .CPL => blk: {
                self.get_r8(.a).* = ~self.get_r8(.a).*;
                self.set_flag(.{ .subtraction = 1 });
                self.set_flag(.{ .half_carry = 1 });
                break :blk self.flag_state().*;
            },
        };

        return instr.cycles;
    }

    fn executeInc(self: *CPU, inc_instr: Increment) usize {
        switch (inc_instr.dest) {
            .r8 => |r8| self.inc(u8, self.get_r8(r8)),
            .r16 => |r16| self.inc(u16, self.get_r16(r16)),
        }

        return inc_instr.cycles;
    }

    fn executeDec(self: *CPU, dec_instr: Decrement) usize {
        switch (dec_instr.dest) {
            .r8 => |r8| self.dec(u8, self.get_r8(r8)),
            .r16 => |r16| self.dec(u16, self.get_r16(r16)),
        }

        return dec_instr.cycles;
    }

    fn jump(self: *CPU, addr: u16) void {
        self.pc = addr;
    }

    fn check_condition(cc: Jump.Condition, flags: Flags) bool {
        return switch (cc) {
            .unconditional => true,
            .nz => flags.zero == 0,
            .z => flags.zero == 1,
            .nc => flags.carry == 0,
            .c => flags.carry == 1,
        };
    }

    fn executeJump(self: *CPU, jmp: Jump) usize {
        const result: struct { addr: u16, cc: Jump.Condition } = switch (jmp.operand) {
            .hl => |cc| .{ .addr = self.get_r16(.hl).*, .cc = cc },
            .a16 => |cc| .{ .addr = self.pc_read(u16), .cc = cc },
            .e8 => |cc| .{ .addr = @as(u16, self.pc_read(u8)) + self.pc, .cc = cc },
        };

        // if the condition is met, then flow through the switch statement,
        // otherwise return the relevant cycle count
        if (CPU.check_condition(result.cc, self.flag_state().*)) {
            self.jump(result.addr);
            return jmp.cycles[0];
        } else return jmp.cycles[1];
    }

    fn call(self: *CPU, addr: u16) void {
        self.stackPush(self.pc);
        self.jump(addr);
    }

    fn executeCall(self: *CPU, fn_call: Call) usize {
        const addr = self.pc_read(u16);

        // if the condition is met, then flow through the switch statement,
        // otherwise return the relevant cycle count
        if (CPU.check_condition(fn_call.condition, self.flag_state().*)) {
            self.call(addr);
            return fn_call.cycles[0];
        } else return fn_call.cycles[1];
    }

    fn executeReturn(self: *CPU, fn_ret: Return) usize {
        // if the condition is met, then flow through the switch statement,
        // otherwise return the relevant cycle count
        if (CPU.check_condition(fn_ret.condition, self.flag_state().*)) {
            self.stackPop(&self.pc);
            return fn_ret.cycles[0];
        } else return fn_ret.cycles[1];
    }

    fn acc_and(self: *CPU, rhs: u8) Flags {
        const acc = self.get_r8(.a);
        acc.* &= rhs;

        return Flags{
            .zero = if (acc.* == 0) 1 else 0,
            .half_carry = 1,
            .carry = 0,
            .subtraction = 0,
        };
    }

    fn acc_or(self: *CPU, rhs: u8) Flags {
        const acc = self.get_r8(.a);
        acc.* |= rhs;

        return Flags{
            .zero = if (acc.* == 0) 1 else 0,
        };
    }

    fn acc_xor(self: *CPU, rhs: u8) Flags {
        const acc = self.get_r8(.a);
        acc.* ^= rhs;

        return Flags{
            .zero = if (acc.* == 0) 1 else 0,
        };
    }

    /// For most operations, the Zero, Carry, and Half carry flags are set, and
    /// this function automatically does that, for more control over flag state call
    /// the regular CPU.add() function
    pub fn genericAdd(self: *CPU, comptime T: type, dest: *T, src: anytype) void {
        if (T == u8) self.flag_state().* = CPU.add(T, dest, src) else if (T == u16) {
            // 16 bit add functions leave zero flag untouched, so the state is saved and rewritten
            const zero = self.flag_state().zero;
            self.flag_state().* = CPU.add(T, dest, src);
            self.set_flag(.{ .zero = zero });
        }
    }

    /// Adds the sum of dest and rhs to the memory location pointed to by dest
    /// return the Zero, Carry, Half-Carry, and Substraction flags, and it is up
    /// to the caller to ensure that the proper flags based on the return value
    pub fn add(comptime T: type, dest: *T, rhs: T) Flags {
        const lhs = dest.*;

        const int_type = @Type(.{ .Int = .{ .signedness = .unsigned, .bits = @typeInfo(T).Int.bits - 4 } });

        const lhs_lower: int_type = @truncate(lhs);
        const rhs_lower: int_type = @truncate(rhs);

        const half_carry = @addWithOverflow(lhs_lower, rhs_lower).@"1";

        const result = @addWithOverflow(lhs, rhs);
        dest.* = result.@"0";
        const zero: u1 = if (result.@"0" == 0) 1 else 0;
        return Flags{ .carry = result.@"1", .zero = zero, .half_carry = half_carry, .subtraction = 0 };
    }

    /// Function to handle specific instructions where immediate data is added to SP
    pub fn add_sp_e8(self: *CPU) void {
        var lsb: u8 = @truncate(self.sp >> 8);
        const e8 = self.pc_read(u8);
        const flags = CPU.add(u8, &lsb, e8);
        self.set_flag(.{ .carry = flags.carry });
        self.set_flag(.{ .half_carry = flags.half_carry });
        self.sp += e8;
    }

    /// Add to the A register, the carry flag, and the associated data
    pub fn adc(self: *CPU, rhs: u8) void {
        const partial = CPU.add(u8, self.get_r8(.a), rhs);
        const partial2 = CPU.add(u8, self.get_r8(.a), @as(u8, self.flag_state().carry));
        self.flag_state().* = CPU.join_flags(partial, partial2);
    }

    /// Adds the sum of dest and rhs to the memory location pointed to by dest
    /// return the Zero, Carry, Half-Carry, and Substraction flags, and it is up
    /// to the caller to ensure that the proper flags based on the return value
    pub fn sub(dest: *u8, rhs: u8) Flags {
        const B3 = 0b00001000;
        const B7 = 0b10000000;

        const init_b3: u1 = @truncate((dest.* & B3) >> 3);
        const init_b7: u1 = @truncate((dest.* & B7) >> 7);

        dest.* -%= rhs;

        const new_b3: u1 = @truncate((dest.* & B3) >> 3);
        const new_b7: u1 = @truncate((dest.* & B7) >> 7);

        const zero: u1 = if (dest.* == 0) 1 else 0;
        // XOR relevant bits to see if they changed,
        return Flags{ .carry = init_b7 ^ new_b7, .zero = zero, .half_carry = init_b3 ^ new_b3, .subtraction = 1 };
    }

    /// Add to the A register, the carry flag, and the associated data
    pub fn sbc(self: *CPU, rhs: u8) Flags {
        const partial = CPU.sub(self.get_r8(.a), rhs);
        const partial2 = CPU.sub(self.get_r8(.a), @as(u8, self.flag_state().carry));
        return CPU.join_flags(partial, partial2);
    }

    /// returns the OR of the two flags
    pub fn join_flags(a: Flags, b: Flags) Flags {
        return Flags{
            .zero = a.zero | b.zero,
            .carry = a.carry | b.carry,
            .half_carry = a.half_carry | b.half_carry,
            .subtraction = a.subtraction | b.subtraction,
        };
    }

    pub fn inc(self: *CPU, comptime T: type, dest: *T) void {
        const new_flag_state = CPU.add(T, dest, 1);
        if (T == u8) {
            self.set_flag(.{ .half_carry = new_flag_state.half_carry });
            self.set_flag(.{ .zero = new_flag_state.zero });
            self.set_flag(.{ .subtraction = 0 });
        }
    }

    pub fn dec(self: *CPU, comptime T: type, dest: *T) void {
        if (T == u16) dest.* -%= 1 else {
            const new_flag_state = CPU.sub(dest, 1);
            self.set_flag(.{ .half_carry = new_flag_state.half_carry });
            self.set_flag(.{ .zero = new_flag_state.zero });
            self.set_flag(.{ .subtraction = 1 });
        }
    }

    // ########################################
    // ###          INTERACTION             ###
    // ########################################

    /// Right now, we just check to see if we should buffer some byte
    /// to stdout based on the SB, and SC registers
    pub fn serialTransfer(self: *CPU, writer: anytype) void {
        const SB = 0xFF01;
        const SC = 0xFF01;

        const serial_control = self.mem_read(u8, SC);
        if (serial_control.* == 0x81) {
            // transfer requested
            _ = writer.write(&.{self.mem_read(u8, SB).*}) catch unreachable;
            serial_control.* = 0x01;
        }
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

    cpu.registers[@intFromEnum(Register8.a)] = 0x0A;
    cpu.registers[@intFromEnum(Register8.a) - 1] = 0x0F;
    cpu.registers[@intFromEnum(Register8.b)] = 0x0B;
    cpu.registers[@intFromEnum(Register8.c)] = 0x0C;
    cpu.registers[@intFromEnum(Register8.d)] = 0x0D;
    cpu.registers[@intFromEnum(Register8.e)] = 0x0E;
    cpu.registers[@intFromEnum(Register8.h)] = 0x06;
    cpu.registers[@intFromEnum(Register8.l)] = 0x09;

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

    cpu.registers[@intFromEnum(Register8.a)] = 0x0A;
    cpu.registers[@intFromEnum(Register8.a) - 1] = 0x0F;
    cpu.registers[@intFromEnum(Register8.b)] = 0x0B;
    cpu.registers[@intFromEnum(Register8.c)] = 0x0C;
    cpu.registers[@intFromEnum(Register8.d)] = 0x0D;
    cpu.registers[@intFromEnum(Register8.e)] = 0x0E;
    cpu.registers[@intFromEnum(Register8.h)] = 0x06;
    cpu.registers[@intFromEnum(Register8.l)] = 0x09;

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

    const word = cpu.mem_read(u16, 0x200);
    const byte = cpu.mem_read(u8, 0x202);

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

    const cycles = cpu.executeLoad(load_a_imm8);

    try testing.expectEqual(cpu.get_r8(.a).*, 0x42);
    try testing.expectEqual(cycles, 2);
    // only + 1 since the opcode is included in the instruction byte count
    try testing.expectEqual(old_pc + 1, cpu.pc);
}

test "flag register" {
    var cpu = CPU{};
    cpu.flag_state().zero = 1;

    const actual: u1 = @truncate((cpu.registers[6]) >> 7);
    try testing.expectEqual(1, actual);
}

// Add these tests to the end of cpu.zig

test "ADD instruction - 8-bit register" {
    var cpu = CPU{};
    cpu.registers[7] = 0x05; // Set register A to 5
    cpu.registers[5] = 0x03; // Set register B to 3

    const add = Add{
        .src = .{ .r8 = .b },
        .dest = .{ .r8 = .a },
        .with_carry = false,
        .cycles = 4,
        .bytes = 1,
    };

    const cycles = cpu.executeAdd(add);

    try testing.expectEqual(@as(u8, 0x08), cpu.get_r8(.a).*);
    try testing.expectEqual(@as(usize, 4), cycles);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().zero);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().carry);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().half_carry);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().subtraction);
}

test "ADD instruction - 16-bit register" {
    var cpu = CPU{};
    // cpu.registers[1] = 0x34; // Set HL to 0x1234
    // cpu.registers[0] = 0x12;
    // cpu.registers[3] = 0x78; // Set DE to 0x5678
    // cpu.registers[2] = 0x56;
    cpu.get_r16(.hl).* = 0x1234;
    cpu.get_r16(.de).* = 0x5678;

    const add = Add{
        .src = .{ .r16 = .de },
        .dest = .{ .r16 = .hl },
        .with_carry = false,
        .cycles = 8,
        .bytes = 1,
    };

    const cycles = cpu.executeAdd(add);

    try testing.expectEqual(@as(u16, 0x68AC), cpu.get_r16(.hl).*);
    try testing.expectEqual(@as(usize, 8), cycles);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().zero);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().carry);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().half_carry);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().subtraction);
}

test "ADC instruction" {
    var cpu = CPU{};
    cpu.registers[7] = 0x05; // Set register A to 5
    cpu.registers[5] = 0x03; // Set register B to 3
    cpu.set_flag(.{ .carry = 1 });

    const add = Add{
        .src = .{ .r8 = .b },
        .dest = .{ .r8 = .a },
        .with_carry = true,
        .cycles = 4,
        .bytes = 1,
    };

    const cycles = cpu.executeAdd(add);

    try testing.expectEqual(@as(u8, 0x09), cpu.get_r8(.a).*);
    try testing.expectEqual(@as(usize, 4), cycles);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().zero);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().carry);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().half_carry);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().subtraction);
}

test "INC instruction - 8-bit register" {
    var cpu = CPU{};
    cpu.registers[5] = 0xFF; // Set register B to 255

    const inc = Increment{
        .dest = .{ .r8 = .b },
        .bytes = 1,
        .cycles = 4,
    };

    cpu.inc(u8, cpu.get_r8(inc.dest.r8));

    try testing.expectEqual(@as(u8, 0x00), cpu.get_r8(.b).*);
    try testing.expectEqual(@as(u1, 1), cpu.flag_state().zero);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().subtraction);
    try testing.expectEqual(@as(u1, 1), cpu.flag_state().half_carry);
    // Carry flag should not be affected by INC
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().carry);
}

test "INC instruction - 16-bit register" {
    var cpu = CPU{};
    cpu.registers[1] = 0xFF; // Set HL to 0xFFFF
    cpu.registers[0] = 0xFF;

    const inc = Increment{
        .dest = .{ .r16 = .hl },
        .bytes = 1,
        .cycles = 8,
    };

    cpu.inc(u16, cpu.get_r16(inc.dest.r16));

    try testing.expectEqual(@as(u16, 0x0000), cpu.get_r16(.hl).*);
    // Flags should not be affected by 16-bit INC
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().zero);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().subtraction);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().half_carry);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().carry);
}

test "DEC instruction - 8-bit register" {
    var cpu = CPU{};
    cpu.registers[5] = 0x01; // Set register B to 1

    const dec = Decrement{
        .dest = .{ .r8 = .b },
        .bytes = 1,
        .cycles = 4,
    };

    cpu.dec(u8, cpu.get_r8(dec.dest.r8));

    try testing.expectEqual(@as(u8, 0x00), cpu.get_r8(.b).*);
    try testing.expectEqual(@as(u1, 1), cpu.flag_state().zero);
    try testing.expectEqual(@as(u1, 1), cpu.flag_state().subtraction);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().half_carry);
    // Carry flag should not be affected by DEC
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().carry);
}

test "DEC instruction - 16-bit register" {
    var cpu = CPU{};
    cpu.registers[1] = 0x00; // Set HL to 0x0000
    cpu.registers[0] = 0x00;

    const dec = Decrement{
        .dest = .{ .r16 = .hl },
        .bytes = 1,
        .cycles = 8,
    };

    cpu.dec(u16, cpu.get_r16(dec.dest.r16));

    try testing.expectEqual(@as(u16, 0xFFFF), cpu.get_r16(.hl).*);
    // Flags should not be affected by 16-bit DEC
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().zero);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().subtraction);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().half_carry);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().carry);
}

fn setupCPU() !CPU {
    // Initialize CPU state as needed
    return try CPU.init(std.testing.allocator);
}

test "NOP instruction" {
    var cpu = try setupCPU();
    defer cpu.deinit();

    const initial_pc = cpu.pc;
    cpu.mem_read(u8, cpu.pc).* = 0x00;
    const cycles = cpu.step();

    try testing.expectEqual(@as(usize, 4), cycles);
    try testing.expectEqual(initial_pc + 1, cpu.pc);
}

test "LD r8, n8 instruction" {
    var cpu = try setupCPU();
    defer cpu.deinit();

    const ld_b_n8 = Instruction{ .Load = .{
        .src = .{ .imm8 = {} },
        .dest = .{ .r8 = .b },
        .cycles = 8,
        .bytes = 2,
    } };

    const init_pc = cpu.pc;
    cpu.memory[cpu.pc] = 0x42; // Immediate value
    const cycles = cpu.execute(ld_b_n8);

    try testing.expectEqual(@as(usize, 8), cycles);
    try testing.expectEqual(@as(u8, 0x42), cpu.get_r8(.b).*);
    try testing.expectEqual(init_pc + 1, cpu.pc);
}

test "LD r16, n16 instruction" {
    var cpu = try setupCPU();
    defer cpu.deinit();

    const ld_hl_n16 = Instruction{ .Load = .{
        .src = .{ .imm16 = {} },
        .dest = .{ .r16 = .hl },
        .cycles = 12,
        .bytes = 3,
    } };

    const init_pc = cpu.pc;
    cpu.memory[cpu.pc] = 0x34;
    cpu.memory[cpu.pc + 1] = 0x12;
    const cycles = cpu.execute(ld_hl_n16);

    try testing.expectEqual(@as(usize, 12), cycles);
    try testing.expectEqual(@as(u16, 0x1234), cpu.get_r16(.hl).*);
    try testing.expectEqual(init_pc + 2, cpu.pc);
}

test "PUSH and POP instructions" {
    var cpu = try setupCPU();
    defer cpu.deinit();

    cpu.get_r16(.bc).* = 0x1234;
    const initial_sp = cpu.sp;

    const push_bc = Instruction{ .StackOp = .{
        .op = .{ .push = .bc },
        .cycles = 16,
    } };
    _ = cpu.execute(push_bc);

    try testing.expectEqual(initial_sp - 2, cpu.sp);
    try testing.expectEqual(@as(u16, 0x1234), cpu.mem_read(u16, cpu.sp).*);

    cpu.get_r16(.bc).* = 0;
    const pop_bc = Instruction{ .StackOp = .{
        .op = .{ .pop = .bc },
        .cycles = 12,
    } };
    _ = cpu.execute(pop_bc);

    try testing.expectEqual(initial_sp, cpu.sp);
    try testing.expectEqual(@as(u16, 0x1234), cpu.get_r16(.bc).*);
}

test "ADD HL, r16 instruction" {
    var cpu = try setupCPU();
    defer cpu.deinit();

    cpu.get_r16(.hl).* = 0x1000;
    cpu.get_r16(.de).* = 0x0234;

    const add_hl_de = Instruction{ .Add = .{
        .src = .{ .r16 = .de },
        .dest = .{ .r16 = .hl },
        .with_carry = false,
        .cycles = 8,
        .bytes = 1,
    } };
    const cycles = cpu.execute(add_hl_de);

    try testing.expectEqual(@as(usize, 8), cycles);
    try testing.expectEqual(@as(u16, 0x1234), cpu.get_r16(.hl).*);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().zero);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().subtraction);
}

test "INC and DEC r8 instructions" {
    var cpu = try setupCPU();
    defer cpu.deinit();

    cpu.get_r8(.a).* = 0xFF;
    const inc_a = Instruction{ .Inc = .{
        .dest = .{ .r8 = .a },
        .bytes = 1,
        .cycles = 4,
    } };
    _ = cpu.execute(inc_a);

    try testing.expectEqual(@as(u8, 0x00), cpu.get_r8(.a).*);
    try testing.expectEqual(@as(u1, 1), cpu.flag_state().zero);
    try testing.expectEqual(@as(u1, 1), cpu.flag_state().half_carry);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().subtraction);

    cpu.get_r8(.b).* = 0x01;
    const dec_b = Instruction{ .Dec = .{
        .dest = .{ .r8 = .b },
        .bytes = 1,
        .cycles = 4,
    } };
    _ = cpu.execute(dec_b);

    try testing.expectEqual(@as(u8, 0x00), cpu.get_r8(.b).*);
    try testing.expectEqual(@as(u1, 1), cpu.flag_state().zero);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().half_carry);
    try testing.expectEqual(@as(u1, 1), cpu.flag_state().subtraction);
}

test "ALU operations" {
    var cpu = try setupCPU();
    defer cpu.deinit();

    // Test AND operation
    cpu.get_r8(.a).* = 0b10101010;
    cpu.get_r8(.b).* = 0b11110000;
    const and_a_b = Instruction{ .ALUOp = .{
        .operation = .{ .AND = .{ .r8 = .b } },
        .bytes = 1,
        .cycles = 4,
    } };
    _ = cpu.execute(and_a_b);

    try testing.expectEqual(@as(u8, 0b10100000), cpu.get_r8(.a).*);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().zero);
    try testing.expectEqual(@as(u1, 1), cpu.flag_state().half_carry);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().subtraction);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().carry);

    // Test XOR operation
    cpu.get_r8(.a).* = 0b10101010;
    cpu.get_r8(.c).* = 0b11110000;
    const xor_a_c = Instruction{ .ALUOp = .{
        .operation = .{ .XOR = .{ .r8 = .c } },
        .bytes = 1,
        .cycles = 4,
    } };
    _ = cpu.execute(xor_a_c);

    try testing.expectEqual(@as(u8, 0b01011010), cpu.get_r8(.a).*);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().zero);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().half_carry);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().subtraction);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().carry);
}

test "Jump instructions" {
    var cpu = try setupCPU();
    defer cpu.deinit();

    const initial_pc = cpu.pc;

    // Test unconditional jump
    const jp_a16 = Instruction{ .Jump = .{
        .operand = .{ .a16 = .unconditional },
        .bytes = 3,
        .cycles = &[_]usize{16},
    } };
    cpu.memory[cpu.pc] = 0x34;
    cpu.memory[cpu.pc + 1] = 0x12;
    _ = cpu.execute(jp_a16);

    try testing.expectEqual(@as(u16, 0x1234), cpu.pc);

    // Test conditional jump (condition met)
    cpu.pc = initial_pc;
    cpu.flag_state().zero = 1;
    const jp_z_a16 = Instruction{ .Jump = .{
        .operand = .{ .a16 = .z },
        .bytes = 3,
        .cycles = &[_]usize{ 16, 12 },
    } };
    cpu.memory[cpu.pc] = 0x78;
    cpu.memory[cpu.pc + 1] = 0x56;
    _ = cpu.execute(jp_z_a16);

    try testing.expectEqual(@as(u16, 0x5678), cpu.pc);

    // Test conditional jump (condition not met)
    cpu.pc = initial_pc;
    cpu.flag_state().zero = 0;
    _ = cpu.execute(jp_z_a16);

    // initial pc starts at the address for immediate data
    try testing.expectEqual(initial_pc + 2, cpu.pc);
}

test "Call and Return instructions" {
    var cpu = try setupCPU();
    defer cpu.deinit();

    const initial_pc = cpu.pc; // = 0x0100
    const initial_sp = cpu.sp;

    // Test CALL instruction
    const call_a16 = Instruction{ .Call = .{
        .condition = .unconditional,
        .bytes = 3,
        .cycles = &[_]usize{24},
    } };
    cpu.memory[cpu.pc] = 0x34; // memory[0x0100] = 0x34
    cpu.memory[cpu.pc + 1] = 0x12; // memory[0x0101] = 0x12
    _ = cpu.execute(call_a16); // memory[0x0102...] = ...

    try testing.expectEqual(@as(u16, 0x1234), cpu.pc);
    try testing.expectEqual(initial_sp - 2, cpu.sp);
    try testing.expectEqual(initial_pc + 2, cpu.mem_read(u16, cpu.sp).*);

    // Test RET instruction
    const ret = Instruction{ .Return = .{
        .condition = .unconditional,
        .bytes = 1,
        .cycles = &[_]usize{16},
    } };
    _ = cpu.execute(ret);

    try testing.expectEqual(initial_pc + 2, cpu.pc);
    try testing.expectEqual(initial_sp, cpu.sp);
}

test "Flags after arithmetic operations" {
    var cpu = try setupCPU();
    defer cpu.deinit();

    // Test ADD with carry and half-carry
    cpu.get_r8(.a).* = 0xFF;
    cpu.get_r8(.b).* = 0x01;
    const add_a_b = Instruction{ .Add = .{
        .src = .{ .r8 = .b },
        .dest = .{ .r8 = .a },
        .with_carry = false,
        .cycles = 4,
        .bytes = 1,
    } };
    _ = cpu.execute(add_a_b);

    try testing.expectEqual(@as(u8, 0x00), cpu.get_r8(.a).*);
    try testing.expectEqual(@as(u1, 1), cpu.flag_state().zero);
    try testing.expectEqual(@as(u1, 1), cpu.flag_state().half_carry);
    try testing.expectEqual(@as(u1, 1), cpu.flag_state().carry);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().subtraction);

    // Test SUB with borrow
    cpu.get_r8(.a).* = 0x00;
    cpu.get_r8(.c).* = 0x01;
    const sub_a_c = Instruction{ .ALUOp = .{
        .operation = .{ .SUB = .{ .r8 = .c } },
        .bytes = 1,
        .cycles = 4,
    } };
    _ = cpu.execute(sub_a_c);

    try testing.expectEqual(@as(u8, 0xFF), cpu.get_r8(.a).*);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().zero);
    try testing.expectEqual(@as(u1, 1), cpu.flag_state().half_carry);
    try testing.expectEqual(@as(u1, 1), cpu.flag_state().carry);
    try testing.expectEqual(@as(u1, 1), cpu.flag_state().subtraction);
}
