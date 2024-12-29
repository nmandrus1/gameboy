const std = @import("std");
const opcodes = @import("opcodes.zig");
const Bus = @import("bus.zig");
const Interrupt = Bus.Interrupt;
const InterruptFlags = Bus.InterruptFlags;
const Address = opcodes.Address;
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
const Reset = opcodes.Reset;
const Return = opcodes.Return;
const InterruptControl = opcodes.InterruptControl;
const Rotation = opcodes.Rotation;
const Shift = opcodes.Shift;
const BitOperation = opcodes.BitOperation;
const Swap = opcodes.Swap;
const Instruction = opcodes.Instruction;

const CPU = @This();

const CPUError = error{
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
    pc,

    // af,
    // bc,
    // de,
    // hl,
    // sp,

    pub fn from_str(str: []const u8) CPUError!Register16 {
        return std.meta.stringToEnum(Register16, str) orelse return CPUError.InvalidRegisterName;
    }
};

// use @bitcast to convert a byte to a field and then back again
const Flags = packed struct {
    _dead_bits: u4 = 0,
    carry: u1 = 0,
    half_carry: u1 = 0,
    subtraction: u1 = 0,
    zero: u1 = 0,

    fn test_carry(self: *Flags) bool {
        return self.carry == 1;
    }

    fn test_half_carry(self: *Flags) bool {
        return self.half_carry == 1;
    }

    fn test_subtraction(self: *Flags) bool {
        return self.subtraction == 1;
    }

    fn test_zero(self: *Flags) bool {
        return self.zero == 1;
    }
};

const Flag = union(enum) { carry: u1, half_carry: u1, subtraction: u1, zero: u1 };

// Assuming DMG startup state  0 1 2 3 4 5 6 7   and   0  1  2  3  4
// Registers are indexed as    L H E D C B F A        HL DE BC AF SP
registers: [8]u8 = [_]u8{ 0x4D, 0x01, 0xD8, 0x00, 0x13, 0x00, 0xB0, 0x01 },
pc: u16 = 0x0100,
sp: u16 = 0xFFFE,

bus: *Bus,

// interrupt flag
ime: bool = false,

cycles: usize = 0,

table: ?opcodes.Opcodes = null,

// Initialize with an allocator and a writer for serial output
pub fn init(
    allocator: std.mem.Allocator,
    bus: *Bus,
) !CPU {
    return CPU{ .table = try opcodes.Opcodes.init(allocator), .bus = bus };
}

pub fn deinit(self: *CPU) void {
    self.table.?.deinit();
}

pub fn doctorLog(self: *CPU) void {
    std.log.info("A:{X:0>2} F:{X:0>2} B:{X:0>2} C:{X:0>2} D:{X:0>2} E:{X:0>2} H:{X:0>2} L:{X:0>2} SP:{X:0>4} PC:{X:0>4} PCMEM:{X:0>2},{X:0>2},{X:0>2},{X:0>2} ", .{
        self.read(u8, Register8.a),
        self.registers[6],
        self.read(u8, Register8.b),
        self.read(u8, Register8.c),
        self.read(u8, Register8.d),
        self.read(u8, Register8.e),
        self.read(u8, Register8.h),
        self.read(u8, Register8.l),
        self.sp,
        self.pc,
        // check for overflow
        // if (self.pc >= self.memory.len) 0xAA else self.read(u8, self.pc),
        // if (self.memory.len - self.pc < 1) 0xAA else self.read(u8, self.pc + 1),
        // if (self.memory.len - self.pc < 2) 0xAA else self.read(u8, self.pc + 2),
        // if (self.memory.len - self.pc < 3) 0xAA else self.read(u8, self.pc + 3),
        self.read(u8, self.pc),
        self.read(u8, self.pc + 1),
        self.read(u8, self.pc + 2),
        self.read(u8, self.pc + 3),
    });
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

// An interrupt has been requested, and now we must handle it
pub fn handle_interrupt(self: *CPU, interrupt: Interrupt) void {
    const IEnable = self.bus.IE;
    var IFlags = self.bus.IF;

    const interrupt_addr = @intFromEnum(interrupt);

    if (self.ime and IEnable.check(interrupt)) {
        self.ime = false;

        self.call(interrupt_addr);

        switch (interrupt) {
            .Serial => std.log.info("Handled Serial Interrupt", .{}),
            else => std.log.warn("Not implemented", .{}),
        }
    }

    // reset IF bet
    IFlags.set(interrupt, false);
}

// Check IF to see if any interrupts have been requested
pub fn poll_interrupts(self: *CPU) ?Interrupt {
    if (!self.ime) return null;

    const IFlags = self.bus.IF;
    const IEnable = self.bus.IE;

    if (IEnable.serial) {
        std.log.info("IF[V:{} L:{} T:{} S:{} J:{}]", .{
            IFlags.vblank,
            IFlags.lcd,
            IFlags.timer,
            IFlags.serial,
            IFlags.joypad,
        });

        std.log.info("IE[V:{} L:{} T:{} S:{} J:{}]", .{
            IEnable.vblank,
            IEnable.lcd,
            IEnable.timer,
            IEnable.serial,
            IEnable.joypad,
        });
    }

    // Check in priority order
    inline for (.{
        Interrupt.Vblank,
        Interrupt.Lcd,
        Interrupt.Timer,
        Interrupt.Serial,
        Interrupt.Joypad,
    }) |interrupt| {
        if (IFlags.check(interrupt) and IEnable.check(interrupt)) {
            return interrupt;
        }
    }

    return null;
}

/// Goal of this function is to take any readable value as a source (r8, r16, address, etc)
/// And sucessfully return the value at that destination.
pub fn read(self: *CPU, comptime T: type, from: anytype) T {
    return switch (@TypeOf(from)) {
        // Address types go through the bus
        Address => self.read(T, self.address_ptr(from)),
        u16 => switch (T) {
            u8 => self.bus.read(from),
            u16 => blk: {
                const low = self.bus.read(from);
                const high = self.bus.read(from + 1);
                break :blk (@as(u16, high) << 8) | low;
            },
            else => @compileError("Invalid type for memory read"),
        },
        // Register reads stay internal to CPU
        Register8 => blk: {
            if (T != u8) @compileError("Cannot read non-byte value from byte register");
            break :blk switch (from) {
                .hl => self.read(u8, self.read(u16, Register16.hl)),
                else => self.registers[@intFromEnum(from)],
            };
        },
        Register16 => blk: {
            if (T != u16) @compileError("Cannot read non-word value from 16 bit register");
            break :blk switch (from) {
                .sp => self.sp,
                .pc => self.pc,
                else => std.mem.readInt(u16, @ptrCast(&self.registers[@intFromEnum(from) * 2]), .little),
            };
        },
        else => @compileError("Invalid type, not readable"),
    };
}

/// Goal of this function is to take any writable value as a destination (r8, r16, address, etc)
/// And sucessfully write the src value to that destination.
/// This function assumes that the src value is raw data, and not a representation of any data, (like a Register or Address)
pub fn write(self: *CPU, comptime T: type, dest: anytype, src: T) void {
    switch (@TypeOf(dest)) {
        // Address types go through the bus
        Address => self.write(T, self.address_ptr(dest), src),
        u16 => switch (T) {
            u8 => self.bus.write(dest, src),
            u16 => {
                self.bus.write(dest, @truncate(src));
                self.bus.write(dest + 1, @truncate(src >> 8));
            },
            else => @compileError("Invalid type for memory write"),
        },
        // Register writes stay internal to CPU
        Register8 => {
            if (T != u8) @compileError("Cannot write non-byte value to byte register");
            switch (dest) {
                .hl => self.write(u8, self.read(u16, Register16.hl), src),
                else => self.registers[@intFromEnum(dest)] = src,
            }
        },
        Register16 => {
            if (T != u16) @compileError("Cannot write non-word value to 16 bit register");
            switch (dest) {
                .sp => self.sp = src,
                .pc => self.pc = src,
                else => std.mem.writeInt(u16, @ptrCast(&self.registers[@intFromEnum(dest) * 2]), src, .little),
            }
        },
        else => @compileError("Invalid type, not writable"),
    }
}

/// Given an address struct this returns a u16 pointer to
/// somewhere in memory, use the read() function to read
/// a byte/word from that address
pub fn address_ptr(self: *CPU, addr: opcodes.Address) u16 {
    return switch (addr) {
        .bc, .de, .hl => ret: {
            const r16 = Register16.from_str(@tagName(addr)) catch unreachable;
            break :ret self.read(u16, r16);
        },
        .hli => ret: {
            const val = self.read(u16, Register16.hl);
            self.write(u16, Register16.hl, val + 1);
            break :ret val;
        },
        .hld => ret: {
            const val = self.read(u16, Register16.hl);
            self.write(u16, Register16.hl, val - 1);
            break :ret val;
        },
        .a16 => self.pc_read(u16),
        .c => 0xFF00 + @as(u16, @intCast(self.read(u8, Register8.c))),
        .a8 => 0xFF00 + @as(u16, @intCast(self.pc_read(u8))),
    };
}

// read a byte or word from the program counter
// TODO: test this
pub fn pc_read(self: *CPU, comptime T: type) T {
    defer self.pc += @divExact(@typeInfo(T).Int.bits, 8);
    switch (T) {
        u8, u16 => return self.read(T, self.pc),
        else => @compileError("Invalid type, can only read u8 or u16 from Program Counter"),
    }
}

// step the CPU forward by one instruction
pub fn step(self: *CPU) usize {
    // switch (self.state) {
    //     .Running => {
    // self.doctorLog();

    if (self.poll_interrupts()) |int| {
        // An interrupt has been requested
        std.log.info("Interrupt Requested", .{});
        self.handle_interrupt(int);
    }

    const opcode = self.pc_read(u8);
    const instr = self.decode(opcode);
    return self.execute(instr);
    //     },
    // }
}

pub fn decode(self: *CPU, opcode: u8) Instruction {
    // TODO: make opcode table better
    if (opcode == 0xCB) {
        // Handle Prefix
        const cb_opcode = self.pc_read(u8);
        // std.log.debug("CB Opcode: 0x{X}", .{cb_opcode});
        return Instruction.from_json_opcode(self.table.?.cbprefixed[cb_opcode]) catch |err| {
            std.log.err("Error getting instruction from JsonOpcode - {!}", .{err});
            return Instruction{ .Nop = {} };
        };
    } else {
        return Instruction.from_json_opcode(self.table.?.unprefixed[opcode]) catch |err| {
            std.log.err("Error getting instruction from JsonOpcode - {!}, Opcode: 0x{X}", .{ err, opcode });
            return Instruction{ .Nop = {} };
        };
    }
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
        .Reset => |rst| self.executeRst(rst),
        .Return => |ret| self.executeReturn(ret),
        .InterruptControl => |inter| self.executeInterruptControl(inter),
        .Rotate => |rot| self.executeRotation(rot),
        .Shift => |shift| self.executeShift(shift),
        .BitOp => |bop| self.executeBitOp(bop),
        .Swap => |swp| self.executeSwap(swp),
    };
}

// ########################################
// ###      INSTRUCTION EXECUTION       ###
// ########################################

// push a value to the stack
pub fn stackPush(self: *CPU, r16: Register16) void {
    self.sp -= 2;

    // write the value contained in the register to the stack
    self.write(u16, self.sp, self.read(u16, r16));
}

// pop a value from the stack
pub fn stackPop(self: *CPU, r16: Register16) void {

    // read value from stack and write it to the register
    var popped = self.read(u16, self.sp);
    if (r16 == .af) popped &= 0xfff0;
    self.write(u16, r16, popped);

    self.sp += 2;

    if (self.sp > 0xFFFE) @panic("Stack Underflow");
}

pub fn executeStackOp(self: *CPU, op: StackOp) usize {
    switch (op.op) {
        .push => |r16| self.stackPush(r16),
        .pop => |r16| self.stackPop(r16),
    }
    return op.cycles;
}

pub fn executeLoad(self: *CPU, op: Load) usize {
    // op.log();
    switch (op.src) {
        // 8 bit load
        .r8, .imm8, .address => {
            const src = self.fetchLoadData(u8, op.src);
            switch (op.dest) {
                .r8 => |r8| self.write(u8, r8, src),
                .address => |addr| self.write(u8, addr, src),
                else => unreachable,
            }
            // CPU.write(u8, dest, src);
        },
        // 16 bit load
        .r16, .imm16, .sp_offset => {
            const src = self.fetchLoadData(u16, op.src);
            switch (op.dest) {
                .r16 => |r16| self.write(u16, r16, src),
                .address => |address| self.write(u16, address, src),
                else => unreachable,
            }
            // CPU.old_write(u16, dest, src);
        },
    }

    return op.cycles;
}

// fetch data for a load operand, can be a word or byte
pub fn fetchLoadData(self: *CPU, comptime T: type, operand: LoadOperand) T {
    comptime if (!(T == u8 or T == u16)) @compileError("Invalid Type -- expected u8 or u16");
    return switch (T) {
        u8 => switch (operand) {
            .r8 => |r8| self.read(u8, r8),
            // TODO: find a way to better express the posibilty of an addr pointing to a u16
            .address => |addr| self.read(u8, addr),
            .imm8 => self.pc_read(u8),
            else => unreachable,
        },
        u16 => switch (operand) {
            .r16 => |r16| self.read(u16, r16),
            .imm16 => self.pc_read(u16),
            .sp_offset => blk: {
                break :blk self.add_sp_e8();
            },
            else => unreachable,
        },
        else => unreachable,
    };
}

pub fn executeAdd(self: *CPU, add_instr: Add) usize {
    // TODO: I think this can be done better....
    const rhs = switch (add_instr.src) {
        // 8 bit add
        .r8 => |r8| self.read(u8, r8),
        .imm8 => self.pc_read(u8),
        .r16 => |r16| {
            self.genericAdd(u16, Register16.hl, self.read(u16, r16));
            return add_instr.cycles;
        },
        .e8 => { // go ahead and execute special function
            self.sp = self.add_sp_e8();
            return add_instr.cycles;
        },
    };

    // if we are here then its an 8 bit load
    if (add_instr.with_carry) self.adc(rhs) else self.genericAdd(u8, Register8.a, rhs);
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
            .r8 => |r8| self.read(u8, r8),
            .imm8 => self.pc_read(u8),
        },
        else => undefined,
    };

    self.flag_state().* = switch (instr.operation) {
        .SUB => blk: {
            const acc = self.read(u8, Register8.a);
            const result = CPU.sub(acc, rhs);
            self.write(u8, Register8.a, result.@"0");
            break :blk result.@"1";
        },
        .SBC => self.sbc(rhs),
        .AND => self.acc_and(rhs),
        .OR => self.acc_or(rhs),
        .XOR => self.acc_xor(rhs),
        .CP => blk: {
            //Subtracts from the 8-bit A register, the 8-bit register r, and updates flags based on the result.
            // This instruction is basically identical to SUB r, but does not update the A register.
            // store and write back the oringal A register value
            const acc = self.read(u8, Register8.a);
            break :blk CPU.sub(acc, rhs).@"1";
        },

        // toggles carry, sets HC, and N to 0, doesnt touch Z
        .CCF => .{ .carry = ~self.flag_state().carry, .zero = self.flag_state().zero },
        .SCF => .{ .carry = 1, .zero = self.flag_state().zero },
        .DAA => blk: {
            self.daa();
            break :blk self.flag_state().*;
        },
        .CPL => blk: {
            self.write(u8, Register8.a, ~self.read(u8, Register8.a));
            self.set_flag(.{ .subtraction = 1 });
            self.set_flag(.{ .half_carry = 1 });
            break :blk self.flag_state().*;
        },
    };

    return instr.cycles;
}

fn executeInc(self: *CPU, inc_instr: Increment) usize {
    switch (inc_instr.dest) {
        .r8 => |r8| self.inc(u8, r8),
        .r16 => |r16| self.inc(u16, r16),
    }

    return inc_instr.cycles;
}

fn executeDec(self: *CPU, dec_instr: Decrement) usize {
    switch (dec_instr.dest) {
        .r8 => |r8| self.dec(u8, r8),
        .r16 => |r16| self.dec(u16, r16),
    }

    return dec_instr.cycles;
}

fn jump(self: *CPU, addr: u16) void {
    std.mem.writeInt(u16, @ptrCast(&self.pc), addr, .little);
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
        .hl => |cc| .{ .addr = self.read(u16, Register16.hl), .cc = cc },
        .a16 => |cc| .{ .addr = self.pc_read(u16), .cc = cc },
        // e8 is a signed integer, so we read a byte, cast it, add it to a casted pc and then cast
        // the result back to a u16
        .e8 => |cc| .{ .addr = @intCast(@as(i8, @bitCast(self.pc_read(u8))) +% @as(i32, @intCast(self.pc))), .cc = cc },
    };

    // if the condition is met, then flow through the switch statement,
    // otherwise return the relevant cycle count
    if (CPU.check_condition(result.cc, self.flag_state().*)) {
        self.jump(result.addr);
        return jmp.cycles[0];
    } else return jmp.cycles[1];
}

fn call(self: *CPU, addr: u16) void {
    self.stackPush(Register16.pc);
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

fn executeRst(self: *CPU, rst: Reset) usize {
    self.call(rst.addr);
    return rst.cycles;
}

fn executeReturn(self: *CPU, fn_ret: Return) usize {
    // if the condition is met, then flow through the switch statement,
    // otherwise return the relevant cycle count
    if (CPU.check_condition(fn_ret.condition, self.flag_state().*)) {
        self.stackPop(Register16.pc);
        return fn_ret.cycles[0];
    } else return fn_ret.cycles[1];
}

fn executeInterruptControl(self: *CPU, intr: InterruptControl) usize {
    //TODO: "The effect of ei is delayed by one instruction.
    // This means that ei followed immediately by di does not allow any interrupts between them"
    return switch (intr) {
        .ei => |data| blk: {
            self.ime = true;
            break :blk data.cycles;
        },
        .reti => |reti| blk: {
            self.ime = true;
            break :blk self.executeReturn(reti);
        },
        .di => |di| blk: {
            self.ime = false;
            break :blk di.cycles;
        },
    };
}

fn acc_and(self: *CPU, rhs: u8) Flags {
    var acc = self.read(u8, Register8.a);
    acc &= rhs;
    self.write(u8, Register8.a, acc);

    return Flags{
        .zero = if (acc == 0) 1 else 0,
        .half_carry = 1,
        .carry = 0,
        .subtraction = 0,
    };
}

fn acc_or(self: *CPU, rhs: u8) Flags {
    var acc = self.read(u8, Register8.a);
    acc |= rhs;
    self.write(u8, Register8.a, acc);

    return Flags{
        .zero = if (acc == 0) 1 else 0,
    };
}

fn acc_xor(self: *CPU, rhs: u8) Flags {
    var acc = self.read(u8, Register8.a);
    acc ^= rhs;
    self.write(u8, Register8.a, acc);

    return Flags{
        .zero = if (acc == 0) 1 else 0,
    };
}

/// For most operations, the Zero, Carry, and Half carry flags are set, and
/// this function automatically does that, for more control over flag state call
/// the regular CPU.add() function
/// T is the type we're operating on & dest/src are readable/writeable types
pub fn genericAdd(self: *CPU, comptime T: type, dest: anytype, src: T) void {
    const lhs = self.read(T, dest);
    switch (T) {
        u8 => {
            const result = CPU.add(T, lhs, src);
            self.write(T, dest, result.@"0");
            self.flag_state().* = result.@"1";
        },
        u16 => {
            const zero = self.flag_state().zero;
            const result = CPU.add(T, lhs, src);
            self.write(T, dest, result.@"0");
            self.flag_state().* = result.@"1";
            self.set_flag(.{ .zero = zero });
        },
        else => @compileError("Invalid type for arithmetic"),
    }
}

/// Adds the sum of dest and rhs to the memory location pointed to by dest
/// return the Zero, Carry, Half-Carry, and Substraction flags, and it is up
/// to the caller to ensure that the proper flags based on the return value
pub fn add(comptime T: type, lhs: T, rhs: T) struct { T, Flags } {
    const int_type = @Type(.{ .Int = .{ .signedness = .unsigned, .bits = @typeInfo(T).Int.bits - 4 } });

    const lhs_lower: int_type = @truncate(lhs);
    const rhs_lower: int_type = @truncate(rhs);

    const half_carry = @addWithOverflow(lhs_lower, rhs_lower).@"1";

    const result = @addWithOverflow(lhs, rhs);
    const zero: u1 = if (result.@"0" == 0) 1 else 0;
    const flags = Flags{ .carry = result.@"1", .zero = zero, .half_carry = half_carry, .subtraction = 0 };
    return .{ result.@"0", flags };
}

/// Function to handle specific instructions where immediate data is added to SP
pub fn add_sp_e8(self: *CPU) u16 {
    const e8 = self.pc_read(u8);
    const lsb: u8 = @truncate(self.sp);
    const flags = CPU.add(u8, lsb, e8).@"1";

    const signed_e8: i32 = @intCast(@as(i8, @bitCast(e8)));
    const signed_sp = @as(i32, self.sp);

    // add and collect possible overflow bit
    const add_result: i16 = @truncate(signed_sp + signed_e8);
    const new_sp: u16 = @bitCast(add_result);

    // If there was a carry in bit 3, then
    // const half_carry: u1 = if (new_sp & 0x10 != self.sp & 0x10) 1 else 0;
    // const carry: u1 = if (new_sp & 0x0100 != self.sp & 0x0100) 1 else 0;

    self.flag_state().* = Flags{ .carry = flags.carry, .half_carry = flags.half_carry, .zero = 0, .subtraction = 0 };
    return new_sp;
}

/// Add to the A register, the carry flag, and the associated data
pub fn adc(self: *CPU, rhs: u8) void {
    const acc = self.read(u8, Register8.a);

    const partial = CPU.add(u8, acc, rhs);
    const partial2 = CPU.add(u8, partial.@"0", @as(u8, self.flag_state().carry));

    self.write(u8, Register8.a, partial2.@"0");
    // TODO: Redundant zero flag operations
    self.flag_state().* = CPU.join_flags(partial.@"1", partial2.@"1");
    self.flag_state().*.zero = if (partial2.@"0" == 0) 1 else 0;
}

/// Adds the sum of dest and rhs to the memory location pointed to by dest
/// return the Zero, Carry, Half-Carry, and Substraction flags, and it is up
/// to the caller to ensure that the proper flags based on the return value
pub fn sub(lhs: u8, rhs: u8) struct { u8, Flags } {
    const result: u8 = lhs -% rhs;

    const flags = Flags{
        .zero = if (result == 0) 1 else 0,
        .carry = if (rhs > lhs) 1 else 0,
        .half_carry = if ((rhs & 0xF) > (lhs & 0xF)) 1 else 0,
        .subtraction = 1,
    };

    return .{ result, flags };
}

/// Add to the A register, the carry flag, and the associated data
pub fn sbc(self: *CPU, rhs: u8) Flags {
    const acc = self.read(u8, Register8.a);

    const partial = CPU.sub(acc, rhs);
    const partial2 = CPU.sub(partial.@"0", @as(u8, self.flag_state().carry));

    self.write(u8, Register8.a, partial2.@"0");
    var flags = CPU.join_flags(partial.@"1", partial2.@"1");
    flags.zero = if (partial2.@"0" == 0) 1 else 0;
    return flags;
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

pub fn inc(self: *CPU, comptime T: type, dest: anytype) void {
    const value = self.read(T, dest);
    const result = CPU.add(T, value, 1);
    self.write(T, dest, result.@"0");
    if (T == u8) {
        self.set_flag(.{ .half_carry = result.@"1".half_carry });
        self.set_flag(.{ .zero = result.@"1".zero });
        self.set_flag(.{ .subtraction = 0 });
    }
}

pub fn dec(self: *CPU, comptime T: type, dest: anytype) void {
    var value = self.read(T, dest);
    switch (T) {
        u16 => value -%= 1,
        u8 => {
            const result = CPU.sub(value, 1);
            value = result.@"0";
            self.set_flag(.{ .half_carry = result.@"1".half_carry });
            self.set_flag(.{ .zero = result.@"1".zero });
            self.set_flag(.{ .subtraction = 1 });
        },
        else => @compileError("Invalid arithmetic type, expected u8 or u16"),
    }
    self.write(T, dest, value);
}

pub fn bit_test(byte: u8, bit: u3) u1 {
    return @truncate((byte >> bit) & 1);
}

pub fn bit_set(byte: u8, pos: u3) u8 {
    return byte | @as(u8, 1) << pos;
}

pub fn bit_clear(byte: u8, pos: u3) u8 {
    return byte & ~(@as(u8, 1) << pos);
}

// TODO: Cleanup?
pub fn executeRotation(self: *CPU, rot: Rotation) usize {
    var target = self.read(u8, rot.r8);
    const flags = self.flag_state();

    // extract either bit 7 or bit 0 depending on direction
    const rotated_bit: u1 = blk: {
        if (rot.direction == .Left) {
            // get left most bit, shift target, replace bit 0 with carry, and set carry to bit 7
            const b7: u1 = @truncate((target >> 7) & 1);
            target <<= 1;
            target |= flags.carry;
            flags.carry = b7;
            break :blk b7;
        } else {
            // get rightmost bit, shift target, replace bit 7 with carry, and set carry to bit 0
            const b0 = CPU.bit_test(target, 0);
            target >>= 1;
            target |= (@as(u8, flags.carry) << 7);
            flags.carry = b0;
            break :blk b0;
        }
    };

    // on circular rotation we overwrite the carry bit with
    // whatever was in bit7
    if (rot.circular and rot.direction == .Left) {
        target = (target & 0b11111110) | rotated_bit;
    } else if (rot.circular and rot.direction == .Right) {
        target = (target & 0b01111111) | (@as(u8, rotated_bit) << 7);
    }

    // only when operation on A register is the zero flag always set to 0 otherwise it
    // depends on the result
    if (rot.r8 == .a and rot.bytes == 1) flags.zero = 0 else flags.zero = if (target == 0) 1 else 0;

    // write result back to register
    self.write(u8, rot.r8, target);
    flags.subtraction = 0;
    flags.half_carry = 0;

    return rot.cycles;
}

pub fn executeShift(self: *CPU, shift: Shift) usize {
    // Logical shift -> fill empty space with 0
    // Arithmetic shift -> sign bit remains unchanged

    var byte = self.read(u8, shift.r8);
    const flags = self.flag_state();
    const bits = .{ .b7 = CPU.bit_test(byte, 7), .b0 = CPU.bit_test(byte, 0) };

    switch (shift.direction) {
        .Left => {
            byte <<= 1;
            flags.carry = bits.b7;
        },
        .Right => {
            byte >>= 1;
            flags.carry = bits.b0;
            if (shift.shift_type == .Arithmetic) byte |= @as(u8, @intCast(bits.b7)) << 7;
        },
    }

    flags.zero = if (byte == 0) 1 else 0;
    flags.half_carry = 0;
    flags.subtraction = 0;

    self.write(u8, shift.r8, byte);

    return shift.cycles;
}

pub fn executeSwap(self: *CPU, swap: Swap) usize {
    var byte = self.read(u8, swap.r8);
    const flags = self.flag_state();

    const upper_nib = (byte & 0xF0) >> 4;
    const lower_nib = (byte & 0x0F) << 4;

    byte = lower_nib | upper_nib;

    flags.subtraction = 0;
    flags.half_carry = 0;
    flags.carry = 0;
    flags.zero = if (byte == 0) 1 else 0;

    self.write(u8, swap.r8, byte);

    return swap.cycles;
}

pub fn executeBitOp(self: *CPU, bop: BitOperation) usize {
    const byte = self.read(u8, bop.r8);
    const flags = self.flag_state();
    switch (bop.op) {
        .Test => {
            flags.half_carry = 1;
            flags.subtraction = 0;
            flags.zero = ~CPU.bit_test(byte, bop.bit);
        },
        .Set => self.write(u8, bop.r8, CPU.bit_set(byte, bop.bit)),
        .Reset => self.write(u8, bop.r8, CPU.bit_clear(byte, bop.bit)),
    }

    return bop.cycles;
}

pub fn daa(self: *CPU) void {
    // https://forums.nesdev.org/viewtopic.php?t=15944

    // note: assumes a is a uint8_t and wraps from 0xff to 0
    const flags = self.flag_state();
    var acc = self.read(u8, Register8.a);
    if (!flags.test_subtraction()) { // after an addition, adjust if (half-)carry occurred or if result is out of bounds
        if (flags.test_carry() or acc > 0x99) {
            acc +%= 0x60;
            flags.carry = 1;
        }
        if (flags.test_half_carry() or (acc & 0x0f) > 0x09) acc +%= 0x6;
    } else { // after a subtraction, only adjust if (half-)carry occurred
        if (flags.test_carry()) acc -%= 0x60;
        if (flags.test_half_carry()) acc -%= 0x6;
    }
    // these flags are always updated
    flags.zero = @intFromBool(acc == 0); // the usual z flag
    flags.half_carry = 0; // h flag is always cleared

    self.write(u8, Register8.a, acc);
}

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

    try testing.expectEqual(cpu.read(u8, Register8.a), 0x42);
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

test "ADC instruction" {
    var cpu = CPU{};
    cpu.registers[7] = 0x05; // Set register A to 5
    cpu.registers[5] = 0x03; // Set register B to 3
    cpu.set_flag(.{ .carry = 1 });

    const add_op = Add{
        .src = .{ .r8 = .b },
        .dest = .{ .r8 = .a },
        .with_carry = true,
        .cycles = 4,
        .bytes = 1,
    };

    const cycles = cpu.executeAdd(add_op);

    try testing.expectEqual(@as(u8, 0x09), cpu.read(u8, Register8.a));
    try testing.expectEqual(@as(usize, 4), cycles);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().zero);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().carry);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().half_carry);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().subtraction);
}

test "INC instruction - 8-bit register" {
    var cpu = CPU{};
    cpu.registers[5] = 0xFF; // Set register B to 255

    const inc_op = Increment{
        .dest = .{ .r8 = .b },
        .bytes = 1,
        .cycles = 4,
    };

    cpu.inc(u8, inc_op.dest.r8);

    try testing.expectEqual(@as(u8, 0x00), cpu.read(u8, inc_op.dest.r8));
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

    const inc_op = Increment{
        .dest = .{ .r16 = .hl },
        .bytes = 1,
        .cycles = 8,
    };

    cpu.inc(u16, inc_op.dest.r16);

    try testing.expectEqual(@as(u16, 0x0000), cpu.read(u16, inc_op.dest.r16));
    // Flags should not be affected by 16-bit INC
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().zero);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().subtraction);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().half_carry);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().carry);
}

test "DEC instruction - 8-bit register" {
    var cpu = CPU{};
    cpu.write(u8, Register8.b, 0x01); // Set register B to 1

    const dec_op = Decrement{
        .dest = .{ .r8 = .b },
        .bytes = 1,
        .cycles = 4,
    };

    cpu.dec(u8, dec_op.dest.r8);

    try testing.expectEqual(@as(u8, 0x00), cpu.read(u8, dec_op.dest.r8));
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

    const dec_op = Decrement{
        .dest = .{ .r16 = .hl },
        .bytes = 1,
        .cycles = 8,
    };

    cpu.dec(u16, dec_op.dest.r16);

    try testing.expectEqual(@as(u16, 0xFFFF), cpu.read(u16, dec_op.dest.r16));
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
    cpu.write(u8, cpu.pc, 0x00);
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
    try testing.expectEqual(@as(u8, 0x42), cpu.read(u8, ld_b_n8.Load.dest.r8));
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
    try testing.expectEqual(@as(u16, 0x1234), cpu.read(u16, ld_hl_n16.Load.dest.r16));
    try testing.expectEqual(init_pc + 2, cpu.pc);
}

test "ADD HL, r16 instruction" {
    var cpu = try setupCPU();
    defer cpu.deinit();

    cpu.write(u16, Register16.hl, 0x1000);
    cpu.write(u16, Register16.de, 0x0234);

    const add_hl_de = Instruction{ .Add = .{
        .src = .{ .r16 = .de },
        .dest = .{ .r16 = .hl },
        .with_carry = false,
        .cycles = 8,
        .bytes = 1,
    } };
    const cycles = cpu.execute(add_hl_de);

    try testing.expectEqual(@as(usize, 8), cycles);
    try testing.expectEqual(@as(u16, 0x1234), cpu.read(u16, add_hl_de.Add.dest.r16));
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().zero);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().subtraction);
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

// ########################################################

test "read and write u8 to registers" {
    var cpu = try setupCPU();
    defer cpu.deinit();

    cpu.write(u8, Register8.a, 0x12);
    cpu.write(u8, Register8.b, 0x34);
    cpu.write(u8, Register8.c, 0x56);

    try testing.expectEqual(@as(u8, 0x12), cpu.read(u8, Register8.a));
    try testing.expectEqual(@as(u8, 0x34), cpu.read(u8, Register8.b));
    try testing.expectEqual(@as(u8, 0x56), cpu.read(u8, Register8.c));
}

test "read and write u16 to registers" {
    var cpu = try setupCPU();
    defer cpu.deinit();

    cpu.write(u16, Register16.bc, 0x1234);
    cpu.write(u16, Register16.de, 0x5678);
    cpu.write(u16, Register16.hl, 0x9ABC);

    try testing.expectEqual(@as(u16, 0x1234), cpu.read(u16, Register16.bc));
    try testing.expectEqual(@as(u16, 0x5678), cpu.read(u16, Register16.de));
    try testing.expectEqual(@as(u16, 0x9ABC), cpu.read(u16, Register16.hl));
}

test "read and write to memory" {
    var cpu = try setupCPU();
    defer cpu.deinit();

    cpu.write(u8, @as(u16, 0x1000), 0x42);
    cpu.write(u16, @as(u16, 0x2000), 0xABCD);

    try testing.expectEqual(@as(u8, 0x42), cpu.read(u8, @as(u16, 0x1000)));
    try testing.expectEqual(@as(u16, 0xABCD), cpu.read(u16, @as(u16, 0x2000)));
}

test "read and write using Address" {
    var cpu = try setupCPU();
    defer cpu.deinit();

    cpu.write(u16, Register16.hl, 0x1234);
    cpu.write(u8, Address.hl, 0x42);

    try testing.expectEqual(@as(u8, 0x42), cpu.read(u8, Address.hl));
}

test "stack operations" {
    var cpu = try setupCPU();
    defer cpu.deinit();

    cpu.write(u16, Register16.sp, 0xFFFE);
    cpu.stackPush(Register16.bc);

    try testing.expectEqual(@as(u16, 0xFFFC), cpu.read(u16, Register16.sp));

    cpu.stackPop(Register16.de);

    try testing.expectEqual(@as(u16, 0xFFFE), cpu.read(u16, Register16.sp));
    try testing.expectEqual(@as(u16, 0x0000), cpu.read(u16, Register16.de));
}

test "ALU operations" {
    var cpu = try setupCPU();
    defer cpu.deinit();

    // Test ADD
    cpu.write(u8, Register8.a, 0x3C);
    cpu.write(u8, Register8.b, 0x12);
    cpu.genericAdd(u8, Register8.a, cpu.read(u8, Register8.b));
    try testing.expectEqual(@as(u8, 0x4E), cpu.read(u8, Register8.a));

    // Test SUB
    cpu.write(u8, Register8.a, 0x3C);
    cpu.write(u8, Register8.c, 0x12);
    const sub_result = CPU.sub(cpu.read(u8, Register8.a), cpu.read(u8, Register8.c));
    cpu.write(u8, Register8.a, sub_result.@"0");
    try testing.expectEqual(@as(u8, 0x2A), cpu.read(u8, Register8.a));

    // Test AND
    cpu.write(u8, Register8.a, 0b10101010);
    cpu.write(u8, Register8.d, 0b11110000);
    _ = cpu.acc_and(cpu.read(u8, Register8.d));
    try testing.expectEqual(@as(u8, 0b10100000), cpu.read(u8, Register8.a));

    // Test OR
    cpu.write(u8, Register8.a, 0b10101010);
    cpu.write(u8, Register8.e, 0b11110000);
    _ = cpu.acc_or(cpu.read(u8, Register8.e));
    try testing.expectEqual(@as(u8, 0b11111010), cpu.read(u8, Register8.a));

    // Test XOR
    cpu.write(u8, Register8.a, 0b10101010);
    cpu.write(u8, Register8.h, 0b11110000);
    _ = cpu.acc_xor(cpu.read(u8, Register8.h));
    try testing.expectEqual(@as(u8, 0b01011010), cpu.read(u8, Register8.a));
}

test "Increment and Decrement" {
    var cpu = try setupCPU();
    defer cpu.deinit();

    // Test INC
    cpu.write(u8, Register8.a, 0xFF);
    cpu.inc(u8, Register8.a);
    try testing.expectEqual(@as(u8, 0x00), cpu.read(u8, Register8.a));
    try testing.expectEqual(@as(u1, 1), cpu.flag_state().zero);
    try testing.expectEqual(@as(u1, 1), cpu.flag_state().half_carry);

    // Test DEC
    cpu.write(u8, Register8.b, 0x01);
    cpu.dec(u8, Register8.b);
    try testing.expectEqual(@as(u8, 0x00), cpu.read(u8, Register8.b));
    try testing.expectEqual(@as(u1, 1), cpu.flag_state().zero);
    try testing.expectEqual(@as(u1, 1), cpu.flag_state().subtraction);
}

test "Jump and Call operations" {
    var cpu = try setupCPU();
    defer cpu.deinit();

    // Test Jump
    cpu.write(u16, Register16.pc, 0x1000);

    cpu.jump(0x2000);
    try testing.expectEqual(@as(u16, 0x2000), cpu.pc);

    // Test Call
    cpu.write(u16, Register16.sp, 0xFFFE);
    cpu.call(0x3000);
    try testing.expectEqual(@as(u16, 0x3000), cpu.pc);
    try testing.expectEqual(@as(u16, 0xFFFC), cpu.read(u16, Register16.sp));
    try testing.expectEqual(@as(u16, 0x2000), cpu.read(u16, cpu.read(u16, Register16.sp)));
}

test "Flags after arithmetic operations" {
    var cpu = try setupCPU();
    defer cpu.deinit();

    // Test ADD with carry and half-carry
    cpu.write(u8, Register8.a, 0xFF);
    cpu.write(u8, Register8.b, 0x01);
    cpu.genericAdd(u8, Register8.a, cpu.read(u8, Register8.b));

    try testing.expectEqual(@as(u8, 0x00), cpu.read(u8, Register8.a));
    try testing.expectEqual(@as(u1, 1), cpu.flag_state().zero);
    try testing.expectEqual(@as(u1, 1), cpu.flag_state().half_carry);
    try testing.expectEqual(@as(u1, 1), cpu.flag_state().carry);
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().subtraction);

    // Test SUB with borrow
    cpu.write(u8, Register8.a, 0x00);
    cpu.write(u8, Register8.c, 0x01);
    const sub_result = CPU.sub(cpu.read(u8, Register8.a), cpu.read(u8, Register8.c));
    cpu.write(u8, Register8.a, sub_result.@"0");
    cpu.flag_state().* = sub_result.@"1";

    try testing.expectEqual(@as(u8, 0xFF), cpu.read(u8, Register8.a));
    try testing.expectEqual(@as(u1, 0), cpu.flag_state().zero);
    try testing.expectEqual(@as(u1, 1), cpu.flag_state().half_carry);
    try testing.expectEqual(@as(u1, 1), cpu.flag_state().carry);
    try testing.expectEqual(@as(u1, 1), cpu.flag_state().subtraction);
}
