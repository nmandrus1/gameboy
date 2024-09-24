const std = @import("std");
const json = std.json;
const cpu = @import("cpu.zig");

const opcode_json = @embedFile("Opcodes.json");
const unprefixed_json = @embedFile("unprefixed.json");

// GOAL: the goal of this module is to develop an Opcode Jump table, where
// each entry in the table knows exactly what data it needs to
// carry out the instruction.

pub const OpcodeError = error{
    InvalidOperandName,
    InvalidSrcName,
    InvalidDestName,
    UnknownMnemonic,
};

// Address knows what data it needs to build the address type
pub const Address = enum {
    // 16 bit register
    bc,
    de,
    hl,
    hli,
    hld,
    // 16 bit immediate data
    a16,
    // 8 bit immediate data
    a8,
    // 8 bit register, in 8 bit LD instructions
    // its only ever the C register
    c,
};

pub const LoadOperand = union(enum) {
    r8: cpu.Register8,
    r16: cpu.Register16,
    address: Address,
    imm8: void,
    imm16: void,
    // special case
    sp_offset: void,

    // Parse an operand from the Loaded JSON file and will
    // interpret it as an operand for a LD instruction
    fn from_json_operand(op: JsonOperand) !LoadOperand {
        if (op.immediate) {
            // operand is a register or byte
            const Case = enum { a, b, c, d, e, h, l, n8, bc, de, hl, sp, n16 };
            const case = std.meta.stringToEnum(Case, op.name) orelse return OpcodeError.InvalidOperandName;
            return switch (case) {
                .a, .b, .c, .d, .e, .h, .l => .{ .r8 = try cpu.Register8.from_str(op.name) },
                .bc, .de, .hl, .sp => .{ .r16 = try cpu.Register16.from_str(op.name) },
                .n16 => .imm16,
                .n8 => .imm8,
            };
        } else {
            const case = std.meta.stringToEnum(Address, op.name) orelse return OpcodeError.InvalidOperandName;
            return .{ .address = case };
        }
    }
};

pub const StackOperation = struct {
    const Op = union(enum) {
        push: cpu.Register16,
        pop: cpu.Register16,
    };

    op: Op,
    cycles: usize,

    fn from_json_opcode(opcode: JsonOpcode) !StackOperation {
        const Case = enum { PUSH, POP };
        const case = std.meta.stringToEnum(Case, opcode.mnemonic) orelse return OpcodeError.InvalidOperandName;
        const r16 = try cpu.Register16.from_str(opcode.operands[0].name);
        return switch (case) {
            .PUSH => .{ .op = Op{ .push = r16 }, .cycles = opcode.cycles[0] },
            .POP => .{ .op = Op{ .pop = r16 }, .cycles = opcode.cycles[0] },
        };
    }
};

pub const Load = struct {
    src: LoadOperand,
    dest: LoadOperand,
    cycles: usize,
    bytes: usize,

    pub fn log(self: Load) void {
        var src_str: [16]u8 = undefined;
        var dest_str: [16]u8 = undefined;

        // Log destination
        const src = switch (self.src) {
            .r8 => |r| std.fmt.bufPrint(&src_str, "r8({s})", .{@tagName(r)}),
            .r16 => |r| std.fmt.bufPrint(&src_str, "r16({s})", .{@tagName(r)}),
            .address => |a| std.fmt.bufPrint(&src_str, "addr({s})", .{@tagName(a)}),
            .imm8 => std.fmt.bufPrint(&src_str, "imm8", .{}),
            .imm16 => std.fmt.bufPrint(&src_str, "imm16", .{}),
            .sp_offset => std.fmt.bufPrint(&src_str, "sp+offset", .{}),
        } catch unreachable;

        // Log source
        const dest = switch (self.dest) {
            .r8 => |r| std.fmt.bufPrint(&dest_str, "r8({s})", .{@tagName(r)}),
            .r16 => |r| std.fmt.bufPrint(&dest_str, "r16({s})", .{@tagName(r)}),
            .address => |a| std.fmt.bufPrint(&dest_str, "addr({s})", .{@tagName(a)}),
            .imm8 => std.fmt.bufPrint(&dest_str, "imm8", .{}),
            .imm16 => std.fmt.bufPrint(&dest_str, "imm16", .{}),
            .sp_offset => std.fmt.bufPrint(&dest_str, "sp+offset", .{}),
        } catch unreachable;

        std.log.info("LD {s} {s}", .{ dest, src });
    }

    fn from_json_opcode(opcode: JsonOpcode) !Load {
        const dest = try LoadOperand.from_json_operand(opcode.operands[0]);
        var src = try LoadOperand.from_json_operand(opcode.operands[1]);

        // quick check for edge case
        if (std.meta.eql(dest, LoadOperand{ .r16 = cpu.Register16.hl }) and std.meta.eql(src, LoadOperand{ .r16 = cpu.Register16.sp })) {
            src = LoadOperand{ .sp_offset = {} };
        }

        return Load{
            .src = src,
            .dest = dest,
            .cycles = opcode.cycles[0],
            .bytes = opcode.bytes,
        };
    }
};

pub const AddOperand = union(enum) {
    r8: cpu.Register8,
    r16: cpu.Register16,
    imm8: void,
    e8: void,

    pub fn from_json_operand(op: JsonOperand) !AddOperand {
        const Case = enum { a, b, c, d, e, h, l, n8, e8, hl, bc, de, sp };
        const case = std.meta.stringToEnum(Case, op.name) orelse return error.InvalidOperandName;

        return switch (case) {
            .a,
            .b,
            .c,
            .d,
            .e,
            .h,
            .l,
            => .{ .r8 = try cpu.Register8.from_str(op.name) },
            .n8 => .{ .imm8 = {} },
            .e8 => .{ .e8 = {} },
            .bc, .de, .sp => .{ .r16 = try cpu.Register16.from_str(op.name) },
            .hl => if (op.immediate) .{ .r16 = cpu.Register16.hl } else .{ .r8 = .hl },
        };
    }
};

pub const Add = struct {
    src: AddOperand,
    dest: AddOperand,
    with_carry: bool,
    cycles: usize,
    bytes: usize,

    pub fn from_json_opcode(op: JsonOpcode) !Add {
        return Add{ .dest = try AddOperand.from_json_operand(op.operands[0]), .src = try AddOperand.from_json_operand(op.operands[1]), .cycles = op.cycles[0], .bytes = op.bytes, .with_carry = std.mem.eql(u8, "ADC", op.mnemonic) };
    }
};

pub const Increment = struct {
    dest: union(enum) {
        r8: cpu.Register8,
        r16: cpu.Register16,
    },

    bytes: usize = 1,
    cycles: usize = 8,

    fn from_json_opcode(op: JsonOpcode) !Increment {
        const Case = enum { a, b, c, d, e, h, l, hl, bc, de, sp };
        const case = std.meta.stringToEnum(Case, op.operands[0].name) orelse return OpcodeError.InvalidDestName;
        return switch (case) {
            .a, .b, .c, .d, .e, .h, .l => .{ .dest = .{ .r8 = try cpu.Register8.from_str(op.operands[0].name) } },
            .bc, .de, .sp => .{ .dest = .{ .r16 = try cpu.Register16.from_str(op.operands[0].name) } },
            .hl => if (op.immediate) .{ .dest = .{ .r16 = cpu.Register16.hl } } else .{ .dest = .{ .r8 = .hl } },
        };
    }
};

pub const Decrement = Increment;

pub const ByteOperand = union(enum) {
    r8: cpu.Register8,
    imm8: void,

    pub fn from_json_operand(op: JsonOperand) !ByteOperand {
        const Case = enum { a, b, c, d, e, h, l, hl, n8 };
        const case = std.meta.stringToEnum(Case, op.name) orelse return error.InvalidOperandName;

        return switch (case) {
            .n8 => .{ .imm8 = {} },
            else => .{ .r8 = try cpu.Register8.from_str(op.name) },
        };
    }
};

pub const ByteArithmetic = struct {
    const ALUOps = union(enum) { SUB: ByteOperand, SBC: ByteOperand, AND: ByteOperand, OR: ByteOperand, XOR: ByteOperand, CP: ByteOperand, CCF: void, SCF: void, DAA: void, CPL: void };

    operation: ALUOps,

    bytes: usize,
    cycles: usize,

    pub fn from_json_opcode(op: JsonOpcode) !ByteArithmetic {
        const Case = std.meta.Tag(ALUOps);
        const case = std.meta.stringToEnum(Case, op.mnemonic) orelse return error.UnknownMnemonic;

        const operation = switch (case) {
            .SUB => ALUOps{ .SUB = try ByteOperand.from_json_operand(op.operands[1]) },
            .SBC => ALUOps{ .SBC = try ByteOperand.from_json_operand(op.operands[1]) },
            .AND => ALUOps{ .AND = try ByteOperand.from_json_operand(op.operands[1]) },
            .OR => ALUOps{ .OR = try ByteOperand.from_json_operand(op.operands[1]) },
            .XOR => ALUOps{ .XOR = try ByteOperand.from_json_operand(op.operands[1]) },
            .CP => ALUOps{ .CP = try ByteOperand.from_json_operand(op.operands[1]) },
            .CCF => ALUOps{ .CCF = {} },
            .SCF => ALUOps{ .SCF = {} },
            .DAA => ALUOps{ .DAA = {} },
            .CPL => ALUOps{ .CPL = {} },
        };

        return ByteArithmetic{
            .operation = operation,
            .bytes = op.bytes,
            .cycles = op.cycles[0],
        };
    }
};

pub const Jump = struct {
    pub const Condition = enum {
        unconditional,
        nz,
        z,
        nc,
        c,
    };

    const JumpOperand = union(enum) {
        hl: Condition,
        a16: Condition,
        e8: Condition,
    };

    operand: JumpOperand,
    bytes: usize,
    // cycles[0] = condition == true
    // cycles[1] = condition == false
    cycles: []const usize,

    pub fn log(self: Jump) void {
        std.log.debug("JUMP {s} {s}", .{ @tagName(self.operand), blk: {
            break :blk switch (self.operand) {
                .hl => |cc| @tagName(cc),
                .a16 => |cc| @tagName(cc),
                .e8 => |cc| @tagName(cc),
            };
        } });
    }

    pub fn from_json_opcode(op: JsonOpcode) !Jump {
        // first identify condition

        var operand_idx: u8 = 0;
        const cc = if (op.operands.len == 1) Condition.unconditional else blk: {
            operand_idx = 1;
            const Case = enum { nz, z, nc, c };
            const case = std.meta.stringToEnum(Case, op.operands[0].name) orelse return error.InvalidOperandName;
            break :blk @as(Condition, switch (case) {
                .nz => .nz,
                .z => .z,
                .nc => .nc,
                .c => .c,
            });
        };

        const operand_type = std.meta.stringToEnum(std.meta.Tag(JumpOperand), op.operands[operand_idx].name) orelse return error.InvalidOperandName;
        return Jump{
            .operand = switch (operand_type) {
                .hl => .{ .hl = cc },
                .a16 => .{ .a16 = cc },
                .e8 => .{ .e8 = cc },
            },
            // .operand = comptime @unionInit(JumpOperand, @tagName(operand_type), cc),
            .bytes = op.bytes,
            .cycles = op.cycles,
        };
    }
};

pub const Call = struct {
    const Condition = Jump.Condition;

    condition: Condition,
    bytes: usize,
    cycles: []const usize,

    fn from_json_opcode(op: JsonOpcode) !Call {
        // first identify condition

        var operand_idx: u8 = 0;
        const cc: Condition = if (op.operands.len == 1) .unconditional else blk: {
            operand_idx = 1;
            const Case = enum { nz, z, nc, c };
            const case = std.meta.stringToEnum(Case, op.operands[0].name) orelse return error.InvalidOperandName;
            break :blk @as(Condition, switch (case) {
                .nz => .nz,
                .z => .z,
                .nc => .nc,
                .c => .c,
            });
        };

        return Call{
            .condition = cc,
            .bytes = op.bytes,
            .cycles = op.cycles,
        };
    }
};

pub const Reset = struct {
    bytes: u8 = 1,
    cycles: u8 = 16,
    addr: u16,

    fn from_json_opcode(op: JsonOpcode) !Reset {
        // first identify condition
        const addr = try std.fmt.parseInt(u16, op.operands[0].name[1..], 16);
        return Reset{ .addr = addr };
    }
};

pub const Return = struct {
    const Condition = Jump.Condition;

    condition: Condition,
    bytes: usize,
    cycles: []const usize,

    fn from_json_opcode(op: JsonOpcode) !Return {
        // first identify condition

        var operand_idx: u8 = 0;
        const cc: Condition = if (op.operands.len == 0) .unconditional else blk: {
            operand_idx = 1;
            const Case = enum { nz, z, nc, c };
            const case = std.meta.stringToEnum(Case, op.operands[0].name) orelse return error.InvalidOperandName;
            break :blk @as(Condition, switch (case) {
                .nz => .nz,
                .z => .z,
                .nc => .nc,
                .c => .c,
            });
        };

        return Return{
            .condition = cc,
            .bytes = op.bytes,
            .cycles = op.cycles,
        };
    }
};

pub const InterruptControl = union(enum) {
    pub const EI = struct { bytes: usize = 1, cycles: usize = 4 };

    reti: Return,
    ei: EI,
    di: EI,

    pub fn from_json_opcode(op: JsonOpcode) !InterruptControl {
        const Case = enum { EI, DI, RETI };
        const case = std.meta.stringToEnum(Case, op.mnemonic) orelse return error.UnknownMnemonic;
        return switch (case) {
            .RETI => .{ .reti = Return{
                .condition = .unconditional,
                .bytes = 1,
                .cycles = &.{16},
            } },
            .EI => .{ .ei = EI{} },
            .DI => .{ .di = EI{} },
        };
    }
};

pub const Rotation = struct {
    const Direction = enum { Left, Right };

    direction: Direction,
    circular: bool,
    r8: cpu.Register8,
    bytes: usize,
    cycles: usize,

    pub fn from_json_opcode(op: JsonOpcode) !Rotation {
        const Case = enum { RLCA, RRCA, RLA, RRA, RLC, RRC, RL, RR };
        const case = std.meta.stringToEnum(Case, op.mnemonic) orelse return error.UnknownMnemonic;

        return switch (case) {
            .RLCA => Rotation{ .direction = .Left, .circular = true, .r8 = .a, .bytes = op.bytes, .cycles = op.cycles[0] },
            .RRCA => Rotation{ .direction = .Right, .circular = true, .r8 = .a, .bytes = op.bytes, .cycles = op.cycles[0] },
            .RLA => Rotation{ .direction = .Left, .circular = false, .r8 = .a, .bytes = op.bytes, .cycles = op.cycles[0] },
            .RRA => Rotation{ .direction = .Right, .circular = false, .r8 = .a, .bytes = op.bytes, .cycles = op.cycles[0] },
            .RLC => Rotation{ .direction = .Left, .circular = true, .r8 = std.meta.stringToEnum(cpu.Register8, op.operands[0].name) orelse return error.InvalidOperandName, .bytes = op.bytes, .cycles = op.cycles[0] },
            .RRC => Rotation{ .direction = .Right, .circular = true, .r8 = std.meta.stringToEnum(cpu.Register8, op.operands[0].name) orelse return error.InvalidOperandName, .bytes = op.bytes, .cycles = op.cycles[0] },
            .RL => Rotation{ .direction = .Left, .circular = false, .r8 = std.meta.stringToEnum(cpu.Register8, op.operands[0].name) orelse return error.InvalidOperandName, .bytes = op.bytes, .cycles = op.cycles[0] },
            .RR => Rotation{ .direction = .Right, .circular = false, .r8 = std.meta.stringToEnum(cpu.Register8, op.operands[0].name) orelse return error.InvalidOperandName, .bytes = op.bytes, .cycles = op.cycles[0] },
        };
    }
};

pub const Shift = struct {
    direction: Rotation.Direction,
    shift_type: enum { Logical, Arithmetic },
    r8: cpu.Register8,
    bytes: usize,
    cycles: usize,

    pub fn from_json_opcode(op: JsonOpcode) !Shift {
        const Case = enum { SLA, SRA, SRL };
        const case = std.meta.stringToEnum(Case, op.mnemonic) orelse return error.UnknownMnemonic;
        const r8 = try cpu.Register8.from_str(op.operands[0].name);

        return switch (case) {
            .SLA => Shift{ .direction = .Left, .shift_type = .Arithmetic, .r8 = r8, .bytes = op.bytes, .cycles = op.cycles[0] },
            .SRA => Shift{ .direction = .Right, .shift_type = .Arithmetic, .r8 = r8, .bytes = op.bytes, .cycles = op.cycles[0] },
            .SRL => Shift{ .direction = .Right, .shift_type = .Logical, .r8 = r8, .bytes = op.bytes, .cycles = op.cycles[0] },
        };
    }
};

// Handle bit testing and setting
pub const BitOperation = struct {
    const OperationType = enum { Test, Set, Reset };

    op: OperationType,
    bit: u3,
    r8: cpu.Register8,
    bytes: usize,
    cycles: usize,

    pub fn from_json_opcode(op: JsonOpcode) !BitOperation {
        const Case = enum { BIT, SET, RES };
        const case = std.meta.stringToEnum(Case, op.mnemonic) orelse return error.UnknownMnemonic;

        const bit = try std.fmt.parseInt(u3, op.operands[0].name, 10);
        const r8 = std.meta.stringToEnum(cpu.Register8, op.operands[1].name) orelse return error.InvalidOperandName;

        const operation: OperationType = switch (case) {
            .BIT => .Test,
            .SET => .Set,
            .RES => .Reset,
        };

        return BitOperation{ .op = operation, .bit = bit, .r8 = r8, .cycles = op.cycles[0], .bytes = op.bytes };
    }
};

pub const Swap = struct {
    r8: cpu.Register8,
    bytes: usize,
    cycles: usize,

    pub fn from_json_opcode(op: JsonOpcode) !Swap {
        const r8 = std.meta.stringToEnum(cpu.Register8, op.operands[0].name) orelse return error.InvalidOperandName;
        return Swap{ .r8 = r8, .cycles = op.cycles[0], .bytes = op.bytes };
    }
};

pub const Instruction = union(enum) {
    Nop: void,
    Load: Load,
    // Push/Pop
    StackOp: StackOperation,
    // ADD/ADC
    Add: Add,
    Inc: Increment,
    Dec: Decrement,
    ALUOp: ByteArithmetic,
    Jump: Jump,
    Call: Call,
    Reset: Reset,
    Return: Return,
    InterruptControl: InterruptControl,

    // CB Prefixed
    Rotate: Rotation,
    Shift: Shift,
    BitOp: BitOperation,
    Swap: Swap,

    pub fn from_json_opcode(op: JsonOpcode) !Instruction {
        const Instructions = enum { NOP, LD, LDH, PUSH, POP, ADD, INC, DEC, ADC, SUB, SBC, AND, OR, XOR, CP, CCF, SCF, DAA, CPL, JP, JR, CALL, RST, RET, EI, RETI, DI, RLCA, RRCA, RLA, RRA, RLC, RRC, RL, RR, SLA, SRA, SRL, BIT, SET, RES, SWAP };

        // if we dont support the instruction don't try to decode it
        const case = std.meta.stringToEnum(Instructions, op.mnemonic) orelse return error.UnknownMnemonic;

        return switch (case) {
            .NOP => .{ .Nop = {} },
            .LD, .LDH => .{ .Load = try Load.from_json_opcode(op) },
            .PUSH, .POP => .{ .StackOp = try StackOperation.from_json_opcode(op) },
            .ADD, .ADC => .{ .Add = try Add.from_json_opcode(op) },
            .INC => .{ .Inc = try Increment.from_json_opcode(op) },
            .DEC => .{ .Dec = try Decrement.from_json_opcode(op) },
            .SUB, .SBC, .AND, .OR, .XOR, .CP, .CCF, .SCF, .DAA, .CPL => .{ .ALUOp = try ByteArithmetic.from_json_opcode(op) },
            .JP, .JR => .{ .Jump = try Jump.from_json_opcode(op) },
            .CALL => .{ .Call = try Call.from_json_opcode(op) },
            .RST => .{ .Reset = try Reset.from_json_opcode(op) },
            .RET => .{ .Return = try Return.from_json_opcode(op) },
            .EI, .DI, .RETI => .{ .InterruptControl = try InterruptControl.from_json_opcode(op) },
            // CB Prefixed Opcodes
            .RLCA, .RRCA, .RLA, .RRA, .RLC, .RRC, .RL, .RR => .{ .Rotate = try Rotation.from_json_opcode(op) },
            .SLA, .SRA, .SRL => .{ .Shift = try Shift.from_json_opcode(op) },
            .BIT, .SET, .RES => .{ .BitOp = try BitOperation.from_json_opcode(op) },
            .SWAP => .{ .Swap = try Swap.from_json_opcode(op) },
        };
    }
};

const JsonOperand = struct {
    name: []const u8,
    bytes: ?u8 = null,
    immediate: bool,
};

const JsonOpcode = struct {
    mnemonic: []const u8,
    bytes: usize,
    cycles: []const usize,
    operands: []const JsonOperand,
    immediate: bool,
};

pub const Opcodes = struct {
    unprefixed: [256]JsonOpcode,
    cbprefixed: [256]JsonOpcode,
    inner_alloc: std.heap.ArenaAllocator,

    pub fn init(alloc: std.mem.Allocator) !Opcodes {
        var arena = std.heap.ArenaAllocator.init(alloc);
        errdefer arena.deinit();

        const allocator = arena.allocator();

        var parsed = try std.json.parseFromSliceLeaky(std.json.Value, allocator, opcode_json, .{ .ignore_unknown_fields = true, .allocate = .alloc_if_needed });
        const parsed_unprefixed = parsed.object.get("unprefixed").?.object;

        var json_op_iterator = parsed_unprefixed.iterator();
        var unprefixed: [256]JsonOpcode = undefined;

        for (0..256) |idx| {
            const entry = json_op_iterator.next() orelse {
                std.log.err("Json iterator stopped early at idx: {d}", .{idx});
                return error.IteratorEndedEarly;
            };

            const json_opcode = try std.json.parseFromValueLeaky(JsonOpcode, allocator, entry.value_ptr.*, .{ .ignore_unknown_fields = true, .allocate = .alloc_if_needed });
            unprefixed[idx] = json_opcode;
        }

        // CB-Prefixed parsing
        parsed = try std.json.parseFromSliceLeaky(std.json.Value, allocator, opcode_json, .{ .ignore_unknown_fields = true, .allocate = .alloc_if_needed });
        const parsed_prefixed = parsed.object.get("cbprefixed").?.object;

        json_op_iterator = parsed_prefixed.iterator();
        var prefixed: [256]JsonOpcode = undefined;

        for (0..256) |idx| {
            const entry = json_op_iterator.next() orelse {
                std.log.err("Json iterator stopped early at idx: {d}", .{idx});
                return error.IteratorEndedEarly;
            };

            const json_opcode = try std.json.parseFromValueLeaky(JsonOpcode, allocator, entry.value_ptr.*, .{ .ignore_unknown_fields = true, .allocate = .alloc_if_needed });
            prefixed[idx] = json_opcode;
        }

        return Opcodes{
            .unprefixed = unprefixed,
            .cbprefixed = prefixed,
            .inner_alloc = arena,
        };
    }

    pub fn deinit(self: *Opcodes) void {
        self.inner_alloc.deinit();
    }
};

// NOTE: This should be removed over time in favor of Opcodes struct
pub const UnprefixedOpcodes = struct {
    table: [256]JsonOpcode,
    inner_alloc: std.heap.ArenaAllocator,

    pub fn init(alloc: std.mem.Allocator) !UnprefixedOpcodes {
        var arena = std.heap.ArenaAllocator.init(alloc);
        errdefer arena.deinit();

        const allocator = arena.allocator();

        var parsed = try std.json.parseFromSliceLeaky(std.json.Value, allocator, unprefixed_json, .{ .ignore_unknown_fields = true, .allocate = .alloc_if_needed });
        const unprefixed = parsed.object.get("unprefixed").?.object;

        var json_op_iterator = unprefixed.iterator();
        var table: [256]JsonOpcode = undefined;

        for (0..256) |idx| {
            const entry = json_op_iterator.next() orelse {
                std.log.err("Json iterator stopped early at idx: {d}", .{idx});
                return error.IteratorEndedEarly;
            };

            const json_opcode = try std.json.parseFromValueLeaky(JsonOpcode, allocator, entry.value_ptr.*, .{ .ignore_unknown_fields = true, .allocate = .alloc_if_needed });
            table[idx] = json_opcode;
        }

        return UnprefixedOpcodes{
            .table = table,
            .inner_alloc = arena,
        };
    }

    pub fn deinit(self: *UnprefixedOpcodes) void {
        self.inner_alloc.deinit();
    }
};

const testing = std.testing;

fn parse_and_compare(slc: []const u8, expected: JsonOpcode) !void {
    const parsed = try json.parseFromSlice(JsonOpcode, testing.allocator, slc, .{ .ignore_unknown_fields = true, .allocate = .alloc_if_needed });
    defer parsed.deinit();

    try testing.expectEqualDeep(expected, parsed.value);
}

test "parse_unprefixed" {
    var unprefixed = try UnprefixedOpcodes.init(testing.allocator);
    defer unprefixed.deinit();

    var buf = std.ArrayList(u8).init(testing.allocator);
    defer buf.deinit();

    var successful_decodes: u16 = 0;
    var attempted_decodes: u16 = 0;

    const Instructions = enum { NOP, LD, LDH, PUSH, POP, ADD, ADC, INC, DEC, SUB, SBC, AND, OR, XOR, CP, CCF, SCF, DAA, CPL, JP, JR, CALL, RET };
    for (0..256) |opcode| {

        // if we dont support the instruction don't try to decode it
        const op = unprefixed.table[opcode];
        const case = std.meta.stringToEnum(Instructions, op.mnemonic) orelse {
            try std.json.stringify(op, .{}, buf.writer());
            std.log.warn("No attempt to decode: {s}", .{buf.items});

            buf.clearRetainingCapacity();
            continue;
        };

        attempted_decodes += 1;

        var decode_successful: bool = true;

        {
            errdefer { // on error print debuf info
                std.json.stringify(op, .{}, buf.writer()) catch unreachable;
                std.log.err("Instruction Not Parsed: {s}", .{buf.items});

                buf.clearRetainingCapacity();
                decode_successful = false;
            }

            _ = switch (case) {
                .NOP => {},
                .LD, .LDH => try Load.from_json_opcode(op),
                .PUSH, .POP => try StackOperation.from_json_opcode(op),
                .ADD, .ADC => try Add.from_json_opcode(op),
                .INC => try Increment.from_json_opcode(op),
                .DEC => try Decrement.from_json_opcode(op),
                .SUB, .SBC, .AND, .OR, .XOR, .CP, .CCF, .SCF, .DAA, .CPL => try ByteArithmetic.from_json_opcode(op),
                .JP, .JR => try Jump.from_json_opcode(op),
                .CALL => try Call.from_json_opcode(op),
                .RET => try Return.from_json_opcode(op),
            };
        }

        if (decode_successful) successful_decodes += 1 else continue;
    }
}

test "LoadOperand8bit - register operands" {
    const json_operands = [_]struct { JsonOperand, cpu.Register8 }{
        .{ JsonOperand{ .name = "a", .immediate = true }, .a },
        .{ JsonOperand{ .name = "b", .immediate = true }, .b },
        .{ JsonOperand{ .name = "c", .immediate = true }, .c },
        .{ JsonOperand{ .name = "d", .immediate = true }, .d },
        .{ JsonOperand{ .name = "e", .immediate = true }, .e },
        .{ JsonOperand{ .name = "h", .immediate = true }, .h },
        .{ JsonOperand{ .name = "l", .immediate = true }, .l },
    };

    for (json_operands) |json_op_tuple| {
        const json_op = json_op_tuple.@"0";
        const expected_r8 = json_op_tuple.@"1";

        const result = try LoadOperand.from_json_operand(json_op);
        try testing.expectEqual(LoadOperand.r8, @as(std.meta.Tag(LoadOperand), result));
        try testing.expectEqual(expected_r8, result.r8);
    }
}

test "LoadOperand - immediate8 operand" {
    const json_operand = JsonOperand{ .name = "n8", .immediate = true };
    const result = try LoadOperand.from_json_operand(json_operand);

    try testing.expectEqual(LoadOperand.imm8, @as(std.meta.Tag(LoadOperand), result));
}

test "LoadOperand - immediate16 operand" {
    const json_operand = JsonOperand{ .name = "n16", .immediate = true };
    const result = try LoadOperand.from_json_operand(json_operand);

    try testing.expectEqual(LoadOperand.imm16, @as(std.meta.Tag(LoadOperand), result));
}

test "LoadOperand - address operands" {
    const json_operands = [_]JsonOperand{
        .{ .name = "hl", .immediate = false },
        .{ .name = "bc", .immediate = false },
        .{ .name = "de", .immediate = false },
        .{ .name = "a8", .immediate = false },
        .{ .name = "a16", .immediate = false },
        .{ .name = "c", .immediate = false },
    };

    const expected_results = [_]LoadOperand{
        .{ .address = .hl },
        .{ .address = .bc },
        .{ .address = .de },
        .{ .address = .a8 },
        .{ .address = .a16 },
        .{ .address = .c },
    };

    for (json_operands, expected_results) |json_op, expected| {
        const result = try LoadOperand.from_json_operand(json_op);
        try testing.expectEqualDeep(expected, result);
    }
}

test "LoadOperand - invalid operand" {
    const json_operand = JsonOperand{ .name = "invalid", .immediate = true };
    try testing.expectError(OpcodeError.InvalidOperandName, LoadOperand.from_json_operand(json_operand));
}
