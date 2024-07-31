const std = @import("std");
const json = std.json;
const cpu = @import("cpu.zig");

const opcode_json = @embedFile("Opcodes.json");
const unprefixed = @embedFile("unprefixed.json");

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
    const Op = enum {
        push,
        pop,
    };

    r16: cpu.Register16,
    op: Op,

    fn from_json_opcode(opcode: JsonOpcode) !StackOperation {
        const Case = enum { PUSH, POP };
        const case = std.meta.stringToEnum(Case, opcode.mnemonic) orelse return OpcodeError.InvalidOperandName;
        const r16 = try cpu.Register16.from_str(opcode.operands[0].name);
        return switch (case) {
            .PUSH => .{ .op = Op.push, .r16 = r16 },
            .POP => .{ .op = Op.pop, .r16 = r16 },
        };
    }
};

pub const Load = struct {
    src: LoadOperand,
    dest: LoadOperand,
    cycles: usize,
    bytes: usize,

    // fn from_json_opcode_expr(opcode: JsonOpcode) !Load {
    //     if (opcode.immediate) {
    //         return Load{
    //             .dest = try LoadOperand.from_json_operand(opcode.operands[0]),
    //             .src = try LoadOperand.from_json_operand(opcode.operands[1]),
    //             .cycles = opcode.cycles[0],
    //             .bytes = opcode.bytes,
    //         };
    //     } else {

    //     }
    // }

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
    hl_addr: void,
    imm8: void,
    e8: void,

    pub fn from_json_operand(op: JsonOperand) !AddOperand {
        const Case = enum { a, b, c, d, e, h, l, n8, hl, bc, de, sp };
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
            .hl => if (op.immediate) .{ .r16 = cpu.Register16.hl } else .{ .hl_addr = {} },
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
        return Add{ .dest = AddOperand.from_json_operand(op.operands[0]), .src = AddOperand.from_json_operand(op.operands[1]), .cycles = op.cycles[0], .bytes = op.bytes, .with_carry = std.mem.eql(u8, "ADC", op.mnemonic) };
    }
};

pub const Increment = struct {
    dest: union(enum) {
        r8: cpu.Register8,
        r16: cpu.Register16,
        hl_addr: void,
    },

    bytes: usize = 1,
    cycles: usize = 8,

    fn from_json_opcode(op: JsonOpcode) !Increment {
        const Case = enum { a, b, c, d, e, h, l, hl, bc, de, sp };
        const case = std.meta.stringToEnum(Case, op.operands[0].name);
        return switch (case) {
            .a, .b, .c, .d, .e, .h, .l => .{ .r8 = try cpu.Register8.from_str(op.operands[0].name) },
            .bc, .de, .sp => .{ .r16 = try cpu.Register16.from_str(op.operands[0].name) },
            .hl => if (op.immedate) .{ .r16 = cpu.Register16.hl } else .{ .hl_addr = {} },
        };
    }
};

pub const Decrement = Increment;

pub const Instruction = union(enum) {
    Load: Load,
    // Push/Pop
    StackOp: StackOperation,
    // ADD/ADC
    Add: Add,
    Inc: Increment,
    Dec: Decrement,
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

const Opcodes = struct {
    unprefixed: [256]JsonOpcode,
    cbprefixed: [256]JsonOpcode,
};

pub fn make_jump_table() !Opcodes {
    var buf = [_]u8{0} ** 4096;
    var fba = std.heap.FixedBufferAllocator.init(buf[0..buf.len]);
    const allocator = fba.allocator();

    const parsed = try json.parseFromSlice(Opcodes, allocator, opcode_json, .{ .ignore_unknown_fields = true });
    return parsed.value;
}

const testing = std.testing;

fn parse_and_compare(slc: []const u8, expected: JsonOpcode) !void {
    const parsed = try json.parseFromSlice(JsonOpcode, testing.allocator, slc, .{ .ignore_unknown_fields = true, .allocate = .alloc_if_needed });
    defer parsed.deinit();

    try testing.expectEqualDeep(expected, parsed.value);
}

test "parse_unprefixed" {
    const UnprefixedOpcodes = struct {
        unprefixed: [256]JsonOpcode,
    };

    const parsed = try json.parseFromSlice(UnprefixedOpcodes, testing.allocator, unprefixed, .{ .ignore_unknown_fields = true, .allocate = .alloc_if_needed });
    defer parsed.deinit();
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

test "Parse all load instructions" {
    const UnprefixedOpcodes = struct {
        unprefixed: [256]JsonOpcode,
    };

    const parsed = try json.parseFromSlice(UnprefixedOpcodes, testing.allocator, unprefixed, .{ .ignore_unknown_fields = true, .allocate = .alloc_if_needed });
    defer parsed.deinit();

    var successful_decodes: usize = 0;
    var attempted_decodes: usize = 0;

    var buf = std.ArrayList(u8).init(testing.allocator);
    defer buf.deinit();

    for (parsed.value.unprefixed) |opcode| {

        // catch multiple opcodes that we are interested in testing
        const Instrs = enum { LD, LDH, PUSH, POP };
        const case = std.meta.stringToEnum(Instrs, opcode.mnemonic) orelse continue;

        attempted_decodes += 1;

        const decode_successful = switch (case) {
            .LD, .LDH => if (Load.from_json_opcode(opcode)) |_| true else |_| false,
            .PUSH, .POP => if (StackOperation.from_json_opcode(opcode)) |_| true else |_| false,
        };

        if (decode_successful) successful_decodes += 1 else {
            try std.json.stringify(opcode, .{}, buf.writer());
            std.log.err("Failed to decode: {s}", .{buf.items});

            buf.clearRetainingCapacity();
            continue;
        }
    }

    try testing.expectEqual(100, attempted_decodes);
    try testing.expectEqual(100, successful_decodes);
}
