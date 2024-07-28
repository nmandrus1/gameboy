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
};

// Address knows what data it needs to build the address type
const Address = union(enum) {
    // 16 bit register
    r16: cpu.Register16,
    // 16 bit immediate data
    a16: void,
    // 8 bit immediate data
    a8: void,
    // 8 bit register, in 8 bit LD instructions
    // its only ever the C register
    r8: cpu.Register8,
};

// Instruction representing an 8bit LD
const Load8bit = struct {
    src: LoadOperand8bit,
    dest: LoadOperand8bit,
    cycles: usize,
    bytes: usize,

    fn from_json_opcode(op: JsonOpcode) !Load8bit {
        std.log.debug("Load8bit: {s} {s}, {s}", .{ op.mnemonic, op.operands[0].name, op.operands[1].name });

        return Load8bit{
            .src = try LoadOperand8bit.from_json_operand(op.operands[0]),
            .dest = try LoadOperand8bit.from_json_operand(op.operands[1]),
            .cycles = op.cycles[0],
            .bytes = op.bytes,
        };
    }
};

const LoadOperand8bit = union(enum) {
    r8: cpu.Register8,
    immediate: void,
    address: Address,

    // Parse an operand from the Loaded JSON file and will
    // interpret it as an operand for an 8 bit LD function
    fn from_json_operand(op: JsonOperand) !LoadOperand8bit {
        // TODO: Maybe expand this to handle 16 as well?
        if (op.immediate) {
            // operand is a register or byte
            const Case = enum {
                a,
                b,
                c,
                d,
                e,
                h,
                l,
                n8,
            };
            const case = std.meta.stringToEnum(Case, op.name) orelse return OpcodeError.InvalidOperandName;
            return switch (case) {
                .n8 => .immediate,
                else => .{ .r8 = try cpu.Register8.from_str(op.name) },
            };
        } else {
            // operand is an address created from one of these
            const Case = enum { a8, a16, hl, bc, de, c };
            const case = std.meta.stringToEnum(Case, op.name) orelse return OpcodeError.InvalidOperandName;
            return switch (case) {
                .a8 => .{ .address = Address.a8 },
                .a16 => .{ .address = Address.a16 },
                .c => .{ .address = Address{ .r8 = cpu.Register8.c } },
                else => .{ .address = Address{ .r16 = try cpu.Register16.from_str(op.name) } },
            };
        }
    }
};

// Instruction representing an 16bit LD
const Load16bit = struct {
    op: Load16Operation,
    cycles: usize,
    bytes: usize,

    fn from_json_opcode(op: JsonOpcode) !Load16bit {
        // std.log.debug("Load16bit: {s} {s}, {s}", .{ op.mnemonic, op.operands[0].name, op.operands[1].name });

        return Load16bit{
            .op = try Load16Operation.from_json_opcode(op),
            .cycles = op.cycles[0],
            .bytes = op.bytes,
        };
    }
};

const Load16Operation = union(enum) {
    PUSH: cpu.Register16,
    POP: cpu.Register16,
    LD: ReigsterLoad,

    const ReigsterLoad = struct { dest: LoadOperand16bit, src: LoadOperand16bit };

    fn from_json_opcode(op: JsonOpcode) !Load16Operation {
        // insane parsing code
        // TODO: Cleanup?

        const case = std.meta.stringToEnum(@typeInfo(Load16Operation).Union.tag_type.?, op.mnemonic) orelse return OpcodeError.InvalidOperandName;

        return switch (case) {
            .PUSH => .{ .PUSH = try cpu.Register16.from_str(op.operands[0].name) },
            .POP => .{ .POP = try cpu.Register16.from_str(op.operands[0].name) },
            .LD => .{
                .LD = blk: {
                    const SourceOperandName = enum { n16, sp, hl };
                    const src = std.meta.stringToEnum(SourceOperandName, op.operands[1].name) orelse return OpcodeError.InvalidSrcName;

                    const DestOperandName = enum {
                        bc,
                        de,
                        hl,
                        sp,
                        a16,
                    };
                    const dest_name = std.meta.stringToEnum(DestOperandName, op.operands[0].name) orelse return OpcodeError.InvalidDestName;

                    const result: ReigsterLoad = switch (src) {
                        // if the source is 16 bit immediate data,
                        // then we know the desination is a 16 bit register

                        .n16 => .{ .dest = .{ .r16 = try cpu.Register16.from_str(op.operands[0].name) }, .src = .{ .immediate = {} } },
                        .hl => .{ .dest = .{
                            .r16 = .sp,
                        }, .src = .{ .r16 = .hl } },
                        .sp => inner: {
                            break :inner switch (dest_name) {
                                .a16 => .{ .dest = .{ .address = Address.a16 }, .src = .{ .r16 = .sp } },
                                .hl => .{ .dest = .{ .r16 = .hl }, .src = .{ .sp_and_extra = {} } },
                                else => unreachable,
                            };
                        },
                    };

                    break :blk result;
                },
            },
        };
    }
};

const LoadOperand16bit = union(enum) {
    r16: cpu.Register16,
    // special case
    sp_and_extra: void,
    immediate: void,
    address: Address,

    // // Parse an operand from the Loaded JSON file and will
    // // interpret it as an operand for a 16 bit LD function
    // fn from_json_operand(op: JsonOperand) !LoadOperand16bit {
    //     if (op.immediate) {
    //         // operand is a register or byte
    //         const Case = enum { bc, de, hl, sp, a16, n16 };
    //         const case = std.meta.stringToEnum(Case, op.name) orelse return OpcodeError.InvalidOperandName;
    //         return switch (case) {
    //             .bc, .de, .hl, .af => .{ .r16 = try cpu.Register16.from_str(case) },
    //             .n16 => .immediate,
    //             else => .{ .r8 = try cpu.Register8.from_str(op.name) },
    //         };
    //     }
    // }
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

        const result = try LoadOperand8bit.from_json_operand(json_op);
        try testing.expectEqual(LoadOperand8bit.r8, @as(std.meta.Tag(LoadOperand8bit), result));
        try testing.expectEqual(expected_r8, result.r8);
    }
}

test "LoadOperand8bit - immediate operand" {
    const json_operand = JsonOperand{ .name = "n8", .immediate = true };
    const result = try LoadOperand8bit.from_json_operand(json_operand);
    try testing.expectEqual(LoadOperand8bit.immediate, @as(std.meta.Tag(LoadOperand8bit), result));
}

test "LoadOperand8bit - address operands" {
    const json_operands = [_]JsonOperand{
        .{ .name = "hl", .immediate = false },
        .{ .name = "bc", .immediate = false },
        .{ .name = "de", .immediate = false },
        .{ .name = "a8", .immediate = false },
        .{ .name = "a16", .immediate = false },
        .{ .name = "c", .immediate = false },
    };

    const expected_results = [_]LoadOperand8bit{
        .{ .address = .{ .r16 = .hl } },
        .{ .address = .{ .r16 = .bc } },
        .{ .address = .{ .r16 = .de } },
        .{ .address = .a8 },
        .{ .address = .a16 },
        .{ .address = .{ .r8 = .c } },
    };

    for (json_operands, expected_results) |json_op, expected| {
        const result = try LoadOperand8bit.from_json_operand(json_op);
        try testing.expectEqualDeep(expected, result);
    }
}

test "LoadOperand8bit - invalid operand" {
    const json_operand = JsonOperand{ .name = "invalid", .immediate = true };
    try testing.expectError(OpcodeError.InvalidOperandName, LoadOperand8bit.from_json_operand(json_operand));
}

test "Parse 8bit load instructions" {
    const UnprefixedOpcodes = struct {
        unprefixed: [256]JsonOpcode,
    };

    const parsed = try json.parseFromSlice(UnprefixedOpcodes, testing.allocator, unprefixed, .{ .ignore_unknown_fields = true, .allocate = .alloc_if_needed });
    defer parsed.deinit();

    var successful_decodes: usize = 0;
    for (parsed.value.unprefixed) |opcode| {
        if (std.mem.eql(u8, opcode.mnemonic, "LD") or std.mem.eql(u8, opcode.mnemonic, "LDH")) {
            _ = Load8bit.from_json_opcode(opcode) catch {
                // std.log.err("Error: {!} -- Failed to decode the following instruction: {s} {s}, {s}", .{ err, opcode.mnemonic, opcode.operands[0].name, opcode.operands[1].name });
                continue;
            };

            successful_decodes += 1;
        }
    }

    // std.debug.print("{d}/85 LD/LDH Instructions successfully decoded ", .{successful_decodes});
    try testing.expectEqual(85, successful_decodes);
}

test "Parse 16bit load instructions" {
    const UnprefixedOpcodes = struct {
        unprefixed: [256]JsonOpcode,
    };

    const parsed = try json.parseFromSlice(UnprefixedOpcodes, testing.allocator, unprefixed, .{ .ignore_unknown_fields = true, .allocate = .alloc_if_needed });
    defer parsed.deinit();

    var successful_decodes: usize = 0;
    for (parsed.value.unprefixed) |opcode| {

        // catch multiple opcodes that we are interested in testing
        const Instrs = enum { LD, PUSH, POP };
        _ = std.meta.stringToEnum(Instrs, opcode.mnemonic) orelse continue;

        _ = Load16bit.from_json_opcode(opcode) catch continue;

        successful_decodes += 1;
    }

    // std.debug.print("{d}/85 LD/LDH Instructions successfully decoded ", .{successful_decodes});
    try testing.expectEqual(15, successful_decodes);
}
