const std = @import("std");
const json = std.json;

const opcode_json = @embedFile("Opcodes.json");

const Operand = struct {
    name: []u8,
    bytes: ?u8,
    immediate: bool,
};

const Opcode = struct {
    mnemonic: []u8,
    bytes: u8,
    cycles: u8,
    operands: [2]?Operand,
    immediate: bool,
};

const Opcodes = struct {
    unprefixed: [256]Opcode,
    prefixed: [256]Opcode,
};

fn make_jump_table() Opcodes {
    const buf = [_]u8 {0} ** 4096;
    const fba = std.heap.FixedBufferAllocator.init(&buf);
    var allocator = fba.allocator();

    const stream = json.parseFromSlice(Opcodes, allocator, opcode_json, .{.ignore_unknown_fields = true, .allocate = })
}
