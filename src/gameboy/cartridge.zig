/// Cartridge Wrapper
///
/// Exposes basic methods for reading/writing
///
const std = @import("std");
const Allocator = std.mem.Allocator;
const MBC1 = @import("cartidges/mbc1.zig");

pub const CartridgeError = error{ UnrecognizedMBCType, InvalidROMSize, InvalidRAMSize };

pub const Cartridge = union(enum) {
    MBC1: MBC1,
    // MBC2,
    // MBC3,
    // MBC5,
    // MBC6,
    // MBC7,
    // MMM01,
    // M161,
    // HuC1,
    // HuC3,

    /// Read from the cartridge
    pub fn read(self: *Cartridge, addr: u16) u8 {
        return switch (self.*) {
            .MBC1 => |*mbc| mbc.read(addr),
        };
    }

    /// Write to the cartridge
    pub fn write(self: *Cartridge, addr: u16, value: u8) void {
        switch (self.*) {
            .MBC1 => |*mbc| mbc.write(addr, value),
        }
    }

    /// initialize Cartridge
    pub fn init(allocator: Allocator, rom: []const u8) !Cartridge {
        // read cartridge header
        const cart_header = rom[0..0x150];
        const cart_type = cart_header[0x0147];
        const rom_banks: u16 = switch (cart_header[0x0148]) {
            0x0...0x8 => |val| 2 * (@as(u16, 1) << @intCast(val)),
            else => return error.InvalidROMSize,
        };

        const ram_banks: u8 = switch (cart_header[0x0149]) {
            0x0 => 0,
            0x2 => 1,
            0x3 => 4,
            0x4 => 16,
            0x5 => 8,
            else => return error.InvalidRAMSize,
        };

        return switch (cart_type) {
            // 0x00 => .{ .NoMBC = ROMOnly{ .rom = rom_data[0..0x8000] } },
            0x1...0x3 => .{ .MBC1 = try MBC1.init(allocator, rom_banks, ram_banks, rom) },
            // 0x5 | 0x6 => .MBC2,
            // 0xB...0xD => .MMM01,
            // 0x0F...0x13 => .MBC3,
            // 0x19...0x1E => .MBC5,
            // 0x20 => .MBC6,
            // 0x22 => .MBC7,
            // 0xFE => .HuC3,
            // 0xFF => .HuC1,
            else => return error.UnrecognizedMBCType,
        };
    }
};
