const std = @import("std");

pub const CartridgeError = error{ UnrecognizedMBCType, InvalidROMSize, InvalidRAMSize };

pub const Cartridge = union(enum) {
    NoMBC: ROMOnly,
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

    pub fn init(rom: std.fs.File, allocator: std.mem.Allocator) !Cartridge {
        // Process Header

        // Read at most 8MB of ROM data
        const rom_data = try rom.readToEndAlloc(allocator, 0x800000);

        // ref bank 0
        const bank0 = rom_data[0..0x4000];

        return switch (bank0[0x0147]) {
            0x00 => .{ .NoMBC = ROMOnly{ .rom = rom_data[0..0x8000] } },
            0x1...0x3 => .{ .MBC1 = MBC1.init(rom_data, allocator) },

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

        return Cartridge{ .rom_data = rom_data, .bank0 = bank0, .switchable_bank = switchable_bank, .ctype = ctype, .rom_size = rom_size, .ram_data = ram_data, .ram_size = ram_size };
    }

    pub fn read(_: Cartridge, _: u16) u8 {
        return 0xAA;
    }

    pub fn readRAM(_: Cartridge, _: u16) u8 {
        return 0xAA;
    }
};

const ROMOnly = struct {
    rom: u8[0x8000],
};

const MBC1 = struct {
    // fixed bank
    bank0: u8[0x4000] = u8{0} ** 0x4000,
    // slice of bytes from the ROM
    switchable_bank: []const u8,
    rom_data: []const u8,
    rom_size: u64,

    // Pointer to current RAM bank
    ram_bank: []const u8,
    // All RAM
    ram: []const u8,
    ram_size: u32,
    ram_enabled: bool = false,

    fn init(rom_data: []const u8, allocator: std.mem.Allocator) MBC1 {
        const rom_size = switch (bank0[0x0148]) {
            // NoMBC -> 32KiB -> 2 Banks
            1...8 => |val| 0x8000 * (1 << val),
            else => return error.InvalidROMSize,
        };

        const ram_size = switch (bank0[0x0149]) {
            0 => 0,
            2 => std.math.pow(u32, 2, 13),
            3 => std.math.pow(u32, 2, 15),
            4 => std.math.pow(u32, 2, 17),
            5 => std.math.pow(u32, 2, 16),
            else => return error.InvalidRAMSize,
        };

        const ram_data = try allocator.alloc(u8, ram_size);
    }
};
