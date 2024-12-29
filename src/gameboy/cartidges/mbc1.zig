/// Implementation of the MBC1 Cartidge for the Gameboy
///
const std = @import("std");
const Allocator = std.mem.Allocator;

const MBC1 = @This();

// Could add these for clarity
const BANK_SIZE = 0x4000;
const RAM_BANK_SIZE = 0x2000;
const MAX_ROM_BANKS = 128;
const MAX_RAM_BANKS = 4;

/// Bank typedef
const ROMBankPtr = *const [BANK_SIZE]u8;
const RAMBankPtr = *[RAM_BANK_SIZE]u8;

const BankingMode = enum {
    Simple,
    Advanced,
};

pub const MBCError = error{
    InvalidROMSize,
    InvalidRAMSize,
    InvalidRAMWithLargeROM,
};

// Up to 2MB of switchable 16KiB Memory Banks
const ROM = struct { data: []u8, banks: []ROMBankPtr, bank: u8 = 0 };

/// Up to 32 KiB of RAM that can be switched
const RAM = struct { data: []u8, banks: []RAMBankPtr, bank: u8 = 0 };

allocator: Allocator,

// Used to calculate the mask & to determine if we need secondary rom bank
bank_bits: u8,

rom: ROM,
ram: RAM,

ram_enabled: bool = false,
bank_mode: BankingMode = .Simple,

/// Allocate Memory Banks
pub fn init(alloc: Allocator, rom_banks: u16, ram_banks: u8, rom_data: []const u8) !MBC1 {
    // ROM size validation
    if (rom_banks < 2 or rom_banks > MAX_ROM_BANKS) {
        return error.InvalidROMSize;
    }

    // RAM size validation
    if (ram_banks > MAX_RAM_BANKS or ram_banks == 2 or ram_banks == 3) {
        return error.InvalidRAMSize;
    }

    // Large ROM (>512KB) can only have 0 or 1 RAM bank
    if (rom_banks > 32 and ram_banks > 1) {
        return error.InvalidRAMWithLargeROM;
    }

    var result = MBC1{ .bank_bits = @intCast(std.math.log2(rom_banks)), .allocator = alloc, .rom = .{
        .data = try alloc.dupe(u8, rom_data),
        .banks = try alloc.alloc(ROMBankPtr, rom_banks),
        .bank = 0,
    }, .ram = .{
        .data = try alloc.alloc(u8, @as(u16, ram_banks) * 0x2000),
        .banks = try alloc.alloc(RAMBankPtr, ram_banks),
        .bank = 0,
    } };

    // create a list of pointers to each ROM & RAM bank
    for (0..rom_banks) |i| result.rom.banks[i] = @ptrCast(&result.rom.data[i * BANK_SIZE]);
    for (0..ram_banks) |i| result.ram.banks[i] = @ptrCast(&result.ram.data[i * RAM_BANK_SIZE]);

    return result;
}

pub fn deinit(self: *MBC1) void {
    self.allocator.free(self.rom.data);
    self.allocator.free(self.rom.banks);
    self.allocator.free(self.ram.data);
    self.allocator.free(self.ram.banks);
}

/// Read memory at the address
pub fn read(self: *MBC1, addr: u16) u8 {
    return switch (addr) {
        0x0000...0x3FFF => self.rom.banks[0][addr],
        0x4000...0x7FFF => self.selectedROMBank()[addr - 0x4000],
        0xA000...0xBFFF => if (self.ram_enabled and self.ram.banks.len > 0) self.selectedRAMBank()[addr - 0xA000] else 0xFF,
        else => 0xFF,
    };
}

/// Write memory to the address
pub fn write(self: *MBC1, addr: u16, value: u8) void {
    switch (addr) {
        // RAM Enable
        0x0000...0x1FFF => self.ram_enabled = (value & 0xF) == 0xA,
        // ROM Bank switch
        0x2000...0x3FFF => self.selectROMBank(value),
        // SecondaryBank switch
        0x4000...0x5FFF => self.selectSecondaryBank(value),
        0x6000...0x7FFF => self.bank_mode = if (value & 1 == 1) .Advanced else .Simple,
        else => {},
    }
}

/// Helper function to return a pointer to the start of the current ROMBank
fn selectedROMBank(self: *MBC1) ROMBankPtr {
    return self.rom.banks[self.rom.bank];
}

// Should handle the complete bank selection:
fn selectROMBank(self: *MBC1, value: u8) void {
    // Handle bank 0 -> 1 translation
    var bank = if (value == 0) 1 else value;
    // Apply 5-bit mask
    bank &= 0x1F;
    // In large ROM mode, combine with upper bits
    if (self.bank_bits > 5) {
        bank |= (self.ram.bank << 5);
    }
    self.rom.bank = bank;
}

/// Helper function to return a pointer to the start of the current ROMBank
fn selectedRAMBank(self: *MBC1) RAMBankPtr {
    const bank = if (self.bank_mode == .Advanced) self.ram.bank else 0;
    return self.ram.banks[bank];
}

fn selectSecondaryBank(self: *MBC1, value: u8) void {
    const bank = value & 0b11;
    if (self.bank_bits > 5 and self.ram.banks.len == 0) {
        // Large ROM mode - value affects ROM bank upper bits
        self.ram.bank = bank;
        self.selectROMBank(self.rom.bank); // Recalculate ROM bank
    } else {
        // Normal mode - value affects RAM bank
        self.ram.bank = bank;
    }
}

const testing = std.testing;

test "MBC1 - ROM Banking Basics" {
    // Create ROM data for 32 banks (32 * 16KB = 512KB)
    const rom_data = try testing.allocator.alloc(u8, 32 * 0x4000);
    defer testing.allocator.free(rom_data);

    var mbc = try MBC1.init(testing.allocator, 32, 0, rom_data);
    defer mbc.deinit();

    // Test bank 0 is always accessible at 0000-3FFF
    try testing.expectEqual(mbc.rom.banks[0][0], mbc.read(0x0000));
    try testing.expectEqual(mbc.rom.banks[0][0x3FFF], mbc.read(0x3FFF));

    // Test bank switching
    mbc.write(0x2000, 0x5); // Select bank 5
    try testing.expectEqual(mbc.rom.banks[5][0], mbc.read(0x4000));

    // Test bank 0 -> 1 translation
    mbc.write(0x2000, 0x0);
    try testing.expectEqual(1, mbc.rom.bank);
}

test "MBC1 - RAM Enable/Disable" {
    // Create ROM data for 2 banks (2 * 16KB = 32KB)
    const rom_data = try testing.allocator.alloc(u8, 2 * 0x4000);
    defer testing.allocator.free(rom_data);

    var mbc = try MBC1.init(testing.allocator, 2, 1, rom_data);
    defer mbc.deinit();

    // RAM should start disabled
    try testing.expect(!mbc.ram_enabled);

    // Write pattern to RAM
    mbc.ram.banks[0][0] = 0x42;

    // Reading disabled RAM should return 0xFF
    try testing.expectEqual(0xFF, mbc.read(0xA000));

    // Enable RAM
    mbc.write(0x0000, 0x0A);
    try testing.expect(mbc.ram_enabled);
    try testing.expectEqual(0x42, mbc.read(0xA000));

    // Disable RAM
    mbc.write(0x0000, 0x00);
    try testing.expect(!mbc.ram_enabled);
    try testing.expectEqual(0xFF, mbc.read(0xA000));
}

test "MBC1 - Banking Modes" {

    // Create ROM data for 32 banks (32 * 16KB = 512KB)
    const rom_data = try testing.allocator.alloc(u8, 32 * 0x4000);
    defer testing.allocator.free(rom_data);

    var mbc = try MBC1.init(testing.allocator, 32, 4, rom_data);
    defer mbc.deinit();

    // Test Simple Mode (default)
    try testing.expectEqual(.Simple, mbc.bank_mode);
    mbc.write(0x4000, 0x3); // Set RAM bank
    try testing.expectEqual(mbc.ram.banks[0], mbc.selectedRAMBank());

    // Test Advanced Mode
    mbc.write(0x6000, 0x1);
    try testing.expectEqual(.Advanced, mbc.bank_mode);
    try testing.expectEqual(mbc.ram.banks[3], mbc.selectedRAMBank());
}

test "MBC1 - Large ROM Banking" {

    // Create ROM data for 32 banks (32 * 16KB = 512KB)
    const rom_data = try testing.allocator.alloc(u8, 128 * 0x4000);
    defer testing.allocator.free(rom_data);

    var mbc = try MBC1.init(testing.allocator, 128, 0, rom_data);
    defer mbc.deinit();

    // Test upper bits of ROM bank number
    mbc.write(0x2000, 0x1F); // Set lower 5 bits
    mbc.write(0x4000, 0x03); // Set upper 2 bits
    try testing.expectEqual(0x7F, mbc.rom.bank);

    // Test bank 0x20/0x40/0x60 quirk
    mbc.write(0x2000, 0x00);
    mbc.write(0x4000, 0x01); // Try to select bank 0x20
    try testing.expectEqual(0x21, mbc.rom.bank);
}

test "MBC1 - RAM/ROM Size Validations" {
    // Test valid configurations first
    {
        // Valid: 32 ROM banks (512KB) with 4 RAM banks (32KB)
        const rom_data = try testing.allocator.alloc(u8, 32 * 0x4000);
        defer testing.allocator.free(rom_data);

        var mbc = try MBC1.init(testing.allocator, 32, 4, rom_data);
        defer mbc.deinit();
        try testing.expectEqual(mbc.ram.banks.len, 4);
    }

    {
        // Valid: 64 ROM banks (1MB) with 1 RAM bank (8KB)
        const rom_data = try testing.allocator.alloc(u8, 64 * 0x4000);
        defer testing.allocator.free(rom_data);

        var mbc = try MBC1.init(testing.allocator, 64, 1, rom_data);
        defer mbc.deinit();
        try testing.expectEqual(mbc.ram.banks.len, 1);
    }

    // Test invalid ROM sizes
    {
        // Too small ROM
        const result = MBC1.init(testing.allocator, 1, 0, &.{});
        try testing.expectError(MBCError.InvalidROMSize, result);
    }
    {
        // Too large ROM
        const result = MBC1.init(testing.allocator, 129, 0, &.{});
        try testing.expectError(MBCError.InvalidROMSize, result);
    }

    // Test invalid RAM sizes
    {
        // Invalid RAM bank count (2 is not valid)
        const result = MBC1.init(testing.allocator, 32, 2, &.{});
        try testing.expectError(MBCError.InvalidRAMSize, result);
    }
    {
        // Invalid RAM bank count (3 is not valid)
        const result = MBC1.init(testing.allocator, 32, 3, &.{});
        try testing.expectError(MBCError.InvalidRAMSize, result);
    }
    {
        // Too many RAM banks
        const result = MBC1.init(testing.allocator, 32, 5, &.{});
        try testing.expectError(MBCError.InvalidRAMSize, result);
    }

    // Test invalid combinations
    {
        // Large ROM (>512KB) with 4 RAM banks
        const result = MBC1.init(testing.allocator, 64, 4, &.{});
        try testing.expectError(MBCError.InvalidRAMWithLargeROM, result);
    }
}
