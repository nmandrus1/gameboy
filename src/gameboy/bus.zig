/// Memory Bus for Gameboy Hardware
///
/// All system read/writes are directed to the Bus
/// All components that need memory reference the Bus
///
const std = @import("std");
const Cartridge = @import("cartridge.zig").Cartridge;

const Bus = @This();

pub const Interrupt = enum(u16) {
    Joypad = 0x0060,
    Serial = 0x0058,
    Timer = 0x0050,
    Lcd = 0x0048,
    Vblank = 0x0040,
};

pub const InterruptFlags = packed struct {
    _dead_bits: u3 = 0,
    joypad: bool = false,
    serial: bool = false,
    timer: bool = false,
    lcd: bool = false,
    vblank: bool = false,

    // checks if the flag is set for an interrupt request
    pub fn check(self: InterruptFlags, int: Interrupt) bool {
        return switch (int) {
            .Joypad => self.joypad,
            .Serial => self.serial,
            .Timer => self.timer,
            .Lcd => self.lcd,
            .Vblank => self.vblank,
        };
    }

    pub fn set(self: *InterruptFlags, int: Interrupt, value: bool) void {
        switch (int) {
            .Joypad => self.joypad = value,
            .Serial => self.serial = value,
            .Timer => self.timer = value,
            .Lcd => self.lcd = value,
            .Vblank => self.vblank = value,
        }
    }
};

/// Bit fields for STAT register
const LCDStatus = packed struct {
    _dead_bit: u1 = 0,
    // read/write
    lyc_select: u1 = 0,
    mode2_select: u1 = 0,
    mode1_select: u1 = 0,
    mode0_select: u1 = 0,
    // read only
    lyc_eq_ly: u1 = 0,
    ppu_mode: u2 = 0,
};

// 8 KiB VRAM + 8KiB of Work RAM + 158 bytes of OAM + 126 Bytes of HRAM
// NOTE: Writing to this field direcly is NEVER allowed
_all_ram: [0x2000 + 0x2000 + 0x9E + 0x7E]u8 = [_]u8{0} ** 0x411C,
// Initialize these slices to _all_ram in init()
vram: []u8 = undefined,
wram: []u8 = undefined,
oam: []u8 = undefined,
hram: []u8 = undefined,
cart: Cartridge,

// Interactions
writer: std.io.BufferedWriter(4096, std.fs.File.Writer) = std.io.bufferedWriter(std.io.getStdOut().writer()),

// System Registers
JOYP: u8 = 0,
SB: u8 = 0,
SC: u8 = 0,
DIV: u8 = 0,
TIMA: u8 = 0,
TMA: u8 = 0,
TAC: u8 = 0,
IF: InterruptFlags = .{},
IE: InterruptFlags = .{},
LCDC: u8 = 0,
STAT: LCDStatus = .{},
SCY: u8 = 0,
SCX: u8 = 0,
LY: u8 = 0x90,
LYC: u8 = 0,
DMA: u8 = 0,
BGP: u8 = 0,
OBP0: u8 = 0,
OBP1: u8 = 0,
WY: u8 = 0,
WX: u8 = 0,

pub fn init(allocator: std.mem.Allocator, rom_file: std.fs.File) !Bus {

    // read entire rom
    const rom = try rom_file.readToEndAlloc(allocator, 0x800000);

    var bus = Bus{
        .cart = try Cartridge.init(allocator, rom),
    };

    bus.vram = bus._all_ram[0..0x2000];
    bus.wram = bus._all_ram[0x2000..0x4000];
    bus.oam = bus._all_ram[0x4000..0x409E];
    bus.hram = bus._all_ram[0x409E..];

    return bus;
}

pub fn write(self: *Bus, addr: u16, value: u8) void {
    switch (addr) {
        // ROM
        0x0000...0x7FFF => self.cart.write(addr, value),
        // VRAM
        0x8000...0x9FFF => if (self.vramAccessible()) {
            self.vram[addr - 0x8000] = value;
        },
        // Cartridge RAM
        0xA000...0xBFFF => self.cart.write(addr, value),
        // Work RAM
        0xC000...0xDFFF => self.wram[addr - 0xC000] = value,
        // Echo RAM (mirror of C000-DDFF)
        0xE000...0xFDFF => self.wram[addr - 0xE000] = value,
        // Object attribute memory (OAM)
        0xFE00...0xFE9F => if (self.oamAccessible()) {
            self.oam[addr - 0xFE00] = value;
        },
        // Not Usable
        // 0xFEA0...0xFEFF => {},
        // I/O Registers
        0xFF00 => self.JOYP = value,
        0xFF01 => self.SB = value,
        0xFF02 => if (value == 0x81) self.handleSerialTransfer(),
        0xFF0F => self.IF = @bitCast(value),
        0xFF40 => self.LCDC = value,
        0xFF41 => self.writeLCDStatus(value),
        0xFF42 => self.SCY = value,
        0xFF43 => self.SCX = value,
        // LY is read-only
        0xFF44 => {},
        0xFF45 => self.LYC = value,
        0xFF46 => self.DMA = value,
        0xFF47 => self.BGP = value,
        0xFF48 => self.OBP0 = value,
        0xFF49 => self.OBP1 = value,
        0xFF4A => self.WY = value,
        0xFF4B => self.WX = value,
        // High RAM (HRAM)
        0xFF80...0xFFFE => self.hram[addr - 0xFF80] = value,
        // Interrupt Enable register
        0xFFFF => self.IE = @bitCast(value),
        else => {},
    }
}

pub fn read(self: *Bus, addr: u16) u8 {
    return switch (addr) {
        // ROM
        0x0000...0x7FFF => self.cart.read(addr),
        // VRAM
        0x8000...0x9FFF => if (self.vramAccessible()) self.vram[addr - 0x8000] else 0xFF,
        // Cartridge RAM
        0xA000...0xBFFF => self.cart.read(addr),
        // Work RAM
        0xC000...0xDFFF => self.wram[addr - 0xC000],
        // Echo RAM (mirror of C000-DDFF)
        0xE000...0xFDFF => 0xFF,
        // Object attribute memory (OAM)
        0xFE00...0xFE9F => if (self.oamAccessible()) self.oam[addr - 0xFE00] else 0xFF,
        // Not Usable
        0xFEA0...0xFEFF => 0xFF,
        // I/O Registers
        0xFF00 => self.JOYP, // Changed self to sys
        0xFF01 => self.SB,
        0xFF02 => self.SC,
        0xFF04 => self.DIV, // Added DIV register
        0xFF05 => self.TIMA, // Added Timer registers
        0xFF06 => self.TMA,
        0xFF07 => self.TAC,
        0xFF0F => @bitCast(self.IF),
        // Audio (returning 0xFF for now)
        0xFF10...0xFF26 => 0xFF,
        // Wave Pattern
        0xFF30...0xFF3F => 0xFF,
        // PPU Registers
        0xFF40 => self.LCDC,
        0xFF41 => @bitCast(self.STAT),
        0xFF42 => self.SCY,
        0xFF43 => self.SCX,
        0xFF44 => self.LY, // Changed from {} to actual LY read
        0xFF45 => self.LYC,
        0xFF46 => self.DMA,
        0xFF47 => self.BGP,
        0xFF48 => self.OBP0,
        0xFF49 => self.OBP1,
        0xFF4A => self.WY,
        0xFF4B => self.WX,
        // High RAM (HRAM)
        0xFF80...0xFFFE => self.hram[addr - 0xFF80],
        // Interrupt Enable register (IE)
        0xFFFF => @bitCast(self.IE),
        else => 0xFF,
    };
}

fn writeLCDStatus(self: *Bus, value: u8) void {
    const casted: LCDStatus = @bitCast(value);
    self.STAT.lyc_select = casted.lyc_select;
    self.STAT.mode2_select = casted.mode2_select;
    self.STAT.mode1_select = casted.mode1_select;
    self.STAT.mode0_select = casted.mode0_select;
}

fn vramAccessible(self: Bus) bool {
    return self.STAT.ppu_mode != 3;
}

fn oamAccessible(self: Bus) bool {
    return self.STAT.ppu_mode < 2;
}

/// helper function to write SB to buffer
fn handleSerialTransfer(self: *Bus) void {
    // std.log.debug("Serial Transfer Happening!", .{});

    // std.log.debug("Value of SC = {X}", .{serial_control});
    _ = self.writer.write(&.{self.SB}) catch unreachable;
    self.SC = 0x7F;
}
