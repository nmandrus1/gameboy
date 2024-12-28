/// Module for Gameboy Memory
///
/// Provides an Abstraction over Memory Access for the whole system
///   - Maintains internal control over the Cartridge
///   - Maintains connection to PPU to facilitate access to VRAM
///   - Exposes basic methods for reading/writing
///
const std = @import("std");
const MBC1 = @import("cartidges/mbc1.zig");
