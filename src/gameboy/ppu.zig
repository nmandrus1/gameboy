const std = @import("std");
const Bus = @import("bus.zig");

pub const PPU = @This();

/// Representation of Tile Data
const Tile = [16]u8;

/// Object Attribute Memory
const OAM = struct {
    const Attributes = packed struct {
        priority: bool = false,
        y_flip: bool = false,
        x_flip: bool = false,
        dmg_palette: bool = false,
        _dead_bits: u4 = 0,
        // CGB Only
        // bank: bool,
        // cgb_palette: u3,
    };

    // Object Attributes
    y: u8 = 0,
    x: u8 = 0,
    tile_index: u8 = 0,
    attributes: Attributes,
};

const PPUMode = enum(u2) {
    HBlank = 0, // Mode 0
    VBlank = 1, // Mode 1
    Scan = 2, // Mode 2
    Draw = 3, // Mode 3
};

const FifoState = enum {
    GetTile,
    GetDataLow,
    GetDataHigh,
    Sleep,
    Push,

    // advance State Machine
    fn next(self: *FifoState) FifoState {
        var next_state = @intFromEnum(self) + 1;
        if (next_state > 4) next_state = 0;
        self = @enumFromInt(next_state);
    }
};

const PPUState = union(PPUMode) {
    HBlank: void,
    VBlank: void,
    Scan: void,
    Draw: FifoState,
};

state: PPUMode,
bus: *Bus,

// 1 scanline = 456 dots
// 144 scanlines = 65664 dots
// 144 scanlines + VBlank => 65664 + 4560 = 70224
/// # of dots advanced in this frame
frame_dots: usize = 0,
scanline_dots: u16 = 0,

objects: [10]OAM = [_]OAM{OAM{}} ** 10,
tile: ?Tile = null,
tile_x: u5,
tile_y: u5,

/// Run the PPU for the specified number of dots
pub fn step(self: *PPU, dots: u8) void {
    // One frame: 70224 dots
    var remaining: u8 = dots;

    // run PPU
    while (remaining > 0) {
        remaining = switch (self.state) {
            // wait for this scanline to end
            .HBlank => self.waitForScanlineEnd(remaining),
            // wait for this frame to end
            .VBlank => self.waitForFrameEnd(remaining),
            // perform OAM Scan
            .Scan => blk: {
                // If this is the start of a Scan then perform OAM scan
                if (self.scanline_dots == 0) self.scan(remaining);
                // Then wait until we switch modes
                break :blk self.waitForOamScanEnd(remaining);
            },
            // Peform a FIFO step & advance state machine
            .Draw => |render_state| switch (render_state) {
                .GetTile => self.getTile(),
            },
        };
    }

    // update LCD Status
    self.bus.STAT.ppu_mode = @intFromEnum(self.state);
}

/// Advance the PPU state a certain number of dots
/// This function returns the number of leftover dots to compute this PPU step
/// a return value of 0 means that all dots were spend waiting on HBlank
fn waitForScanlineEnd(self: *PPU, dots: u8) usize {
    // Calculate dots until the end of the current HBlank
    if (self.scanline_dots + dots < 456) {
        self.scanline_dots += dots;
        return 0;
    } else {
        // Scanline ends somewhere in the next few dots

        // update state
        defer self.scanline_dots = 0;
        defer self.bus.LY += 1;

        self.frame_dots += 456;
        if (self.frame_dots < 65664) self.state = .Scan else self.state = .VBlank;

        // Example: This scanline has 450 dots already processed, and we have 10 more dots
        // we finish this scanline, and return 4 remaining dots of PPU time
        return (dots + self.scanline_dots) - 456;
    }
}

/// Advance the PPU state a certain number of dots
/// This function returns the number of leftover dots to compute this PPU step
/// a return value of 0 means that all dots were spend waiting on VBlank
fn waitForFrameEnd(self: *PPU, dots: u8) usize {
    // Calculate dots until the end of the current HBlank
    const next_frame = 70220;

    if (self.frame_dots + dots < next_frame) {
        // return
        self.frame_dots += dots;
        self.bus.LY = @divFloor(self.frame_dots, 456);
        return 0;
    } else {
        // Frame ends somewhere in the next few dots
        defer self.frame_dots = 0;

        // update state
        self.state = .Scan;
        self.bus.LY = 0;

        // calculate remaining dots for this step
        return (self.frame_dots + dots) - next_frame;
    }
}

/// Perform OAM scan
fn scan(self: *PPU) usize {
    // Cast the bytes into OAM objects
    const objects: []OAM = @ptrCast(self.bus.oam);
    var count = 0;
    for (objects) |obj| {
        if (obj.y == self.bus.LY and count < 10) {
            self.objects[count] = obj;
            count += 1;
        }
    }
}

/// Advance the PPU state a certain number of dots
/// This function returns the number of leftover dots to compute this PPU step
/// a return value of 0 means that all dots were spent on OAM scan
fn waitForOamScanEnd(self: *PPU, dots: u8) usize {
    // Calculate dots until the end of the current HBlank
    if (self.scanline_dots + dots < 80) {
        self.scanline_dots += dots;
        return 0;
    } else {
        // OAM Scan is done
        self.state = .{ .Draw = .GetTile };
        self.tile_x = 0;
        self.tile_y = 0;
        return (dots + self.scanline_dots) - 80;
    }
}

/// Gather the data necessary to compose the row of pixels to be rendered
fn prepareScanline(self: *PPU) void {
    const VRAM = self.bus.vram;
    const TILE_DATA: []Tile = @ptrCast(VRAM[0..0x1800]);
    const BLOCK0: [128]Tile = if (self.bus.LCDC.tiles == 1) TILE_DATA[0..0x800] else TILE_DATA[0x1000..0x1800];
    const BLOCK1: [128]Tile = TILE_DATA[0x0800..0x1000];

    const tile_row = (self.bus.LY + self.bus.SCY) / 8;
    const pixel_row = (self.bus.LY + self.bus.SCY) % 8;

    // fetch background (32x32 Tile Map)
    const bg_map = if (self.bus.LCDC.bg_map == 0) VRAM[0x1800..0x1C00] else VRAM[0x1C00..0x2000];
    const bg_map_row = bg_map[32 * tile_row .. 32 * (tile_row + 1)];
    var bg_tiles: [32]Tile = undefined;
    mapToTiles(bg_map_row, bg_tiles, BLOCK0, BLOCK1);

    // fetch window (32x32 Tile Map)
    const win_map = if (self.bus.LCDC.window_map == 0) VRAM[0x1800..0x1C00] else VRAM[0x1C00..0x2000];
    const win_map_row = win_map[32 * tile_row .. 32 * (tile_row + 1)];
    var win_tiles: [32]Tile = undefined;
    mapToTiles(win_map_row, win_tiles, BLOCK0, BLOCK1);

    // overlay window

    // add sprites
}

/// Helper function to
fn mapToTiles(tile_map: []const u8, tiles: []u8, block0: []const Tile, block1: []const Tile) void {
    for (tile_map, 0..) |id, index| {
        // id specifies which tile in the tile data we want
        if (id > 127) {
            tiles[index] = block1[id - 128];
        } else {
            tiles[index] = block0[id];
        }
    }
}

fn getTile(self: *PPU) u8 {

    // this step takes 2 dots
    return 2;
}
