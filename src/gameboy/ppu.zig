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

const PPUState = union(PPUMode) {
    HBlank: void,
    VBlank: void,
    Scan: void,
    // Keep a copy of a PixelFifo that manages its own state
    Draw: PixelFifo,
};

state: PPUMode,
bus: *Bus,

// 1 scanline = 456 dots
// 144 scanlines = 65664 dots
// 144 scanlines + VBlank => 65664 + 4560 = 70224
/// number of dots advanced in this frame
frame_dots: usize = 0,
scanline_dots: u16 = 0,

objects: [10]OAM = [_]OAM{OAM{}} ** 10,
tile: ?Tile = null,

/// Run the PPU for the specified number of dots
pub fn step(self: *PPU, dots: u8) void {
    // One frame: 70224 dots
    var remaining: u8 = dots;

    // run PPU
    while (remaining > 0) {
        // Subtract the number of dots consumed from the total budget for this step
        remaining -= switch (self.state) {
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
            .Draw => |fifo| switch (fifo.state) {
                .GetTile => fifo.getTile(),
                .GetDataLow => fifo.getTileLow(),
                .GetDataHigh => fifo.getTileHigh(),
            },
        };
    }

    // update LCD Status
    self.bus.STAT.ppu_mode = @intFromEnum(self.state);
}

/// Advance the PPU state a certain number of dots
/// This function returns the number of dots used to compute this PPU step
fn waitForScanlineEnd(self: *PPU, dots: u8) usize {
    // Calculate dots until the end of the current HBlank
    if (self.scanline_dots + dots < 456) {
        self.scanline_dots += dots;
        return dots;
    } else {
        // Scanline ends somewhere in the next few dots

        // update state
        defer self.scanline_dots = 0;
        defer self.bus.LY += 1;

        self.frame_dots += 456;
        if (self.frame_dots < 65664) self.state = .Scan else self.state = .VBlank;

        // Example: Suppose this scanline has 450 dots already processed, and we have 10 more dots.
        // We finish this scanline, and return 6 since that is how many dots we needed to
        // finish the scanline
        // 6 = 456 - 450
        return 456 - self.scanline_dots;
    }
}

/// Advance the PPU state a certain number of dots
/// This function returns the number of dots consumed during this step
fn waitForFrameEnd(self: *PPU, dots: u8) usize {
    // Calculate dots until the end of the current HBlank
    const next_frame = 70220;

    if (self.frame_dots + dots < next_frame) {
        self.frame_dots += dots;
        self.bus.LY = @divFloor(self.frame_dots, 456);
        return dots;
    } else {
        // Frame ends somewhere in the next few dots
        defer self.frame_dots = 0;

        // update state
        self.state = .Scan;
        self.bus.LY = 0;

        // calculate remaining dots for this step
        return next_frame - self.frame_dots;
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
        return dots;
    } else {
        // OAM Scan is done
        self.state = .{ .Draw = PixelFifo.init(self) };
        return 80 - self.scanline_dots;
    }
}

// PIXEL FIFO

const FifoState = enum {
    GetTile,
    GetDataLow,
    GetDataHigh,
    Sleep,
    Push,

    // advance State Machine
    fn next(self: *FifoState) void {
        var next_state = @intFromEnum(self) + 1;
        if (next_state > 4) next_state = 0;
        self = @enumFromInt(next_state);
    }
};

const PixelFifo = struct {
    state: FifoState = .GetTile,

    // These track the current tile being fetched
    bg_tile_x: u8 = 0,
    window_tile_x: u8 = 0,
    fetch_x: u8 = 0, // Position within the 32-tile map row

    // Current tile data being processed
    tile_id: u8,
    tile_low: u8,
    tile_high: u8,

    // FIFO for holding pixels (1 pixel = 2 bits)
    bg_fifo: [4]u2 = [_]u2{0} ** 4,

    // number of pixels in FIFO
    bg_fifo_size: u8 = 0,

    // reference to PPU data
    ppu: *const PPU,

    fn init(ppu: *const PPU) PixelFifo {
        var fifo = PixelFifo{};
        fifo.ppu = ppu;
        return fifo;
    }

    /// Fetch the tile that contains the pixel being pushed to the FIFO
    fn getTile(self: *PixelFifo) usize {
        const WX = self.ppu.bus.WX;
        const WY = self.ppu.bus.WY;
        const LY = self.ppu.bus.LY;
        const LCDC = self.ppu.bus.LCDC;
        const VRAM = self.ppu.bus.vram;

        // Check if the window is overlayed at the current pixel
        if (LCDC.window_enable and LY >= WY and self.fetch_x >= WX - 7) {
            // fetch window tile
            // determine tile map
            const win_map = if (LCDC.window_map == 0) VRAM[0x1800..0x1C00] else VRAM[0x1C00..0x2000];
            // get tile id
            const id = win_map[32 * WY + (WX - 7)];
            // fetch actual tile
            self.tile_id = id;
        } else {
            // fetch bg tile
            const SCY = self.ppu.bus.SCY;
            const SCX = self.ppu.bus.SCX;
            const bg_y = (LY + SCY) & 255;
            const bg_x = (SCX / 8) + self.fetch_x & 31;

            const bg_map = if (LCDC.bg_map == 0) VRAM[0x1800..0x1C00] else VRAM[0x1C00..0x2000];
            const id = bg_map[bg_y * 32 + bg_x];
            self.tile_id = id;
        }

        // done with this step, so advance the state
        self.state.next();
        self.fetch_x += 1;

        // consume 2 dots
        return 2;
    }

    // Attempt to read VRAM (if accessible) for Tile data
    fn getDataLow(self: *PixelFifo) usize {
        const VRAM = self.ppu.bus.vram;
        const TILE_DATA: []Tile = @ptrCast(VRAM[0..0x1800]);
        const BLOCK0: [128]Tile = if (self.ppu.bus.LCDC.tiles == 1) TILE_DATA[0..0x800] else TILE_DATA[0x1000..0x1800];
        const BLOCK1: [128]Tile = TILE_DATA[0x0800..0x1000];

        if (self.ppu.bus.vramAccessible()) {
            const tile = if (self.tile_id > 127) BLOCK1[self.tile_id - 128] else BLOCK0[self.tile_id];
            const pixel_row = self.ppu.bus.LY % 8;
            self.tile_low = tile[pixel_row * 2];
        } else {
            self.tile_low = 0xFF;
        }

        return 2;
    }

    /// Attempt to read VRAM (if accessible) for Tile data
    fn getDataHigh() usize {
        // TODO: Pandocs say something about pushing to FIFO here
        // NOTE: read tile through bus, for access check
        return 2;
    }

    fn push(self: *PixelFifo) usize {
        // Determine which row of 8 pixels in the Tile need to be pushed
        const pixel_row = self.ppu.bus.LY % 8;

        return 2;
    }
};
