const Tile = struct {
    const TileType = enum { Object, Background, Window };

    id: u4,
    tile_type: TileType,
};

const OAM = struct {
    const Attributes = packed struct {
        priority: bool,
        y_flip: bool,
        x_flip: bool,
        dmg_palette: bool,
        _dead_bits: u4,
        // CGB Only
        // bank: bool,
        // cgb_palette: u3,
    };

    // Object Attributes
    y: u8,
    x: u8,
    tile_index: u8,
    attributes: Attributes,
};

pub const PPU = struct {};
