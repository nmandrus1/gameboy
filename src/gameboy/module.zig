// Here is where Gameboy components will be included and glued together
const std = @import("std");
const CPU = @import("cpu.zig").CPU;
const PPU = @import("ppu.zig").PPU;
const Cartridge = @import("cartridge.zig").Cartridge;

const System = struct {
    cpu: CPU,
    ppu: PPU,
    cart: Cartridge,
    scheduler: Scheduler,

    // Interactions
    writer: std.io.BufferedWriter(4096, std.fs.File.Writer) = std.io.bufferedWriter(std.io.getStdOut().writer()),

    // System Registers
    JOYP: u8 = 0,
    SB: u8 = 0,
    SC: u8 = 0,
    IF: u8 = 0,
    IE: u8 = 0,
    LCDC: u8 = 0,
    STAT: u8 = 0,
    SCY: u8 = 0,
    SCX: u8 = 0,
    LY: u8 = 0,
    LYC: u8 = 0,
    DMA: u8 = 0,
    BGP: u8 = 0,
    OBP0: u8 = 0,
    OBP1: u8 = 0,
    WY: u8 = 0,
    WX: u8 = 0,

    pub fn init(allocator: std.mem.Allocator, rom: std.fs.File) System {
        return System{
            .cpu = CPU.init(allocator),
            .ppu = PPU{},
            // Create Cartridge
            .cart = Cartridge.init(rom),
            .scheduler = Scheduler{},
        };
    }

    pub fn write(sys: *System, addr: u16, value: u8) void {
        switch (addr) {
            // ROM
            // 0x0000...0x7FFF => {},
            // VRAM
            0x8000...0x9FFF => if (sys.ppu.accessible()) sys.ppu.writeVRAM(addr - 0x8000, value),
            // Cartridge RAM
            // 0xA000...0xBFFF => {},
            // Work RAM
            0xC000...0xCFFF => sys.cpu.memory[addr - 0xC000] = value,
            0xD000...0xDFFF => sys.cpu.memory[addr - 0xD000] = value,
            // Echo RAM (mirror of C000-DDFF)
            // 0xE000...0xFDFF => {},
            // Object attribute memory (OAM)
            0xFE00...0xFE9F => if (sys.ppu.accessible()) sys.ppu.writeOAM(addr - 0xFE00, value),
            // Not Usable
            // 0xFEA0...0xFEFF => {},
            // I/O Registers
            0xFF00 => sys.JOYP = value,
            0xFF01 => sys.SB = value,
            0xFF02 => if (value == 0x81) sys.cpu.serialTransfer(),
            0xFF0F => sys.IF = value,
            0xFF40 => sys.LCDC = value,
            0xFF41 => sys.STAT = value,
            0xFF42 => sys.SCY = value,
            0xFF43 => sys.SCX = value,
            0xFF44 => {}, // LY is read-only
            0xFF45 => sys.LYC = value,
            0xFF46 => sys.DMA = value,
            0xFF47 => sys.BGP = value,
            0xFF48 => sys.OBP0 = value,
            0xFF49 => sys.OBP1 = value,
            0xFF4A => sys.WY = value,
            0xFF4B => sys.WX = value,
            // High RAM (HRAM)
            0xFF80...0xFFFE => sys.cpu.stack[0xFF8E - addr] = value,
            // Interrupt Enable register
            0xFFFF => sys.IE = value,
            else => {},
        }
    }

    pub fn read(sys: *System, addr: u16) u8 {
        return switch (addr) {
            // ROM
            0x0000...0x7FFF => sys.cart.read(addr),
            // VRAM
            0x8000...0x9FFF => sys.ppu.readVRAM(addr - 0x8000) orelse 0xFF, // Removed 'value' parameter
            // Cartridge RAM
            0xA000...0xBFFF => sys.cart.readRAM(addr),
            // Work RAM
            0xC000...0xCFFF => sys.cpu.memory[addr - 0xC000],
            0xD000...0xDFFF => sys.cpu.memory[addr - 0xD000],
            // Echo RAM (mirror of C000-DDFF)
            0xE000...0xFDFF => 0xFF,
            // Object attribute memory (OAM)
            0xFE00...0xFE9F => sys.ppu.readOAM(addr - 0xFE00) orelse 0xFF, // Removed 'value' parameter
            // Not Usable
            0xFEA0...0xFEFF => 0xFF,
            // I/O Registers
            0xFF00 => sys.JOYP, // Changed self to sys
            0xFF01 => sys.SB,
            0xFF02 => sys.SC,
            0xFF04 => sys.DIV, // Added DIV register
            0xFF05 => sys.TIMA, // Added Timer registers
            0xFF06 => sys.TMA,
            0xFF07 => sys.TAC,
            0xFF0F => sys.IF,
            // Audio (returning 0xFF for now)
            0xFF10...0xFF26 => 0xFF,
            // Wave Pattern
            0xFF30...0xFF3F => 0xFF,
            // PPU Registers
            0xFF40 => sys.LCDC,
            0xFF41 => sys.STAT,
            0xFF42 => sys.SCY,
            0xFF43 => sys.SCX,
            0xFF44 => sys.LY, // Changed from {} to actual LY read
            0xFF45 => sys.LYC,
            0xFF46 => sys.DMA,
            0xFF47 => sys.BGP,
            0xFF48 => sys.OBP0,
            0xFF49 => sys.OBP1,
            0xFF4A => sys.WY,
            0xFF4B => sys.WX,
            // High RAM (HRAM)
            0xFF80...0xFFFE => sys.cpu.stack[0xFF8E - addr],
            // Interrupt Enable register (IE)
            0xFFFF => sys.IE,
            else => 0xFF,
        };
    }
};

// Event scheduler for Gameboy
const Scheduler = struct {
    // register events by adding them to the queue, sorted by
    // when events need to be handled
    const EventQueue = std.PriorityQueue(Event, void, Event.compare);
    events: EventQueue,

    // Keep internal track of cycles
    cycles: usize,

    /// Queue an Event to occur in a given number of cycles
    fn queue(self: *Scheduler, etype: EventType, in: usize) void {
        self.events.add(.{ .event_type = etype, .when = self.cycles + in }) catch unreachable;
    }

    /// Pop event from queue, and run the cpu until the event needs to be handled
    fn runUntilNextEvent(self: *Scheduler, cpu: anytype) EventType {
        const next = self.events.removeOrNull().?;
        while (self.cycles < next.when) : (self.cycles += cpu.step()) {
            if (cpu.pc == cpu.memory.len) return .Quit;
            // const interrupt = cpu.poll_interrupts() orelse continue;
        }
        return next.event_type;
    }

    // Event Object
    const Event = struct {
        when: usize,
        event_type: EventType,

        fn compare(_: void, a: Event, b: Event) std.math.Order {
            return std.math.order(a.when, b.when);
        }
    };

    const EventType = enum {
        Joypad,
        SerialTransfer,
        Vblank,
        Quit,
    };

    // run CPU until next
    fn run(cpu: anytype) !void {
        // initialize scheduler, load first events

        var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
        defer arena.deinit();

        const allocator = arena.allocator();
        var scheduler = Scheduler{
            .events = EventQueue.init(allocator, {}),
            .cycles = 0,
        };

        // const writer = buffered_stdout.writer();

        // load first events
        scheduler.queue(.SerialTransfer, 1000000);

        // main loop
        while (true) {
            const event = scheduler.runUntilNextEvent(cpu);
            switch (event) {
                .SerialTransfer => {
                    cpu.writer.flush() catch unreachable;
                    scheduler.queue(.SerialTransfer, 1000000);
                },
                .Quit => return,
                .Vblank => scheduler.queue(.Vblank, 70000),
                else => {},
            }
        }
    }
};

pub fn run(rom: []const u8, allocator: std.mem.Allocator) !void {

    // Using with stdout
    var stdout = std.io.getStdOut();
    const buffered = std.io.bufferedWriter(stdout.writer());
    var cpu = try CPU(@TypeOf(buffered)).init(allocator, buffered);
    defer cpu.deinit();

    cpu.loadROM(rom);

    try Scheduler.run(&cpu);
}
