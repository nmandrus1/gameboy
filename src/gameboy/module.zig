/// System Module
///
/// This module facilitates system interaction, coordinates read/writes across
/// different hardware components, and drives the event system/interrupts
///
const std = @import("std");
const Allocator = std.mem.Allocator;
const CPU = @import("cpu.zig");
const PPU = @import("ppu.zig");
const Bus = @import("bus.zig");

const System = struct {
    cpu: CPU,
    bus: Bus,
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
    fn runUntilNextEvent(self: *Scheduler, cpu: *CPU) EventType {
        const next = self.events.removeOrNull().?;
        while (self.cycles < next.when) : (self.cycles += cpu.step()) {
            if (cpu.pc == 0xFFFF) return .Quit;
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
    fn run(sys: *System) !void {
        // initialize scheduler, load first events

        var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
        defer arena.deinit();

        const allocator = arena.allocator();
        var scheduler = Scheduler{
            .events = EventQueue.init(allocator, {}),
            .cycles = 0,
        };

        // load first events
        scheduler.queue(.SerialTransfer, 1000000);

        // main loop
        while (true) {
            const event = scheduler.runUntilNextEvent(&sys.cpu);
            switch (event) {
                .SerialTransfer => {
                    sys.bus.writer.flush() catch unreachable;
                    scheduler.queue(.SerialTransfer, 1000000);
                },
                .Quit => return,
                .Vblank => scheduler.queue(.Vblank, 70000),
                else => {},
            }
        }
    }
};

pub fn run(rom: std.fs.File, allocator: std.mem.Allocator) !void {

    // Using with stdout
    // var stdout = std.io.getStdOut();
    // const buffered = std.io.bufferedWriter(stdout.writer());

    var bus = try Bus.init(allocator, rom);

    var cpu = try CPU.init(allocator, &bus);
    defer cpu.deinit();

    // build system
    var system = System{
        .cpu = cpu,
        .bus = bus,
    };

    try Scheduler.run(&system);
}
