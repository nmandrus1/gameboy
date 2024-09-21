// Here is where Gameboy components will be included and glued together
const std = @import("std");
const CPU = @import("cpu.zig").CPU;

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
            if (cpu.pc == cpu.memory.len) return .Quit;
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
        UserInput,
        SerialTransfer,
        Quit,
    };

    // run CPU until next
    fn run(cpu: *CPU) !void {
        // initialize scheduler, load first events

        var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
        defer arena.deinit();

        const allocator = arena.allocator();
        var scheduler = Scheduler{
            .events = EventQueue.init(allocator, {}),
            .cycles = 0,
        };

        // buffered I/O to stdout
        var stdout = std.io.getStdOut();
        var buffered_stdout = std.io.bufferedWriter(stdout.writer());
        defer buffered_stdout.flush() catch unreachable;
        const writer = buffered_stdout.writer();

        // load first events
        scheduler.queue(.SerialTransfer, 500);

        // main loop
        while (true) {
            const event = scheduler.runUntilNextEvent(cpu);
            switch (event) {
                .SerialTransfer => {
                    cpu.serialTransfer(writer);
                    scheduler.queue(.SerialTransfer, 500);
                },
                .UserInput => {},
                .Quit => return,
            }
        }
    }

    pub fn runInteractive(cpu: *CPU) !void {
        var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
        defer arena.deinit();

        const allocator = arena.allocator();
        var scheduler = Scheduler{
            .events = EventQueue.init(allocator, {}),
            .cycles = 0,
        };

        // buffered I/O to stdout
        var stdout = std.io.getStdOut();
        var buffered_stdout = std.io.bufferedWriter(stdout.writer());
        defer buffered_stdout.flush() catch unreachable;
        const writer = buffered_stdout.writer();

        // load first events
        scheduler.queue(.SerialTransfer, 500);

        const stdin = std.io.getStdIn().reader();
        var buf = [_]u8{0} ** 256;

        // main loop
        while (true) {
            // block execution until "enter" is pressed
            const event = blk: {
                const next = scheduler.events.removeOrNull().?;
                while (scheduler.cycles < next.when) {
                    if (cpu.pc == cpu.memory.len) break :blk .Quit;
                    _ = try stdin.readUntilDelimiter(&buf, '\n');
                    scheduler.cycles += cpu.step();

                    // quit emulator if we have passed arbitrary cycle limit (testing)
                    if (scheduler.cycles > 1000) break :blk .Quit;
                }
                break :blk next.event_type;
            };

            switch (event) {
                .SerialTransfer => {
                    cpu.serialTransfer(writer);
                    scheduler.queue(.SerialTransfer, 500);
                },
                .UserInput => {},
                .Quit => return,
            }
        }
    }
};

pub fn run(rom: []const u8, allocator: std.mem.Allocator, interative: bool) !void {
    var cpu = try CPU.init(allocator);
    defer cpu.deinit();
    cpu.loadROM(rom);
    if (interative) try Scheduler.runInteractive(&cpu) else try Scheduler.run(&cpu);
}
