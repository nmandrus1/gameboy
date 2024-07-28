// Here is where Gameboy components will be included and glued together

const std = @import("std");
const opcodes = @import("opcodes.zig");
const CPU = @import("cpu.zig").CPU;

// Event scheduler for Gameboy
const Scheduler = struct {
    cpu: CPU,
    // register events by adding them to the queue, sorted by
    // when events need to be handled
    events: std.PriorityQueue(Event, void, Event.compare),

    // Event Object
    const Event = struct {
        when: usize,
        event_type: EventType,

        fn compare(_: void, a: Event, b: Event) std.math.Order {
            return std.math.compare(usize, a.when, b.when);
        }
    };

    const EventType = enum {
        UserInput,
    };

    // initialize scheduler, load first events
    fn init(self: *Scheduler, cpu: CPU) !void {}

    // run CPU until next
    fn run(self: *Scheduler) !void {
        main: while (true) {}
    }
};

pub fn run() !void {
    // std.debug.print("Does nothing rn \n", .{});
    const table = try opcodes.make_jump_table();
    std.debug.print("{any}", .{table});
}
