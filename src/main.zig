const std = @import("std");
const gameboy = @import("gameboy");

pub const log_level = std.log.Level.debug;

pub fn main() !void {
    // Get an allocator
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    // Get the arguments
    const args = try std.process.argsAlloc(allocator);
    defer std.process.argsFree(allocator, args);

    // Check if enough arguments were provided
    if (args.len < 2) {
        std.debug.print("Usage: {s} [--interactive|-i] <rom_file>\n", .{args[0]});
        return error.InvalidArgument;
    }

    var interactive_mode = false;
    var filename: []const u8 = undefined;

    // Parse arguments
    if (std.mem.eql(u8, args[1], "--interactive") or std.mem.eql(u8, args[1], "-i")) {
        if (args.len < 3) {
            std.debug.print("Error: ROM file not specified.\n", .{});
            std.debug.print("Usage: {s} [--interactive|-i] <rom_file>\n", .{args[0]});
            return error.InvalidArgument;
        }
        interactive_mode = true;
        filename = args[2];
    } else {
        filename = args[1];
    }

    // Open the file
    const file = try std.fs.cwd().openFile(filename, .{});
    defer file.close();

    // Get the file size
    const file_size = try file.getEndPos();

    // Allocate memory for the ROM data
    const rom_data = try allocator.alloc(u8, file_size);
    defer allocator.free(rom_data);

    // Read the file contents into the slice
    const bytes_read = try file.readAll(rom_data);
    if (bytes_read != file_size) {
        return error.IncompleteRead;
    }

    // Run the emulator
    try gameboy.run(rom_data, allocator, interactive_mode);
}
