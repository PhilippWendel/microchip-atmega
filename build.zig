const std = @import("std");
const microzig = @import("microzig");

pub const boards = @import("src/boards.zig");
pub const chips = @import("src/chips.zig");

pub fn build(b: *std.build.Builder) void {
    const optimize = b.standardOptimizeOption(.{});

    inline for (@typeInfo(boards).Struct.decls) |decl| {
        const exe = microzig.addEmbeddedExecutable(b, .{
            .name = @field(boards, decl.name).name ++ ".minimal",
            .source_file = .{
                .path = "test/programs/minimal.zig",
            },
            .backing = .{ .board = @field(boards, decl.name) },
            .optimize = optimize,
        });
        exe.installArtifact(b);
    }

    inline for (@typeInfo(boards).Struct.decls) |decl| {
        const exe = microzig.addEmbeddedExecutable(b, .{
            .name = @field(boards, decl.name).name ++ ".uart",
            .source_file = .{
                .path = "test/programs/uart.zig",
            },
            .backing = .{ .board = @field(boards, decl.name) },
            .optimize = optimize,
        });
        exe.installArtifact(b);
    }

    inline for (@typeInfo(chips).Struct.decls) |decl| {
        const exe = microzig.addEmbeddedExecutable(b, .{
            .name = @field(chips, decl.name).name ++ ".minimal",
            .source_file = .{
                .path = "test/programs/minimal.zig",
            },
            .backing = .{ .chip = @field(chips, decl.name) },
            .optimize = optimize,
        });
        exe.installArtifact(b);
    }
}
