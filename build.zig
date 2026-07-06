const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const libinput_mod = b.addModule("libinput", .{
        .root_source_file = b.path("src/libinput.zig"),
        .target = target,
        .optimize = optimize,
    });

    const test_step = b.step("test", "Run tests");
    const tests = b.addTest(.{ .root_module = libinput_mod });
    test_step.dependOn(&b.addRunArtifact(tests).step);

    const uinput_mod = b.createModule(.{
        .root_source_file = b.path("src/libinput/uinput_test.zig"),
        .target = target,
        .optimize = optimize,
    });
    uinput_mod.addImport("libinput", libinput_mod);

    const uinput_tests = b.addTest(.{ .root_module = uinput_mod });
    test_step.dependOn(&b.addRunArtifact(uinput_tests).step);

    const example_mod = b.createModule(.{
        .root_source_file = b.path("examples/pointer_dump.zig"),
        .target = target,
        .optimize = optimize,
    });

    example_mod.addImport("libinput", libinput_mod);

    const example_exe = b.addExecutable(.{
        .name = "pointer_dump",
        .root_module = example_mod,
    });

    b.installArtifact(example_exe);
}
