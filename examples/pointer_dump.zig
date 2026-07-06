//! pointer_dump: open a real evdev device and print every libinput event.
//!
//! Usage: pointer_dump /dev/input/eventN
//!
//! Requires read access to the device node. Run as root or add the user
//! to the `input` group:
//!   sudo usermod -aG input $USER
//!
//! This is Slice 1 only (relative pointer devices: mice, trackpoints, wheels).

const std = @import("std");
const libinput = @import("libinput");

pub fn main(init: std.process.Init.Minimal) !void {
    var gpa_state: std.heap.DebugAllocator(.{}) = .init;
    defer _ = gpa_state.deinit();
    const alloc = gpa_state.allocator();

    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    const args = try init.args.toSlice(arena.allocator());

    if (args.len < 2) {
        std.debug.print("usage: pointer_dump /dev/input/eventN\n", .{});
        return;
    }

    const path: []const u8 = args[1];

    var ctx = libinput.Context.init(alloc);
    defer ctx.deinit();

    const handle = ctx.addDevice(path) catch |err| {
        std.debug.print("error: cannot open {s}: {}\n", .{ path, err });
        return;
    };

    // Optional per-device configuration.
    handle.setAccelProfile(.adaptive);
    handle.setAccelSpeed(0.0);
    handle.setNaturalScroll(false);
    handle.setMiddleEmulation(false);
    handle.setScrollMethod(.none);
    handle.setLeftHanded(false);

    std.debug.print("listening on {s}  (Ctrl-C to stop)\n\n", .{path});

    while (true) {
        ctx.dispatch() catch |err| {
            std.debug.print("dispatch error: {}\n", .{err});
            return;
        };

        while (ctx.next()) |ev| {
            switch (ev) {
                .pointer_motion => |m| std.debug.print(
                    "motion        dx={d:.3}  dy={d:.3}  " ++
                        "dx_unaccel={d:.3}  dy_unaccel={d:.3}\n",
                    .{ m.dx, m.dy, m.dx_unaccel, m.dy_unaccel },
                ),
                .pointer_button => |b| std.debug.print(
                    "button        btn={d}  state={s}\n",
                    .{ b.button, @tagName(b.state) },
                ),
                .pointer_axis => |a| std.debug.print(
                    "axis          axis={s}  value={d:.3}  source={s}  v120={d:.3}\n",
                    .{ @tagName(a.axis), a.value, @tagName(a.source), a.v120 },
                ),
                .key => |k| std.debug.print(
                    "key           code={d}  state={s}\n",
                    .{ k.code, @tagName(k.state) },
                ),
                .device_added => |d| std.debug.print(
                    "device_added  id={d}\n",
                    .{d.device_id},
                ),
                .device_removed => |d| {
                    std.debug.print(
                        "device_removed  id={d}  (device unplugged, exiting)\n",
                        .{d.device_id},
                    );
                    return;
                },
            }
        }

        // poll on an empty fd list is a portable 1ms sleep between reads.
        var no_fds = [0]std.posix.pollfd{};
        _ = std.posix.poll(&no_fds, 1) catch {};
    }
}
