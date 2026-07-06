//! Root of the pure-Zig libinput reimplementation (one `libinput` module).
//! Consumers reach the pieces as namespaces: `libinput.evdev`, `libinput.event`,
//! `libinput.device`, etc. Slice 1 covers relative-motion pointer devices only.

const std = @import("std");

pub const evdev = @import("libinput/evdev.zig");
pub const source = @import("libinput/source.zig");
pub const event = @import("libinput/event.zig");
pub const filter = @import("libinput/filter.zig");
pub const pointer = @import("libinput/pointer.zig");
pub const device = @import("libinput/device.zig");
pub const context = @import("libinput/context.zig");

pub const Event = event.Event;
pub const Device = device.Device;
pub const Context = context.Context;

test {
    std.testing.refAllDecls(@This());
}
