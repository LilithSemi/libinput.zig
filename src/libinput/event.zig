//! Event types: pointer motion, buttons, keys, device notifications. Slice 1 covers relative-motion pointer events.

const std = @import("std");

test "Event union tags carry their payloads" {
    const e = Event{ .pointer_motion = .{
        .time_ms = 10,
        .dx = 1.5,
        .dy = -2.0,
        .dx_unaccel = 1.0,
        .dy_unaccel = -1.0,
    } };
    switch (e) {
        .pointer_motion => |m| try std.testing.expectEqual(@as(f64, 1.5), m.dx),
        else => try std.testing.expect(false),
    }
}

pub const ButtonState = enum { released, pressed };

pub const Axis = enum { vertical, horizontal };

pub const AxisSource = enum { wheel, continuous };

pub const PointerMotion = struct {
    time_ms: u64,
    dx: f64,
    dy: f64,
    dx_unaccel: f64,
    dy_unaccel: f64,
};

pub const PointerButton = struct {
    time_ms: u64,
    button: u32,
    state: ButtonState,
    seat_button_count: u32,
};

pub const PointerAxis = struct {
    time_ms: u64,
    axis: Axis,
    value: f64,
    source: AxisSource,
    v120: f64,
};

pub const Key = struct {
    time_ms: u64,
    code: u32,
    state: ButtonState,
};

pub const DeviceEvent = struct {
    device_id: u32,
};

pub const TabletProximity = struct {
    time_ms: u64,
    in_prox: bool,
};

pub const TabletTip = struct {
    time_ms: u64,
    down: bool,
};

pub const TabletAxis = struct {
    time_ms: u64,
    x: f64,
    y: f64,
    pressure: f64,
};

pub const Event = union(enum) {
    pointer_motion: PointerMotion,
    pointer_button: PointerButton,
    pointer_axis: PointerAxis,
    key: Key,
    device_added: DeviceEvent,
    device_removed: DeviceEvent,
    tablet_proximity: TabletProximity,
    tablet_tip: TabletTip,
    tablet_axis: TabletAxis,
};
