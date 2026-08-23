//! TabletProcessor: accumulates stylus/tablet evdev events for one frame,
//! then on flush emits neutral tablet events (proximity, tip, axis).

const std = @import("std");
const evdev = @import("evdev.zig");
const event = @import("event.zig");

/// Normalize a raw axis value to [0, 1] using the given AbsRange.
/// Clamps the result to [0, 1] to guard against out-of-range hardware values.
pub fn normalize(raw: i32, range: evdev.AbsRange) f64 {
    // Cast to i64 before subtracting to avoid i32 overflow for adversarial
    // kernel-reported ranges (e.g. min = i32_min, max = i32_max).
    const span: i64 = @as(i64, range.max) - @as(i64, range.min);
    // Guard against degenerate ranges (max <= min).
    if (span <= 0) return 0.0;
    const num: i64 = @as(i64, raw) - @as(i64, range.min);
    const n = @as(f64, @floatFromInt(num)) / @as(f64, @floatFromInt(span));
    return std.math.clamp(n, 0.0, 1.0);
}

pub const TabletProcessor = struct {
    range_x: evdev.AbsRange,
    range_y: evdev.AbsRange,
    range_pressure: evdev.AbsRange,

    // Current raw values.
    raw_x: i32 = 0,
    raw_y: i32 = 0,
    raw_pressure: i32 = 0,

    // Current state.
    in_prox: bool = false,
    tip_down: bool = false,

    // Frame-dirty flags.
    prox_changed: bool = false,
    tip_changed: bool = false,
    axis_dirty: bool = false,

    pub fn init(range_x: evdev.AbsRange, range_y: evdev.AbsRange, range_pressure: evdev.AbsRange) TabletProcessor {
        return .{
            .range_x = range_x,
            .range_y = range_y,
            .range_pressure = range_pressure,
        };
    }

    pub fn feed(self: *TabletProcessor, e: evdev.InputEvent) void {
        switch (e.type) {
            evdev.EV_ABS => switch (e.code) {
                evdev.ABS_X => {
                    self.raw_x = e.value;
                    self.axis_dirty = true;
                },
                evdev.ABS_Y => {
                    self.raw_y = e.value;
                    self.axis_dirty = true;
                },
                evdev.ABS_PRESSURE => {
                    self.raw_pressure = e.value;
                    self.axis_dirty = true;
                },
                else => {},
            },
            evdev.EV_KEY => switch (e.code) {
                evdev.BTN_TOOL_PEN => {
                    self.in_prox = (e.value != 0);
                    self.prox_changed = true;
                },
                evdev.BTN_TOUCH => {
                    self.tip_down = (e.value != 0);
                    self.tip_changed = true;
                },
                else => {},
            },
            else => {},
        }
    }

    pub fn flush(
        self: *TabletProcessor,
        time_ms: u64,
        out: *std.ArrayList(event.Event),
        gpa: std.mem.Allocator,
    ) !void {
        defer self.clearFrame();

        if (self.prox_changed) {
            try out.append(gpa, .{ .tablet_proximity = .{
                .time_ms = time_ms,
                .in_prox = self.in_prox,
            } });
        }

        if (self.tip_changed) {
            try out.append(gpa, .{ .tablet_tip = .{
                .time_ms = time_ms,
                .down = self.tip_down,
            } });
        }

        if (self.axis_dirty) {
            try out.append(gpa, .{ .tablet_axis = .{
                .time_ms = time_ms,
                .x = normalize(self.raw_x, self.range_x),
                .y = normalize(self.raw_y, self.range_y),
                .pressure = normalize(self.raw_pressure, self.range_pressure),
            } });
        }
    }

    fn clearFrame(self: *TabletProcessor) void {
        self.prox_changed = false;
        self.tip_changed = false;
        self.axis_dirty = false;
    }
};

test "normalize: min, mid, max, and clamp" {
    const range = evdev.AbsRange{ .min = 0, .max = 1000 };

    // min -> 0.0
    try std.testing.expectEqual(@as(f64, 0.0), normalize(0, range));

    // mid -> 0.5
    try std.testing.expectApproxEqAbs(@as(f64, 0.5), normalize(500, range), 1e-9);

    // max -> 1.0
    try std.testing.expectEqual(@as(f64, 1.0), normalize(1000, range));

    // below min -> clamped to 0.0
    try std.testing.expectEqual(@as(f64, 0.0), normalize(-50, range));

    // above max -> clamped to 1.0
    try std.testing.expectEqual(@as(f64, 1.0), normalize(2000, range));
}

test "normalize: degenerate range (max <= min) returns 0" {
    const bad = evdev.AbsRange{ .min = 100, .max = 100 };
    try std.testing.expectEqual(@as(f64, 0.0), normalize(100, bad));
}
