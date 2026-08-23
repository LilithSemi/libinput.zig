//! Relative pointer processor: accumulates raw evdev events for one frame,
//! then on flush runs acceleration, middle-button emulation, button-scroll,
//! and appends resulting Events to the caller-supplied output list.

const std = @import("std");
const evdev = @import("evdev.zig");
const filter = @import("filter.zig");
const event = @import("event.zig");

pub const ScrollMethod = enum { none, on_button_down };

pub const Config = struct {
    middle_emulation: bool = false,
    scroll_method: ScrollMethod = .none,
    scroll_button: u32 = evdev.BTN_MIDDLE,
    natural_scroll: bool = false,
    left_handed: bool = false,
};

/// States for middle-button emulation, ported from enum evdev_middlebutton_state (evdev-middle-button.c).
const MidState = enum {
    idle,
    left_down,
    right_down,
    middle,
    left_up_pending,
    right_up_pending,
    passthrough,
    ignore_lr,
    ignore_l,
    ignore_r,
};

/// Events for the middle-button state machine, ported from enum evdev_middlebutton_event.
const MidEvent = enum { l_down, r_down, other, l_up, r_up, timeout, all_up };

const BtnScrollState = enum { idle, button_down_pending, scrolling };

const MAX_PENDING = 8;

const PendingButton = struct {
    button: u32,
    state: event.ButtonState,
};

/// 50ms timeout from MIDDLEBUTTON_TIMEOUT (src/evdev-middle-button.c).
/// If the second button does not arrive within this window, the held button is flushed.
const MID_EMU_TIMEOUT_MS: u64 = 50;

/// Each pending event expands to at most two synthesized events. The +2 covers a possible timeout flush.
const MAX_RESOLVED = MAX_PENDING * 2 + 2;

const SCROLL_START_THRESHOLD: f64 = 5.0;

pub const Processor = struct {
    fil: *filter.Filter,
    cfg: *const Config,

    // Frame accumulators (cleared after every flush).
    rel_x: i32 = 0,
    rel_y: i32 = 0,
    wheel: i32 = 0,
    hwheel: i32 = 0,
    wheel_hi: i32 = 0,
    hwheel_hi: i32 = 0,

    // Pending button events for this frame.
    pending_items: [MAX_PENDING]PendingButton = undefined,
    pending_len: usize = 0,

    // Pending keyboard key events for this frame.
    pending_keys: [MAX_PENDING]event.Key = undefined,
    pending_keys_len: usize = 0,

    // Middle-button emulation state machine (port of evdev-middle-button.c).
    mid_state: MidState = .idle,
    // Bitmask of currently-pressed buttons, bit N = (button - BTN_LEFT). Mirrors device->middlebutton.button_mask.
    mid_button_mask: u32 = 0,
    // Time the first button was pressed to arm the timeout. Mirrors device->middlebutton.first_event_time.
    mid_first_event_time: u64 = 0,

    // Button-scroll state machine.
    scroll_state: BtnScrollState = .idle,
    scroll_accum_x: f64 = 0.0,
    scroll_accum_y: f64 = 0.0,

    // Per-button held-count, indexed by (button - BTN_LEFT). Size 8 covers BTN_LEFT..BTN_TASK (0x110..0x117).
    // Updated after left-handed swap so the consumer gets a consistent count per emitted code.
    seat_btn_count: [8]u32 = [_]u32{0} ** 8,

    pub fn init(f: *filter.Filter, cfg: *const Config) Processor {
        return .{ .fil = f, .cfg = cfg };
    }

    pub fn feed(self: *Processor, e: evdev.InputEvent) void {
        switch (e.type) {
            evdev.EV_REL => switch (e.code) {
                evdev.REL_X => self.rel_x +%= e.value,
                evdev.REL_Y => self.rel_y +%= e.value,
                evdev.REL_WHEEL => self.wheel +%= e.value,
                evdev.REL_HWHEEL => self.hwheel +%= e.value,
                evdev.REL_WHEEL_HI_RES => self.wheel_hi +%= e.value,
                evdev.REL_HWHEEL_HI_RES => self.hwheel_hi +%= e.value,
                else => {},
            },
            evdev.EV_KEY => {
                const code = e.code;
                if (code < @as(u16, evdev.BTN_MISC)) {
                    // Keyboard key: raw code passthrough, framed on SYN_REPORT.
                    if (self.pending_keys_len < MAX_PENDING) {
                        self.pending_keys[self.pending_keys_len] = .{
                            .time_ms = 0, // set at flush from the frame timestamp
                            .code = @as(u32, code),
                            .state = if (e.value != 0) .pressed else .released,
                        };
                        self.pending_keys_len += 1;
                    }
                } else if (code >= @as(u16, evdev.BTN_LEFT) and code <= @as(u16, evdev.BTN_TASK)) {
                    const st: event.ButtonState = if (e.value != 0) .pressed else .released;
                    // Raw events stored verbatim. Middle-button emulation runs at flush time so the timeout is driven by the frame timestamp.
                    if (self.pending_len < MAX_PENDING) {
                        self.pending_items[self.pending_len] = .{
                            .button = @as(u32, code),
                            .state = st,
                        };
                        self.pending_len += 1;
                    }
                }
            },
            else => {},
        }
    }

    const MidOut = struct {
        buf: []PendingButton,
        len: usize = 0,

        fn post(self: *MidOut, button: u32, st: event.ButtonState) void {
            if (self.len < self.buf.len) {
                self.buf[self.len] = .{ .button = button, .state = st };
                self.len += 1;
            }
        }
    };

    // Mirrors middlebutton_set_state. Entering LEFT_DOWN or RIGHT_DOWN records the press time to arm the timeout.
    fn midSetState(self: *Processor, state: MidState, time_ms: u64) void {
        switch (state) {
            .left_down, .right_down => self.mid_first_event_time = time_ms,
            else => {},
        }
        self.mid_state = state;
    }

    // Port of evdev_middlebutton_handle_event. Returns true to suppress the physical event (rc == 1),
    // false to pass it through (rc == 0). Synthesized events are appended to o.
    fn midHandleEvent(self: *Processor, ev: MidEvent, time_ms: u64, o: *MidOut) bool {
        switch (self.mid_state) {
            .idle => switch (ev) {
                .l_down => self.midSetState(.left_down, time_ms),
                .r_down => self.midSetState(.right_down, time_ms),
                .other => return false,
                // l_up, r_up, timeout are state errors here. all_up is a no-op.
                .l_up, .r_up, .timeout, .all_up => {},
            },
            .left_down => switch (ev) {
                .r_down => {
                    o.post(evdev.BTN_MIDDLE, .pressed);
                    self.midSetState(.middle, time_ms);
                },
                .other => {
                    o.post(evdev.BTN_LEFT, .pressed);
                    self.midSetState(.passthrough, time_ms);
                    return false;
                },
                .l_up => {
                    o.post(evdev.BTN_LEFT, .pressed);
                    o.post(evdev.BTN_LEFT, .released);
                    self.midSetState(.idle, time_ms);
                },
                .timeout => {
                    o.post(evdev.BTN_LEFT, .pressed);
                    self.midSetState(.passthrough, time_ms);
                },
                // l_down, r_up, all_up are state errors.
                .l_down, .r_up, .all_up => {},
            },
            .right_down => switch (ev) {
                .l_down => {
                    o.post(evdev.BTN_MIDDLE, .pressed);
                    self.midSetState(.middle, time_ms);
                },
                .other => {
                    o.post(evdev.BTN_RIGHT, .pressed);
                    self.midSetState(.passthrough, time_ms);
                    return false;
                },
                .r_up => {
                    o.post(evdev.BTN_RIGHT, .pressed);
                    o.post(evdev.BTN_RIGHT, .released);
                    self.midSetState(.idle, time_ms);
                },
                .timeout => {
                    o.post(evdev.BTN_RIGHT, .pressed);
                    self.midSetState(.passthrough, time_ms);
                },
                // r_down, l_up, all_up are state errors.
                .r_down, .l_up, .all_up => {},
            },
            .middle => switch (ev) {
                .other => {
                    o.post(evdev.BTN_MIDDLE, .released);
                    self.midSetState(.ignore_lr, time_ms);
                    return false;
                },
                .r_up => {
                    o.post(evdev.BTN_MIDDLE, .released);
                    self.midSetState(.left_up_pending, time_ms);
                },
                .l_up => {
                    o.post(evdev.BTN_MIDDLE, .released);
                    self.midSetState(.right_up_pending, time_ms);
                },
                // l_down, r_down, timeout, all_up are state errors.
                .l_down, .r_down, .timeout, .all_up => {},
            },
            .left_up_pending => switch (ev) {
                .r_down => {
                    o.post(evdev.BTN_MIDDLE, .pressed);
                    self.midSetState(.middle, time_ms);
                },
                .other => {
                    self.midSetState(.ignore_l, time_ms);
                    return false;
                },
                .l_up => self.midSetState(.idle, time_ms),
                // l_down, r_up, timeout, all_up are state errors.
                .l_down, .r_up, .timeout, .all_up => {},
            },
            .right_up_pending => switch (ev) {
                .l_down => {
                    o.post(evdev.BTN_MIDDLE, .pressed);
                    self.midSetState(.middle, time_ms);
                },
                .other => {
                    self.midSetState(.ignore_r, time_ms);
                    return false;
                },
                .r_up => self.midSetState(.idle, time_ms),
                // r_down, l_up, timeout, all_up are state errors.
                .r_down, .l_up, .timeout, .all_up => {},
            },
            .passthrough => switch (ev) {
                .l_down, .r_down, .other, .r_up, .l_up => return false,
                .all_up => self.midSetState(.idle, time_ms),
                .timeout => {}, // state error
            },
            .ignore_lr => switch (ev) {
                .other => return false,
                .r_up => self.midSetState(.ignore_l, time_ms),
                .l_up => self.midSetState(.ignore_r, time_ms),
                // l_down, r_down, timeout, all_up are state errors.
                .l_down, .r_down, .timeout, .all_up => {},
            },
            .ignore_l => switch (ev) {
                .r_down, .other, .r_up => return false,
                .l_up => self.midSetState(.passthrough, time_ms),
                // l_down, timeout, all_up are state errors.
                .l_down, .timeout, .all_up => {},
            },
            .ignore_r => switch (ev) {
                .l_down, .other, .l_up => return false,
                .r_up => self.midSetState(.passthrough, time_ms),
                // r_down is a state error. timeout, all_up are no-ops.
                .r_down, .timeout, .all_up => {},
            },
        }
        return true;
    }

    // Port of evdev_middlebutton_filter_button. Runs pending physical buttons through the state
    // machine, writes the resolved events into buf, and returns the count.
    fn resolveMiddleEmu(
        self: *Processor,
        pending: []const PendingButton,
        time_ms: u64,
        buf: []PendingButton,
    ) usize {
        var o = MidOut{ .buf = buf };

        // Check for the frame-driven timeout. Only LEFT_DOWN and RIGHT_DOWN arm the timer.
        if ((self.mid_state == .left_down or self.mid_state == .right_down) and
            time_ms >= self.mid_first_event_time and
            (time_ms - self.mid_first_event_time) >= MID_EMU_TIMEOUT_MS)
        {
            _ = self.midHandleEvent(.timeout, time_ms, &o);
        }

        for (pending) |pb| {
            const ev: MidEvent = switch (pb.button) {
                evdev.BTN_LEFT => if (pb.state == .pressed) .l_down else .l_up,
                evdev.BTN_RIGHT => if (pb.state == .pressed) .r_down else .r_up,
                else => .other,
            };

            const suppress = self.midHandleEvent(ev, time_ms, &o);

            // Fire ALL_UP on the last-button-release transition, as in evdev_middlebutton_filter_button.
            const btnbit: u5 = @intCast(pb.button - evdev.BTN_LEFT);
            const old_mask = self.mid_button_mask;
            if (pb.state == .pressed) {
                self.mid_button_mask |= (@as(u32, 1) << btnbit);
            } else {
                self.mid_button_mask &= ~(@as(u32, 1) << btnbit);
            }
            if (old_mask != self.mid_button_mask and self.mid_button_mask == 0) {
                _ = self.midHandleEvent(.all_up, time_ms, &o);
            }

            if (!suppress) o.post(pb.button, pb.state);
        }

        return o.len;
    }

    pub fn flush(
        self: *Processor,
        time_ms: u64,
        out: *std.ArrayList(event.Event),
        gpa: std.mem.Allocator,
    ) !void {
        defer self.clearFrame();

        // Keyboard keys: raw code + state, stamped with the frame timestamp.
        {
            var ki: usize = 0;
            while (ki < self.pending_keys_len) : (ki += 1) {
                var k = self.pending_keys[ki];
                k.time_ms = time_ms;
                try out.append(gpa, .{ .key = k });
            }
        }

        // Middle-button emulation: the raw pending list is replaced by the machine's output.
        var resolved: [MAX_RESOLVED]PendingButton = undefined;
        const pending: []const PendingButton = if (self.cfg.middle_emulation)
            resolved[0..self.resolveMiddleEmu(self.pending_items[0..self.pending_len], time_ms, &resolved)]
        else
            self.pending_items[0..self.pending_len];

        var suppress_motion = false;
        var suppress_scroll_btn = false;

        if (self.cfg.scroll_method == .on_button_down) {
            for (pending) |pb| {
                if (pb.button == self.cfg.scroll_button) {
                    suppress_scroll_btn = true;
                    if (pb.state == .pressed) {
                        if (self.scroll_state == .idle) {
                            self.scroll_state = .button_down_pending;
                        }
                    } else {
                        self.scroll_state = .idle;
                        self.scroll_accum_x = 0.0;
                        self.scroll_accum_y = 0.0;
                    }
                }
            }

            if (self.scroll_state == .button_down_pending or self.scroll_state == .scrolling) {
                self.scroll_accum_x += @floatFromInt(self.rel_x);
                self.scroll_accum_y += @floatFromInt(self.rel_y);
                const dist = std.math.hypot(self.scroll_accum_x, self.scroll_accum_y);
                if (dist >= SCROLL_START_THRESHOLD) {
                    self.scroll_state = .scrolling;
                }
                // Motion always suppressed here. Axis events only emitted once the scrolling state is reached.
                suppress_motion = true;
                if (self.scroll_state == .scrolling and (self.rel_x != 0 or self.rel_y != 0)) {
                    const sign: f64 = if (self.cfg.natural_scroll) -1.0 else 1.0;
                    if (self.rel_y != 0) {
                        const val = @as(f64, @floatFromInt(self.rel_y)) * sign;
                        try out.append(gpa, .{ .pointer_axis = .{
                            .time_ms = time_ms,
                            .axis = .vertical,
                            .value = val,
                            .source = .continuous,
                            .v120 = 0,
                        } });
                    }
                    if (self.rel_x != 0) {
                        const val = @as(f64, @floatFromInt(self.rel_x)) * sign;
                        try out.append(gpa, .{ .pointer_axis = .{
                            .time_ms = time_ms,
                            .axis = .horizontal,
                            .value = val,
                            .source = .continuous,
                            .v120 = 0,
                        } });
                    }
                }
            }
        }

        if (!suppress_motion and (self.rel_x != 0 or self.rel_y != 0)) {
            const dx_raw: f64 = @floatFromInt(self.rel_x);
            const dy_raw: f64 = @floatFromInt(self.rel_y);
            const accel = self.fil.filter(dx_raw, dy_raw, time_ms);
            try out.append(gpa, .{ .pointer_motion = .{
                .time_ms = time_ms,
                .dx = accel.dx,
                .dy = accel.dy,
                .dx_unaccel = dx_raw,
                .dy_unaccel = dy_raw,
            } });
        }

        // Buttons: left-handed swap applied, scroll button suppressed.
        for (pending) |pb| {
            const suppressed = self.cfg.scroll_method == .on_button_down and
                pb.button == self.cfg.scroll_button and suppress_scroll_btn;
            if (!suppressed) {
                var btn = pb.button;
                if (self.cfg.left_handed) {
                    if (btn == evdev.BTN_LEFT) btn = evdev.BTN_RIGHT else if (btn == evdev.BTN_RIGHT) btn = evdev.BTN_LEFT;
                }
                // Count uses the emitted code (after swap) so the consumer gets a consistent count per code.
                var sbc: u32 = 0;
                if (btn >= evdev.BTN_LEFT) {
                    const btn_idx: usize = @intCast(btn - evdev.BTN_LEFT);
                    if (btn_idx < 8) {
                        if (pb.state == .pressed) {
                            self.seat_btn_count[btn_idx] += 1;
                        } else if (self.seat_btn_count[btn_idx] > 0) {
                            self.seat_btn_count[btn_idx] -= 1;
                        }
                        sbc = self.seat_btn_count[btn_idx];
                    }
                }
                try out.append(gpa, .{ .pointer_button = .{
                    .time_ms = time_ms,
                    .button = btn,
                    .state = pb.state,
                    .seat_button_count = sbc,
                } });
            }
        }

        // Wheel: hi-res wins, low-res suppressed when both arrive in the same frame.
        const sign: f64 = if (self.cfg.natural_scroll) -1.0 else 1.0;

        if (self.wheel_hi != 0 or self.wheel != 0) {
            const v120_raw: f64 = if (self.wheel_hi != 0)
                @floatFromInt(self.wheel_hi)
            else
                @as(f64, @floatFromInt(self.wheel)) * 120.0;
            const v120 = v120_raw * sign;
            try out.append(gpa, .{ .pointer_axis = .{
                .time_ms = time_ms,
                .axis = .vertical,
                .value = v120 / 120.0,
                .source = .wheel,
                .v120 = v120,
            } });
        }

        if (self.hwheel_hi != 0 or self.hwheel != 0) {
            const v120_raw: f64 = if (self.hwheel_hi != 0)
                @floatFromInt(self.hwheel_hi)
            else
                @as(f64, @floatFromInt(self.hwheel)) * 120.0;
            const v120 = v120_raw * sign;
            try out.append(gpa, .{ .pointer_axis = .{
                .time_ms = time_ms,
                .axis = .horizontal,
                .value = v120 / 120.0,
                .source = .wheel,
                .v120 = v120,
            } });
        }
    }

    pub fn reset(self: *Processor) void {
        self.clearFrame();
        self.mid_state = .idle;
        self.mid_button_mask = 0;
        self.mid_first_event_time = 0;
        self.scroll_state = .idle;
        self.scroll_accum_x = 0.0;
        self.scroll_accum_y = 0.0;
        self.seat_btn_count = [_]u32{0} ** 8;
        self.fil.reset();
    }

    fn clearFrame(self: *Processor) void {
        self.rel_x = 0;
        self.rel_y = 0;
        self.wheel = 0;
        self.hwheel = 0;
        self.wheel_hi = 0;
        self.hwheel_hi = 0;
        self.pending_len = 0;
        self.pending_keys_len = 0;
    }
};

fn iev(t: u16, c: u16, v: i32) evdev.InputEvent {
    return .{ .time = .{ .sec = 0, .usec = 0 }, .type = t, .code = c, .value = v };
}

test "a relative frame produces one motion event with unaccel deltas" {
    var f = filter.Filter.init(.mouse, 1000);
    f.setProfile(.flat);
    f.setSpeed(0.0);
    var cfg = Config{};
    var p = Processor.init(&f, &cfg);
    var out = std.ArrayList(event.Event).empty;
    defer out.deinit(std.testing.allocator);

    p.feed(iev(evdev.EV_REL, evdev.REL_X, 3));
    p.feed(iev(evdev.EV_REL, evdev.REL_Y, 4));
    try p.flush(1, &out, std.testing.allocator);

    try std.testing.expectEqual(@as(usize, 1), out.items.len);
    const m = out.items[0].pointer_motion;
    try std.testing.expectEqual(@as(f64, 3), m.dx_unaccel);
    try std.testing.expectEqual(@as(f64, 4), m.dy_unaccel);
}

test "left then right within timeout emits a synthesized middle button" {
    var f = filter.Filter.init(.mouse, 1000);
    var cfg = Config{ .middle_emulation = true };
    var p = Processor.init(&f, &cfg);
    var out = std.ArrayList(event.Event).empty;
    defer out.deinit(std.testing.allocator);

    p.feed(iev(evdev.EV_KEY, evdev.BTN_LEFT, 1));
    p.feed(iev(evdev.EV_KEY, evdev.BTN_RIGHT, 1));
    try p.flush(5, &out, std.testing.allocator);

    var saw_middle = false;
    for (out.items) |e| switch (e) {
        .pointer_button => |b| {
            if (b.button == evdev.BTN_MIDDLE and b.state == .pressed) saw_middle = true;
            // No stray physical left/right press or release must leak out while
            // middle is being emulated.
            try std.testing.expect(b.button != evdev.BTN_LEFT);
            try std.testing.expect(b.button != evdev.BTN_RIGHT);
        },
        else => {},
    };
    try std.testing.expect(saw_middle);
    try std.testing.expectEqual(MidState.middle, p.mid_state);
}

// Count button events of a given code and state in out.
fn countButton(out: *const std.ArrayList(event.Event), button: u32, st: event.ButtonState) usize {
    var n: usize = 0;
    for (out.items) |e| switch (e) {
        .pointer_button => |b| if (b.button == button and b.state == st) {
            n += 1;
        },
        else => {},
    };
    return n;
}

test "middle emulation: left released then right released emits one middle release, no strays" {
    var f = filter.Filter.init(.mouse, 1000);
    var cfg = Config{ .middle_emulation = true };
    var p = Processor.init(&f, &cfg);
    var out = std.ArrayList(event.Event).empty;
    defer out.deinit(std.testing.allocator);

    // Frame 1: left+right within timeout -> middle press.
    p.feed(iev(evdev.EV_KEY, evdev.BTN_LEFT, 1));
    p.feed(iev(evdev.EV_KEY, evdev.BTN_RIGHT, 1));
    try p.flush(5, &out, std.testing.allocator);
    p.feed(iev(evdev.EV_KEY, evdev.BTN_LEFT, 0));
    try p.flush(10, &out, std.testing.allocator);
    // Right released in a separate frame, as in real use.
    p.feed(iev(evdev.EV_KEY, evdev.BTN_RIGHT, 0));
    try p.flush(15, &out, std.testing.allocator);

    try std.testing.expectEqual(@as(usize, 1), countButton(&out, evdev.BTN_MIDDLE, .pressed));
    try std.testing.expectEqual(@as(usize, 1), countButton(&out, evdev.BTN_MIDDLE, .released));
    // No physical left/right leaked out at all.
    try std.testing.expectEqual(@as(usize, 0), countButton(&out, evdev.BTN_LEFT, .pressed));
    try std.testing.expectEqual(@as(usize, 0), countButton(&out, evdev.BTN_LEFT, .released));
    try std.testing.expectEqual(@as(usize, 0), countButton(&out, evdev.BTN_RIGHT, .pressed));
    try std.testing.expectEqual(@as(usize, 0), countButton(&out, evdev.BTN_RIGHT, .released));
    try std.testing.expectEqual(MidState.idle, p.mid_state);
    try std.testing.expectEqual(@as(u32, 0), p.mid_button_mask);
}

test "middle emulation: right released then left released emits one middle release, no strays" {
    var f = filter.Filter.init(.mouse, 1000);
    var cfg = Config{ .middle_emulation = true };
    var p = Processor.init(&f, &cfg);
    var out = std.ArrayList(event.Event).empty;
    defer out.deinit(std.testing.allocator);

    p.feed(iev(evdev.EV_KEY, evdev.BTN_RIGHT, 1));
    p.feed(iev(evdev.EV_KEY, evdev.BTN_LEFT, 1));
    try p.flush(5, &out, std.testing.allocator);
    p.feed(iev(evdev.EV_KEY, evdev.BTN_RIGHT, 0));
    try p.flush(10, &out, std.testing.allocator);
    p.feed(iev(evdev.EV_KEY, evdev.BTN_LEFT, 0));
    try p.flush(15, &out, std.testing.allocator);

    try std.testing.expectEqual(@as(usize, 1), countButton(&out, evdev.BTN_MIDDLE, .pressed));
    try std.testing.expectEqual(@as(usize, 1), countButton(&out, evdev.BTN_MIDDLE, .released));
    try std.testing.expectEqual(@as(usize, 0), countButton(&out, evdev.BTN_LEFT, .pressed));
    try std.testing.expectEqual(@as(usize, 0), countButton(&out, evdev.BTN_LEFT, .released));
    try std.testing.expectEqual(@as(usize, 0), countButton(&out, evdev.BTN_RIGHT, .pressed));
    try std.testing.expectEqual(@as(usize, 0), countButton(&out, evdev.BTN_RIGHT, .released));
    try std.testing.expectEqual(MidState.idle, p.mid_state);
    try std.testing.expectEqual(@as(u32, 0), p.mid_button_mask);
}

test "middle emulation: right pressed after timeout does not emulate, both buttons pass through" {
    var f = filter.Filter.init(.mouse, 1000);
    var cfg = Config{ .middle_emulation = true };
    var p = Processor.init(&f, &cfg);
    var out = std.ArrayList(event.Event).empty;
    defer out.deinit(std.testing.allocator);

    // Frame 1 at t=0: left pressed, held (suppressed for now).
    p.feed(iev(evdev.EV_KEY, evdev.BTN_LEFT, 1));
    try p.flush(0, &out, std.testing.allocator);
    // Frame 2 at t=1000: right pressed, but the 50ms timeout already elapsed,
    // so the held left is flushed and no middle is emulated.
    p.feed(iev(evdev.EV_KEY, evdev.BTN_RIGHT, 1));
    try p.flush(1000, &out, std.testing.allocator);

    try std.testing.expectEqual(@as(usize, 0), countButton(&out, evdev.BTN_MIDDLE, .pressed));
    try std.testing.expectEqual(@as(usize, 0), countButton(&out, evdev.BTN_MIDDLE, .released));
    try std.testing.expectEqual(@as(usize, 1), countButton(&out, evdev.BTN_LEFT, .pressed));
    try std.testing.expectEqual(@as(usize, 1), countButton(&out, evdev.BTN_RIGHT, .pressed));
    // State is passthrough while both physical buttons remain held.
    try std.testing.expectEqual(MidState.passthrough, p.mid_state);
}

test "hi-res wheel becomes a vertical axis event with v120" {
    var f = filter.Filter.init(.mouse, 1000);
    var cfg = Config{};
    var p = Processor.init(&f, &cfg);
    var out = std.ArrayList(event.Event).empty;
    defer out.deinit(std.testing.allocator);

    p.feed(iev(evdev.EV_REL, evdev.REL_WHEEL_HI_RES, 120));
    p.feed(iev(evdev.EV_REL, evdev.REL_WHEEL, 1)); // low-res duplicate, must be ignored when hi-res present
    try p.flush(2, &out, std.testing.allocator);

    var axis_events: usize = 0;
    for (out.items) |e| switch (e) {
        .pointer_axis => |a| {
            axis_events += 1;
            try std.testing.expectEqual(event.Axis.vertical, a.axis);
            try std.testing.expectEqual(@as(f64, 120), a.v120);
        },
        else => {},
    };
    try std.testing.expectEqual(@as(usize, 1), axis_events);
}

test "natural scroll flips axis sign" {
    var f = filter.Filter.init(.mouse, 1000);
    var cfg = Config{ .natural_scroll = true };
    var p = Processor.init(&f, &cfg);
    var out = std.ArrayList(event.Event).empty;
    defer out.deinit(std.testing.allocator);
    p.feed(iev(evdev.EV_REL, evdev.REL_WHEEL_HI_RES, 120));
    try p.flush(1, &out, std.testing.allocator);
    try std.testing.expect(out.items[0].pointer_axis.v120 == -120);
}

test "left-handed swaps left and right buttons" {
    var f = filter.Filter.init(.mouse, 1000);
    var cfg = Config{ .left_handed = true };
    var p = Processor.init(&f, &cfg);
    var out = std.ArrayList(event.Event).empty;
    defer out.deinit(std.testing.allocator);
    p.feed(iev(evdev.EV_KEY, evdev.BTN_LEFT, 1));
    try p.flush(1, &out, std.testing.allocator);
    try std.testing.expectEqual(@as(u32, evdev.BTN_RIGHT), out.items[0].pointer_button.button);
}

test "seat_button_count is 1 on press and 0 on release (separate frames)" {
    var f = filter.Filter.init(.mouse, 1000);
    var cfg = Config{};
    var p = Processor.init(&f, &cfg);
    var out = std.ArrayList(event.Event).empty;
    defer out.deinit(std.testing.allocator);

    p.feed(iev(evdev.EV_KEY, evdev.BTN_LEFT, 1));
    try p.flush(1, &out, std.testing.allocator);
    const press_ev = out.items[out.items.len - 1].pointer_button;
    try std.testing.expectEqual(event.ButtonState.pressed, press_ev.state);
    try std.testing.expectEqual(@as(u32, 1), press_ev.seat_button_count);

    // Release in a separate frame. Count store persists across flushes.
    const before = out.items.len;
    p.feed(iev(evdev.EV_KEY, evdev.BTN_LEFT, 0));
    try p.flush(2, &out, std.testing.allocator);
    const release_ev = out.items[before].pointer_button;
    try std.testing.expectEqual(event.ButtonState.released, release_ev.state);
    try std.testing.expectEqual(@as(u32, 0), release_ev.seat_button_count);
}

test "processor emits keyboard key events (below BTN_MISC)" {
    var fil = filter.Filter.init(.mouse, 1000);
    var cfg = Config{};
    _ = &cfg;
    var p = Processor.init(&fil, &cfg);
    const KEY_A: u16 = 30;
    p.feed(iev(evdev.EV_KEY, KEY_A, 1)); // press

    var out = std.ArrayList(event.Event).empty;
    defer out.deinit(std.testing.allocator);
    try p.flush(1000, &out, std.testing.allocator);

    try std.testing.expectEqual(@as(usize, 1), out.items.len);
    try std.testing.expect(out.items[0] == .key);
    try std.testing.expectEqual(@as(u32, KEY_A), out.items[0].key.code);
    try std.testing.expectEqual(event.ButtonState.pressed, out.items[0].key.state);
}

test "button-scroll continuous axis has v120 == 0, wheel axis keeps v120 == 120" {
    // Part A: button-scroll continuous source must have v120 == 0.
    {
        var f = filter.Filter.init(.mouse, 1000);
        f.setProfile(.flat);
        f.setSpeed(0.0);
        var cfg = Config{
            .scroll_method = .on_button_down,
            .scroll_button = evdev.BTN_MIDDLE,
        };
        var p = Processor.init(&f, &cfg);
        var out = std.ArrayList(event.Event).empty;
        defer out.deinit(std.testing.allocator);

        // Arm scroll button.
        p.feed(iev(evdev.EV_KEY, evdev.BTN_MIDDLE, 1));
        try p.flush(1, &out, std.testing.allocator);

        // Move past threshold (>= 5.0 units) to transition into scrolling.
        p.feed(iev(evdev.EV_REL, evdev.REL_Y, 10));
        const before = out.items.len;
        try p.flush(2, &out, std.testing.allocator);

        var found_axis = false;
        for (out.items[before..]) |e| switch (e) {
            .pointer_axis => |a| {
                try std.testing.expectEqual(event.AxisSource.continuous, a.source);
                try std.testing.expectEqual(@as(f64, 0), a.v120);
                found_axis = true;
            },
            else => {},
        };
        try std.testing.expect(found_axis);
    }

    // Part B: wheel source keeps v120 set correctly.
    {
        var f = filter.Filter.init(.mouse, 1000);
        var cfg = Config{};
        var p = Processor.init(&f, &cfg);
        var out = std.ArrayList(event.Event).empty;
        defer out.deinit(std.testing.allocator);

        p.feed(iev(evdev.EV_REL, evdev.REL_WHEEL_HI_RES, 120));
        try p.flush(3, &out, std.testing.allocator);

        var found_wheel = false;
        for (out.items) |e| switch (e) {
            .pointer_axis => |a| {
                try std.testing.expectEqual(event.AxisSource.wheel, a.source);
                try std.testing.expectEqual(@as(f64, 120), a.v120);
                found_wheel = true;
            },
            else => {},
        };
        try std.testing.expect(found_wheel);
    }
}
