//! Device: ties an EventSource to a Filter, Processor, and Caps.
//! Dispatches raw evdev events into frames and emits Event values.
//! Supports fd-backed devices (capability probe via ioctls) and
//! memory-backed devices for testing (caps supplied directly).

const std = @import("std");
const linux = std.os.linux;
const posix = std.posix;
const evdev = @import("evdev.zig");
const source = @import("source.zig");
const event = @import("event.zig");
const filter = @import("filter.zig");
const pointer = @import("pointer.zig");
const tablet = @import("tablet.zig");

pub const Caps = struct {
    has_rel: bool,
    has_buttons: bool,
    has_wheel: bool,
    has_keys: bool,
    is_tablet: bool,
    /// Direct-input touch surface (a touchscreen), not a touchpad and not a pen
    /// tablet. Defaults to false so existing Caps literals stay valid.
    is_touchscreen: bool = false,
    id: evdev.InputId,
    kind: filter.Kind,
    name: [64]u8,
    name_len: usize,
    abs_x: evdev.AbsRange = .{},
    abs_y: evdev.AbsRange = .{},
    abs_pressure: evdev.AbsRange = .{},
};

/// The evdev capability bitmasks a classification needs, as returned by
/// EVIOCGBIT(EV_ABS), EVIOCGBIT(EV_KEY), and EVIOCGPROP. Grouping them keeps the
/// three same-typed slices from being passed in the wrong order.
pub const DeviceBits = struct {
    abs: []const u8,
    key: []const u8,
    prop: []const u8,
};

/// Report whether these bitmasks describe a pen tablet: absolute position plus
/// a pen tool. The pen tool is what keeps a graphics tablet apart from every
/// other absolute device.
pub fn isTablet(bits: DeviceBits) bool {
    return evdev.testBit(bits.abs, evdev.ABS_X) and
        evdev.testBit(bits.abs, evdev.ABS_Y) and
        evdev.testBit(bits.key, evdev.BTN_TOOL_PEN);
}

/// Report whether these bitmasks describe a touchscreen.
///
/// A laptop touchpad reports the same ABS_MT axes as a touchscreen, so the
/// multitouch axes alone cannot separate the two. INPUT_PROP_DIRECT is what
/// separates them: it says the contact lands under the finger instead of driving
/// a cursor elsewhere. A touchpad sets INPUT_PROP_POINTER, which is rejected
/// here even in the contradictory case where a device claims both.
///
/// BTN_TOOL_PEN rejects pen tablets, including display tablets that set
/// INPUT_PROP_DIRECT and BTN_TOUCH as well.
pub fn isTouchscreen(bits: DeviceBits) bool {
    if (!evdev.testBit(bits.prop, evdev.INPUT_PROP_DIRECT)) return false;
    if (evdev.testBit(bits.prop, evdev.INPUT_PROP_POINTER)) return false;
    if (evdev.testBit(bits.key, evdev.BTN_TOOL_PEN)) return false;

    const has_mt_position = evdev.testBit(bits.abs, evdev.ABS_MT_POSITION_X) and
        evdev.testBit(bits.abs, evdev.ABS_MT_POSITION_Y);
    // Touchscreens older than the MT protocol report one contact through
    // BTN_TOUCH and the plain absolute axes, with no ABS_MT axes at all.
    const has_single_touch = evdev.testBit(bits.key, evdev.BTN_TOUCH) and
        evdev.testBit(bits.abs, evdev.ABS_X) and
        evdev.testBit(bits.abs, evdev.ABS_Y);

    return has_mt_position or has_single_touch;
}

pub const Error = error{
    OpenFailed,
    NotACharDevice,
    IoctlFailed,
    DeviceGone,
    Unexpected,
} || source.Error;

pub const Device = struct {
    src: source.EventSource,
    caps: Caps,
    device_id: u32,
    fil: filter.Filter,
    cfg: pointer.Config,
    proc: pointer.Processor,
    tproc: ?tablet.TabletProcessor = null,
    owns_fd: bool = true,

    /// Probe a ready non-blocking evdev fd for its type + capabilities.
    /// Does NOT close the fd on error; the caller owns it.
    fn probe(the_fd: posix.fd_t) Error!Caps {
        // Use statx(fd, "", AT_EMPTY_PATH) to check the file type (no fstat in Zig 0.16).
        var sx: linux.Statx = undefined;
        const empty_path: [*:0]const u8 = "";
        switch (posix.errno(linux.statx(
            the_fd,
            empty_path,
            @as(u32, linux.AT.EMPTY_PATH),
            linux.STATX{ .TYPE = true },
            &sx,
        ))) {
            .SUCCESS => {},
            else => return error.OpenFailed,
        }
        // mode bits: S_IFMT = 0xF000, S_IFCHR = 0x2000
        if ((sx.mode & 0xF000) != 0x2000) return error.NotACharDevice;

        var caps = Caps{
            .has_rel = false,
            .has_buttons = false,
            .has_wheel = false,
            .has_keys = false,
            .is_tablet = false,
            .id = undefined,
            .kind = .mouse,
            .name = undefined,
            .name_len = 0,
        };

        // EVIOCGID: device identity (bustype, vendor, product, version).
        switch (posix.errno(linux.ioctl(the_fd, evdev.EVIOCGID(), @intFromPtr(&caps.id)))) {
            .SUCCESS => {},
            else => return error.IoctlFailed,
        }

        // EVIOCGNAME: null-terminated device name string.
        switch (posix.errno(linux.ioctl(the_fd, evdev.EVIOCGNAME(caps.name.len), @intFromPtr(&caps.name)))) {
            .SUCCESS => {},
            else => return error.IoctlFailed,
        }
        caps.name_len = std.mem.indexOfScalar(u8, &caps.name, 0) orelse caps.name.len;

        // EVIOCGBIT(EV_REL): relative axis capability bitmask.
        // REL_MAX = 0x0f, nbits(REL_MAX+1) = nbits(16) = 2 bytes.
        var rel_bits: [(evdev.REL_MAX + 1 + 7) / 8]u8 = @splat(0);
        switch (posix.errno(linux.ioctl(the_fd, evdev.EVIOCGBIT(evdev.EV_REL, rel_bits.len), @intFromPtr(&rel_bits)))) {
            .SUCCESS => {},
            else => return error.IoctlFailed,
        }
        caps.has_rel = evdev.testBit(&rel_bits, evdev.REL_X) or evdev.testBit(&rel_bits, evdev.REL_Y);
        caps.has_wheel = evdev.testBit(&rel_bits, evdev.REL_WHEEL) or
            evdev.testBit(&rel_bits, evdev.REL_WHEEL_HI_RES);

        // EVIOCGBIT(EV_KEY): key/button capability bitmask.
        // KEY_MAX = 0x2ff = 767, nbits(768) = 96 bytes.
        var key_bits: [(evdev.KEY_MAX + 1 + 7) / 8]u8 = @splat(0);
        switch (posix.errno(linux.ioctl(the_fd, evdev.EVIOCGBIT(evdev.EV_KEY, key_bits.len), @intFromPtr(&key_bits)))) {
            .SUCCESS => {},
            else => return error.IoctlFailed,
        }
        caps.has_buttons = evdev.testBit(&key_bits, evdev.BTN_LEFT) or
            evdev.testBit(&key_bits, evdev.BTN_RIGHT) or
            evdev.testBit(&key_bits, evdev.BTN_MIDDLE);

        // Keyboard keys present: any EV_KEY bit below BTN_MISC (0x100) set.
        caps.has_keys = blk: {
            var code: u16 = 0;
            while (code < @as(u16, evdev.BTN_MISC)) : (code += 1) {
                if (evdev.testBit(&key_bits, code)) break :blk true;
            }
            break :blk false;
        };

        // EVIOCGBIT(EV_ABS): absolute axis capability bitmask.
        // ABS_MAX = 0x3f = 63, nbits(64) = 8 bytes.
        var abs_bits: [(evdev.ABS_MAX + 1 + 7) / 8]u8 = @splat(0);
        switch (posix.errno(linux.ioctl(the_fd, evdev.EVIOCGBIT(evdev.EV_ABS, abs_bits.len), @intFromPtr(&abs_bits)))) {
            .SUCCESS => {},
            else => return error.IoctlFailed,
        }

        // EVIOCGPROP: device property bitmask, the only thing that tells a
        // touchscreen apart from a touchpad.
        // INPUT_PROP_MAX = 0x1f = 31, nbits(32) = 4 bytes.
        var prop_bits: [(evdev.INPUT_PROP_MAX + 1 + 7) / 8]u8 = @splat(0);
        switch (posix.errno(linux.ioctl(the_fd, evdev.EVIOCGPROP(prop_bits.len), @intFromPtr(&prop_bits)))) {
            .SUCCESS => {},
            else => return error.IoctlFailed,
        }

        const bits = DeviceBits{ .abs = &abs_bits, .key = &key_bits, .prop = &prop_bits };
        caps.is_tablet = isTablet(bits);
        caps.is_touchscreen = isTouchscreen(bits);

        // For tablet devices, read the absolute axis ranges for normalization.
        if (caps.is_tablet) {
            var ai: evdev.AbsInfo = undefined;
            switch (posix.errno(linux.ioctl(the_fd, evdev.EVIOCGABS(evdev.ABS_X), @intFromPtr(&ai)))) {
                .SUCCESS => caps.abs_x = .{ .min = ai.minimum, .max = ai.maximum },
                else => {},
            }
            switch (posix.errno(linux.ioctl(the_fd, evdev.EVIOCGABS(evdev.ABS_Y), @intFromPtr(&ai)))) {
                .SUCCESS => caps.abs_y = .{ .min = ai.minimum, .max = ai.maximum },
                else => {},
            }
            switch (posix.errno(linux.ioctl(the_fd, evdev.EVIOCGABS(evdev.ABS_PRESSURE), @intFromPtr(&ai)))) {
                .SUCCESS => caps.abs_pressure = .{ .min = ai.minimum, .max = ai.maximum },
                else => {},
            }
        }

        // trackpoint: name match, or has BTN_MIDDLE with relative axes but no wheel (pointing-stick heuristic).
        const name_slice = caps.name[0..caps.name_len];
        const is_trackpoint = std.mem.indexOf(u8, name_slice, "TrackPoint") != null or
            std.mem.indexOf(u8, name_slice, "Stick") != null or
            (caps.has_rel and !caps.has_wheel and evdev.testBit(&key_bits, evdev.BTN_MIDDLE));
        caps.kind = if (is_trackpoint) .trackpoint else .mouse;

        return caps;
    }

    /// Open a real evdev device node. Probes capabilities via ioctls.
    pub fn open(gpa: std.mem.Allocator, path: []const u8, id: u32) Error!*Device {
        const path_z = gpa.dupeSentinel(u8, path, 0) catch return error.OpenFailed;
        defer gpa.free(path_z);

        const open_rc = linux.openat(
            linux.AT.FDCWD,
            path_z,
            linux.O{ .ACCMODE = .RDONLY, .NONBLOCK = true, .CLOEXEC = true },
            0,
        );
        switch (posix.errno(open_rc)) {
            .SUCCESS => {},
            else => return error.OpenFailed,
        }
        const the_fd: posix.fd_t = @intCast(open_rc);
        errdefer _ = linux.close(the_fd);
        const caps = try probe(the_fd);
        const dev = try makeDevice(gpa, .{ .fd = the_fd }, caps, id);
        dev.owns_fd = true;
        return dev;
    }

    /// Open a libinput device from an already-open, non-blocking evdev fd
    /// (e.g. handed over by a seat manager). The caller retains fd ownership:
    /// owns_fd = false, so Device.deinit will NOT close it.
    pub fn openFd(gpa: std.mem.Allocator, the_fd: posix.fd_t, id: u32) Error!*Device {
        const caps = try probe(the_fd);
        const dev = try makeDevice(gpa, .{ .fd = the_fd }, caps, id);
        dev.owns_fd = false;
        return dev;
    }

    /// Memory-backed constructor for unit tests. No ioctls, no file operations.
    pub fn initMemory(gpa: std.mem.Allocator, events: []const evdev.InputEvent, caps: Caps, id: u32) Error!*Device {
        return makeDevice(gpa, .{ .memory = .{ .events = events, .pos = 0 } }, caps, id);
    }

    fn makeDevice(gpa: std.mem.Allocator, src: source.EventSource, caps: Caps, id: u32) Error!*Device {
        const dev = gpa.create(Device) catch return error.Unexpected;
        dev.src = src;
        dev.caps = caps;
        dev.device_id = id;
        dev.fil = filter.Filter.init(caps.kind, 1000);
        dev.cfg = pointer.Config{};
        // proc points into this Device's own fields. dev is heap-allocated so they stay valid.
        dev.proc = pointer.Processor.init(&dev.fil, &dev.cfg);
        dev.tproc = if (caps.is_tablet)
            tablet.TabletProcessor.init(caps.abs_x, caps.abs_y, caps.abs_pressure)
        else
            null;
        return dev;
    }

    /// Return the pollable fd, or null for memory-backed devices.
    pub fn fd(self: *Device) ?posix.fd_t {
        return switch (self.src) {
            .fd => |f| f,
            .memory => null,
        };
    }

    pub fn config(self: *Device) *pointer.Config {
        return &self.cfg;
    }

    pub fn filterPtr(self: *Device) *filter.Filter {
        return &self.fil;
    }

    /// Reads events from the source. Flushes on SYN_REPORT, resets on SYN_DROPPED.
    /// Returns cleanly on WouldBlock or when a memory source is exhausted.
    /// Returns DeviceGone when the fd closes (ENODEV, device removed).
    pub fn dispatch(self: *Device, out: *std.ArrayList(event.Event), gpa: std.mem.Allocator) Error!void {
        var scratch: [64]evdev.InputEvent = undefined;
        while (true) {
            const n = self.src.read(&scratch) catch |err| switch (err) {
                error.WouldBlock => return,
                error.Closed => switch (self.src) {
                    // memory source exhausted, end of test data.
                    .memory => return,
                    // real fd closed (ENODEV/EOF), device was removed.
                    .fd => return error.DeviceGone,
                },
                error.Io => return error.Io,
                error.Unexpected => return error.Unexpected,
            };
            for (scratch[0..n]) |e| {
                if (e.type == evdev.EV_SYN) {
                    if (e.code == evdev.SYN_REPORT) {
                        // @max(0,...) guards negative time fields before the unsigned cast. ms = sec*1000 + usec/1000.
                        const sec: u64 = @intCast(@max(0, e.time.sec));
                        const usec: u64 = @intCast(@max(0, e.time.usec));
                        const time_ms: u64 = sec * 1000 + usec / 1000;
                        if (self.tproc) |*tp| {
                            tp.flush(time_ms, out, gpa) catch return error.Unexpected;
                        } else {
                            self.proc.flush(time_ms, out, gpa) catch return error.Unexpected;
                        }
                    } else if (e.code == evdev.SYN_DROPPED) {
                        if (self.tproc == null) self.proc.reset();
                    }
                } else {
                    if (self.tproc) |*tp| {
                        tp.feed(e);
                    } else {
                        self.proc.feed(e);
                    }
                }
            }
        }
    }

    /// Release the fd (if any) and free the Device allocation.
    pub fn deinit(self: *Device, gpa: std.mem.Allocator) void {
        if (self.owns_fd) {
            if (self.fd()) |f| _ = linux.close(f);
        }
        gpa.destroy(self);
    }
};

fn iev(t: u16, c: u16, v: i32) evdev.InputEvent {
    return .{ .time = .{ .sec = 0, .usec = 1000 }, .type = t, .code = c, .value = v };
}

test "device frames events on SYN_REPORT and emits motion" {
    const data = [_]evdev.InputEvent{
        iev(evdev.EV_REL, evdev.REL_X, 2),
        iev(evdev.EV_REL, evdev.REL_Y, 0),
        iev(evdev.EV_SYN, evdev.SYN_REPORT, 0),
    };
    const caps = Caps{
        .has_rel = true,
        .has_buttons = true,
        .has_wheel = true,
        .has_keys = false,
        .is_tablet = false,
        .id = .{ .bustype = 3, .vendor = 1, .product = 1, .version = 1 },
        .kind = .mouse,
        .name = undefined,
        .name_len = 0,
    };
    const dev = try Device.initMemory(std.testing.allocator, &data, caps, 1);
    defer dev.deinit(std.testing.allocator);
    dev.filterPtr().setProfile(.flat);
    dev.filterPtr().setSpeed(0.0);

    var out = std.ArrayList(event.Event).empty;
    defer out.deinit(std.testing.allocator);
    try dev.dispatch(&out, std.testing.allocator);

    try std.testing.expectEqual(@as(usize, 1), out.items.len);
    try std.testing.expectEqual(@as(f64, 2), out.items[0].pointer_motion.dx_unaccel);
}

test "tablet device emits tablet_axis and tablet_tip on a pen frame" {
    const range = evdev.AbsRange{ .min = 0, .max = 1000 };
    const data = [_]evdev.InputEvent{
        iev(evdev.EV_ABS, evdev.ABS_X, 500),
        iev(evdev.EV_ABS, evdev.ABS_Y, 250),
        iev(evdev.EV_ABS, evdev.ABS_PRESSURE, 750),
        iev(evdev.EV_KEY, evdev.BTN_TOUCH, 1),
        iev(evdev.EV_SYN, evdev.SYN_REPORT, 0),
    };
    const caps = Caps{
        .has_rel = false,
        .has_buttons = false,
        .has_wheel = false,
        .has_keys = false,
        .is_tablet = true,
        .id = .{ .bustype = 3, .vendor = 0x56a, .product = 0x27, .version = 1 },
        .kind = .mouse,
        .name = undefined,
        .name_len = 0,
        .abs_x = range,
        .abs_y = range,
        .abs_pressure = range,
    };
    const dev = try Device.initMemory(std.testing.allocator, &data, caps, 42);
    defer dev.deinit(std.testing.allocator);

    var out = std.ArrayList(event.Event).empty;
    defer out.deinit(std.testing.allocator);
    try dev.dispatch(&out, std.testing.allocator);

    var saw_axis = false;
    var saw_tip = false;
    for (out.items) |e| switch (e) {
        .tablet_axis => |a| {
            try std.testing.expectApproxEqAbs(@as(f64, 0.5), a.x, 1e-9);
            try std.testing.expectApproxEqAbs(@as(f64, 0.25), a.y, 1e-9);
            try std.testing.expectApproxEqAbs(@as(f64, 0.75), a.pressure, 1e-9);
            saw_axis = true;
        },
        .tablet_tip => |t| {
            try std.testing.expect(t.down);
            saw_tip = true;
        },
        else => {},
    };
    try std.testing.expect(saw_axis);
    try std.testing.expect(saw_tip);
}

test "tablet device emits tablet_proximity on BTN_TOOL_PEN=1" {
    const range = evdev.AbsRange{ .min = 0, .max = 1000 };
    const data = [_]evdev.InputEvent{
        iev(evdev.EV_KEY, evdev.BTN_TOOL_PEN, 1),
        iev(evdev.EV_SYN, evdev.SYN_REPORT, 0),
    };
    const caps = Caps{
        .has_rel = false,
        .has_buttons = false,
        .has_wheel = false,
        .has_keys = false,
        .is_tablet = true,
        .id = .{ .bustype = 3, .vendor = 0x56a, .product = 0x27, .version = 1 },
        .kind = .mouse,
        .name = undefined,
        .name_len = 0,
        .abs_x = range,
        .abs_y = range,
        .abs_pressure = range,
    };
    const dev = try Device.initMemory(std.testing.allocator, &data, caps, 43);
    defer dev.deinit(std.testing.allocator);

    var out = std.ArrayList(event.Event).empty;
    defer out.deinit(std.testing.allocator);
    try dev.dispatch(&out, std.testing.allocator);

    var saw_proximity = false;
    for (out.items) |e| switch (e) {
        .tablet_proximity => |p| {
            try std.testing.expect(p.in_prox);
            saw_proximity = true;
        },
        else => {},
    };
    try std.testing.expect(saw_proximity);
}

/// Fixed-size stand-ins for the three bitmasks `probe` reads from the kernel, so
/// classification can be exercised with no hardware and no ioctls.
const FakeBits = struct {
    abs: [(evdev.ABS_MAX + 1 + 7) / 8]u8 = @splat(0),
    key: [(evdev.KEY_MAX + 1 + 7) / 8]u8 = @splat(0),
    prop: [(evdev.INPUT_PROP_MAX + 1 + 7) / 8]u8 = @splat(0),

    fn set(mask: []u8, bit: usize) void {
        mask[bit / 8] |= @as(u8, 1) << @intCast(bit % 8);
    }

    fn view(self: *const FakeBits) DeviceBits {
        return .{ .abs = &self.abs, .key = &self.key, .prop = &self.prop };
    }
};

/// Build the Caps `probe` would produce for these bits, then hand them to a
/// memory-backed Device so the tests check what a real device would carry.
fn deviceFromBits(bits: *const FakeBits) Error!*Device {
    const caps = Caps{
        .has_rel = false,
        .has_buttons = evdev.testBit(&bits.key, evdev.BTN_LEFT),
        .has_wheel = false,
        .has_keys = false,
        .is_tablet = isTablet(bits.view()),
        .is_touchscreen = isTouchscreen(bits.view()),
        .id = .{ .bustype = 3, .vendor = 1, .product = 1, .version = 1 },
        .kind = .mouse,
        .name = undefined,
        .name_len = 0,
    };
    return Device.initMemory(std.testing.allocator, &.{}, caps, 1);
}

test "a direct-input device with multitouch position axes is a touchscreen" {
    var bits = FakeBits{};
    FakeBits.set(&bits.prop, evdev.INPUT_PROP_DIRECT);
    FakeBits.set(&bits.abs, evdev.ABS_MT_POSITION_X);
    FakeBits.set(&bits.abs, evdev.ABS_MT_POSITION_Y);
    FakeBits.set(&bits.abs, evdev.ABS_MT_SLOT);
    FakeBits.set(&bits.abs, evdev.ABS_MT_TRACKING_ID);
    FakeBits.set(&bits.key, evdev.BTN_TOUCH);

    const dev = try deviceFromBits(&bits);
    defer dev.deinit(std.testing.allocator);
    try std.testing.expectEqual(true, dev.caps.is_touchscreen);
    try std.testing.expectEqual(false, dev.caps.is_tablet);
}

test "a multitouch device with INPUT_PROP_POINTER is a touchpad, not a touchscreen" {
    // Regression guard: a laptop touchpad exposes the same ABS_MT axes as a
    // touchscreen. Classifying it as one hands a plain laptop the tablet layout.
    var bits = FakeBits{};
    FakeBits.set(&bits.prop, evdev.INPUT_PROP_POINTER);
    FakeBits.set(&bits.abs, evdev.ABS_X);
    FakeBits.set(&bits.abs, evdev.ABS_Y);
    FakeBits.set(&bits.abs, evdev.ABS_MT_POSITION_X);
    FakeBits.set(&bits.abs, evdev.ABS_MT_POSITION_Y);
    FakeBits.set(&bits.abs, evdev.ABS_MT_SLOT);
    FakeBits.set(&bits.abs, evdev.ABS_MT_TRACKING_ID);
    FakeBits.set(&bits.key, evdev.BTN_TOUCH);
    FakeBits.set(&bits.key, evdev.BTN_LEFT);

    const dev = try deviceFromBits(&bits);
    defer dev.deinit(std.testing.allocator);
    try std.testing.expectEqual(false, dev.caps.is_touchscreen);
    try std.testing.expectEqual(false, dev.caps.is_tablet);
}

test "a pen tablet is still a tablet and is not a touchscreen" {
    // A display tablet sets INPUT_PROP_DIRECT and BTN_TOUCH as well, so only
    // BTN_TOOL_PEN keeps it out of the touchscreen answer.
    var bits = FakeBits{};
    FakeBits.set(&bits.prop, evdev.INPUT_PROP_DIRECT);
    FakeBits.set(&bits.abs, evdev.ABS_X);
    FakeBits.set(&bits.abs, evdev.ABS_Y);
    FakeBits.set(&bits.abs, evdev.ABS_PRESSURE);
    FakeBits.set(&bits.key, evdev.BTN_TOOL_PEN);
    FakeBits.set(&bits.key, evdev.BTN_TOUCH);
    FakeBits.set(&bits.key, evdev.BTN_STYLUS);

    const dev = try deviceFromBits(&bits);
    defer dev.deinit(std.testing.allocator);
    try std.testing.expectEqual(true, dev.caps.is_tablet);
    try std.testing.expectEqual(false, dev.caps.is_touchscreen);
}

test "an ordinary mouse is neither a touchscreen nor a tablet" {
    var bits = FakeBits{};
    FakeBits.set(&bits.key, evdev.BTN_LEFT);
    FakeBits.set(&bits.key, evdev.BTN_RIGHT);
    FakeBits.set(&bits.key, evdev.BTN_MIDDLE);

    const dev = try deviceFromBits(&bits);
    defer dev.deinit(std.testing.allocator);
    try std.testing.expectEqual(false, dev.caps.is_touchscreen);
    try std.testing.expectEqual(false, dev.caps.is_tablet);
}

test "a single-touch direct device with BTN_TOUCH and absolute axes is a touchscreen" {
    // Pre-multitouch screens report one contact and carry no ABS_MT axes.
    var bits = FakeBits{};
    FakeBits.set(&bits.prop, evdev.INPUT_PROP_DIRECT);
    FakeBits.set(&bits.abs, evdev.ABS_X);
    FakeBits.set(&bits.abs, evdev.ABS_Y);
    FakeBits.set(&bits.key, evdev.BTN_TOUCH);

    const dev = try deviceFromBits(&bits);
    defer dev.deinit(std.testing.allocator);
    try std.testing.expectEqual(true, dev.caps.is_touchscreen);
    try std.testing.expectEqual(false, dev.caps.is_tablet);
}

test "openFd rejects a non-character fd without owning it" {
    var fds: [2]i32 = undefined;
    const rc = linux.pipe2(&fds, linux.O{ .NONBLOCK = true, .CLOEXEC = true });
    if (posix.errno(rc) != .SUCCESS) return error.SkipZigTest;
    defer _ = linux.close(fds[0]);
    defer _ = linux.close(fds[1]);

    // Read end of a pipe: statx TYPE is FIFO, not char device -> NotACharDevice.
    try std.testing.expectError(error.NotACharDevice, Device.openFd(std.testing.allocator, fds[0], 1));
}

test {
    std.testing.refAllDecls(@This());
}
