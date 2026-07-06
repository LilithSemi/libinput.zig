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

pub const Caps = struct {
    has_rel: bool,
    has_buttons: bool,
    has_wheel: bool,
    id: evdev.InputId,
    kind: filter.Kind,
    name: [64]u8,
    name_len: usize,
};

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

    /// Open a real evdev device node. Probes capabilities via ioctls.
    pub fn open(gpa: std.mem.Allocator, path: []const u8, id: u32) Error!*Device {
        const path_z = gpa.dupeZ(u8, path) catch return error.OpenFailed;
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
        var rel_bits = [_]u8{0} ** ((evdev.REL_MAX + 1 + 7) / 8);
        switch (posix.errno(linux.ioctl(the_fd, evdev.EVIOCGBIT(evdev.EV_REL, rel_bits.len), @intFromPtr(&rel_bits)))) {
            .SUCCESS => {},
            else => return error.IoctlFailed,
        }
        caps.has_rel = evdev.testBit(&rel_bits, evdev.REL_X) or evdev.testBit(&rel_bits, evdev.REL_Y);
        caps.has_wheel = evdev.testBit(&rel_bits, evdev.REL_WHEEL) or
            evdev.testBit(&rel_bits, evdev.REL_WHEEL_HI_RES);

        // EVIOCGBIT(EV_KEY): key/button capability bitmask.
        // KEY_MAX = 0x2ff = 767, nbits(768) = 96 bytes.
        var key_bits = [_]u8{0} ** ((evdev.KEY_MAX + 1 + 7) / 8);
        switch (posix.errno(linux.ioctl(the_fd, evdev.EVIOCGBIT(evdev.EV_KEY, key_bits.len), @intFromPtr(&key_bits)))) {
            .SUCCESS => {},
            else => return error.IoctlFailed,
        }
        caps.has_buttons = evdev.testBit(&key_bits, evdev.BTN_LEFT) or
            evdev.testBit(&key_bits, evdev.BTN_RIGHT) or
            evdev.testBit(&key_bits, evdev.BTN_MIDDLE);

        // trackpoint: name match, or has BTN_MIDDLE with relative axes but no wheel (pointing-stick heuristic).
        const name_slice = caps.name[0..caps.name_len];
        const is_trackpoint = std.mem.indexOf(u8, name_slice, "TrackPoint") != null or
            std.mem.indexOf(u8, name_slice, "Stick") != null or
            (caps.has_rel and !caps.has_wheel and evdev.testBit(&key_bits, evdev.BTN_MIDDLE));
        caps.kind = if (is_trackpoint) .trackpoint else .mouse;

        return makeDevice(gpa, .{ .fd = the_fd }, caps, id);
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
                        self.proc.flush(time_ms, out, gpa) catch return error.Unexpected;
                    } else if (e.code == evdev.SYN_DROPPED) {
                        self.proc.reset();
                    }
                } else {
                    self.proc.feed(e);
                }
            }
        }
    }

    /// Release the fd (if any) and free the Device allocation.
    pub fn deinit(self: *Device, gpa: std.mem.Allocator) void {
        if (self.fd()) |f| _ = linux.close(f);
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

test {
    std.testing.refAllDecls(@This());
}
