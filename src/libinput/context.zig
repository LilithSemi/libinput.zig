//! Context owns a device list, an event queue, and a device ID counter.
//! Callers add devices, call dispatch() to pump events, then drain with next().
//!
//! read_cursor tracks consumed events. Each dispatch() compacts the consumed
//! prefix before appending, so the backing buffer stays bounded.
//!
//! DeviceGone handles are collected during dispatch() then removed via
//! swapRemove in descending index order, so lower indices stay valid.

const std = @import("std");
const posix = std.posix;
const evdev_mod = @import("evdev.zig");
const device = @import("device.zig");
const event = @import("event.zig");
const filter = @import("filter.zig");
const pointer = @import("pointer.zig");

pub const DeviceHandle = struct {
    dev: *device.Device,

    /// Set the pointer acceleration speed, clamped to [-1, 1].
    pub fn setAccelSpeed(self: *DeviceHandle, s: f64) void {
        self.dev.filterPtr().setSpeed(s);
    }

    /// Set the pointer acceleration profile (flat or adaptive).
    pub fn setAccelProfile(self: *DeviceHandle, p: filter.Profile) void {
        self.dev.filterPtr().setProfile(p);
    }

    pub fn setMiddleEmulation(self: *DeviceHandle, on: bool) void {
        self.dev.config().middle_emulation = on;
    }

    pub fn setScrollMethod(self: *DeviceHandle, m: pointer.ScrollMethod) void {
        self.dev.config().scroll_method = m;
    }

    /// Enable or disable natural scroll (reversed axis direction).
    pub fn setNaturalScroll(self: *DeviceHandle, on: bool) void {
        self.dev.config().natural_scroll = on;
    }

    /// Enable or disable left-handed mode (swaps left and right buttons).
    pub fn setLeftHanded(self: *DeviceHandle, on: bool) void {
        self.dev.config().left_handed = on;
    }
};

pub const Context = struct {
    gpa: std.mem.Allocator,
    handles: std.ArrayList(*DeviceHandle),
    queue: std.ArrayList(event.Event),
    read_cursor: usize,
    next_id: u32,

    /// Create a new, empty context backed by `gpa`.
    pub fn init(gpa: std.mem.Allocator) Context {
        return .{
            .gpa = gpa,
            .handles = std.ArrayList(*DeviceHandle).empty,
            .queue = std.ArrayList(event.Event).empty,
            .read_cursor = 0,
            .next_id = 1,
        };
    }

    /// Release all devices and free all memory.
    pub fn deinit(self: *Context) void {
        for (self.handles.items) |h| {
            h.dev.deinit(self.gpa);
            self.gpa.destroy(h);
        }
        self.handles.deinit(self.gpa);
        self.queue.deinit(self.gpa);
    }

    // on error, the handle is never appended, so the caller must deinit dev.
    fn newHandle(self: *Context, dev: *device.Device) !*DeviceHandle {
        const h = try self.gpa.create(DeviceHandle);
        errdefer self.gpa.destroy(h);
        h.dev = dev;
        try self.handles.append(self.gpa, h);
        return h;
    }

    /// Open a real evdev device node, assign it the next available id, and
    /// enqueue a `device_added` event. Returns a pointer to the handle.
    pub fn addDevice(self: *Context, path: []const u8) !*DeviceHandle {
        const id = self.next_id;
        self.next_id += 1;
        const dev = try device.Device.open(self.gpa, path, id);
        // newHandle failure path: handle was never appended, only dev needs cleanup.
        const h = self.newHandle(dev) catch |e| {
            dev.deinit(self.gpa);
            return e;
        };
        // queue.append failure path: handle IS appended and owns dev, so undo all three.
        errdefer {
            _ = self.handles.pop();
            self.gpa.destroy(h);
            dev.deinit(self.gpa);
        }
        try self.queue.append(self.gpa, .{ .device_added = .{ .device_id = id } });
        return h;
    }

    /// Test hook: create a memory-backed device from pre-supplied event data
    /// and capability flags. Enqueues a `device_added` event.
    pub fn addMemoryDevice(
        self: *Context,
        events: []const evdev_mod.InputEvent,
        caps: device.Caps,
    ) !*DeviceHandle {
        const id = self.next_id;
        self.next_id += 1;
        const dev = try device.Device.initMemory(self.gpa, events, caps, id);
        // newHandle failure path: handle was never appended, only dev needs cleanup.
        const h = self.newHandle(dev) catch |e| {
            dev.deinit(self.gpa);
            return e;
        };
        // queue.append failure path: handle IS appended and owns dev, so undo all three.
        errdefer {
            _ = self.handles.pop();
            self.gpa.destroy(h);
            dev.deinit(self.gpa);
        }
        try self.queue.append(self.gpa, .{ .device_added = .{ .device_id = id } });
        return h;
    }

    /// Pump events from all registered devices into the queue.
    ///
    /// Real-fd devices: a non-blocking poll(timeout=0) determines readiness,
    /// and only ready fds are dispatched. Memory devices are always dispatched.
    ///
    /// DeviceGone enqueues device_removed and cleans up the handle. The error
    /// does not propagate.
    pub fn dispatch(self: *Context) !void {
        if (self.read_cursor > 0) {
            const remaining = self.queue.items.len - self.read_cursor;
            if (remaining > 0) {
                std.mem.copyForwards(
                    event.Event,
                    self.queue.items[0..remaining],
                    self.queue.items[self.read_cursor..],
                );
            }
            self.queue.items.len = remaining;
            self.read_cursor = 0;
        }

        // Build pollfd array for fd-backed devices, remembering handle indices.
        var pollfds = std.ArrayList(posix.pollfd).empty;
        defer pollfds.deinit(self.gpa);
        var fd_handle_idx = std.ArrayList(usize).empty;
        defer fd_handle_idx.deinit(self.gpa);

        for (self.handles.items, 0..) |h, i| {
            if (h.dev.fd()) |fd| {
                try pollfds.append(self.gpa, .{
                    .fd = fd,
                    .events = posix.POLL.IN,
                    .revents = 0,
                });
                try fd_handle_idx.append(self.gpa, i);
            }
        }

        // Non-blocking poll for real fds (timeout 0 = return immediately).
        // Retry on EINTR rather than propagating it as an error.
        if (pollfds.items.len > 0) {
            while (true) {
                _ = posix.poll(pollfds.items, 0) catch |e| {
                    if (e == error.Interrupted) continue;
                    return e;
                };
                break;
            }
        }

        // Collect indices of handles that must be removed after the loops.
        var to_remove = std.ArrayList(usize).empty;
        defer to_remove.deinit(self.gpa);

        for (pollfds.items, 0..) |pfd, pi| {
            // Dispatch on IN, HUP, or ERR so gone devices are detected promptly.
            if (pfd.revents != 0) {
                const hi = fd_handle_idx.items[pi];
                const h = self.handles.items[hi];
                h.dev.dispatch(&self.queue, self.gpa) catch |err| switch (err) {
                    error.DeviceGone => {
                        try self.queue.append(self.gpa, .{
                            .device_removed = .{ .device_id = h.dev.device_id },
                        });
                        try to_remove.append(self.gpa, hi);
                    },
                    else => return err,
                };
            }
        }

        // Always dispatch memory-backed devices once per dispatch() call.
        for (self.handles.items, 0..) |h, i| {
            if (h.dev.fd() == null) {
                h.dev.dispatch(&self.queue, self.gpa) catch |err| switch (err) {
                    error.DeviceGone => {
                        try self.queue.append(self.gpa, .{
                            .device_removed = .{ .device_id = h.dev.device_id },
                        });
                        try to_remove.append(self.gpa, i);
                    },
                    else => return err,
                };
            }
        }

        // Remove dead handles. Sort indices descending so that swapRemove(hi)
        // never invalidates a lower index we have yet to process.
        if (to_remove.items.len > 0) {
            {
                var si: usize = 1;
                while (si < to_remove.items.len) : (si += 1) {
                    const v = to_remove.items[si];
                    var sj: usize = si;
                    while (sj > 0 and to_remove.items[sj - 1] < v) {
                        to_remove.items[sj] = to_remove.items[sj - 1];
                        sj -= 1;
                    }
                    to_remove.items[sj] = v;
                }
            }
            for (to_remove.items) |hi| {
                const h = self.handles.items[hi];
                h.dev.deinit(self.gpa);
                self.gpa.destroy(h);
                _ = self.handles.swapRemove(hi);
            }
        }
    }

    /// Pop the next event from the queue in FIFO order.
    /// Returns null when no events are pending.
    pub fn next(self: *Context) ?event.Event {
        if (self.read_cursor >= self.queue.items.len) return null;
        const e = self.queue.items[self.read_cursor];
        self.read_cursor += 1;
        return e;
    }
};

const evdev = @import("evdev.zig");

fn iev(t: u16, c: u16, v: i32) evdev.InputEvent {
    return .{ .time = .{ .sec = 0, .usec = 0 }, .type = t, .code = c, .value = v };
}

test "addMemoryDevice: no double-free when queue.append OOM after newHandle" {
    const data = [_]evdev.InputEvent{
        iev(evdev.EV_REL, evdev.REL_X, 1),
        iev(evdev.EV_SYN, evdev.SYN_REPORT, 0),
    };
    const caps = device.Caps{
        .has_rel = true,
        .has_buttons = true,
        .has_wheel = true,
        .id = .{ .bustype = 3, .vendor = 1, .product = 1, .version = 1 },
        .kind = .mouse,
        .name = undefined,
        .name_len = 0,
    };

    // fail_index=3 hits queue.append after the handle is already in self.handles.
    // The errdefer must pop it, destroy it, and deinit dev without a double-free.
    var failing = std.testing.FailingAllocator.init(std.testing.allocator, .{ .fail_index = 3 });
    var ctx = Context.init(failing.allocator());
    defer ctx.deinit();

    try std.testing.expectError(error.OutOfMemory, ctx.addMemoryDevice(&data, caps));
    // errdefer must have removed the handle.
    try std.testing.expectEqual(@as(usize, 0), ctx.handles.items.len);
    // deinit via defer above must not double-free.
}

test "context delivers device_added then a motion event" {
    const data = [_]evdev.InputEvent{
        iev(evdev.EV_REL, evdev.REL_X, 7),
        iev(evdev.EV_SYN, evdev.SYN_REPORT, 0),
    };
    const caps = device.Caps{
        .has_rel = true,
        .has_buttons = true,
        .has_wheel = true,
        .id = .{ .bustype = 3, .vendor = 1, .product = 1, .version = 1 },
        .kind = .mouse,
        .name = undefined,
        .name_len = 0,
    };

    var ctx = Context.init(std.testing.allocator);
    defer ctx.deinit();
    const h = try ctx.addMemoryDevice(&data, caps);
    h.setAccelProfile(.flat);
    h.setAccelSpeed(0.0);

    try ctx.dispatch();

    const first = ctx.next().?;
    try std.testing.expect(first == .device_added);
    const second = ctx.next().?;
    try std.testing.expectEqual(@as(f64, 7), second.pointer_motion.dx_unaccel);
}

test {
    std.testing.refAllDecls(@This());
}
