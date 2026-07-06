//! The injectable read boundary. Real devices use `.fd`, tests use `.memory`.

const std = @import("std");
const linux = std.os.linux;
const posix = std.posix;
const evdev = @import("evdev.zig");

pub const Error = error{ Io, WouldBlock, Closed, Unexpected };

pub const Memory = struct {
    events: []const evdev.InputEvent,
    pos: usize = 0,
};

pub const EventSource = union(enum) {
    fd: posix.fd_t,
    memory: Memory,

    pub fn read(self: *EventSource, buf: []evdev.InputEvent) Error!usize {
        switch (self.*) {
            .memory => |*m| {
                if (m.pos >= m.events.len) return Error.Closed;
                const remaining = m.events.len - m.pos;
                const n = @min(remaining, buf.len);
                @memcpy(buf[0..n], m.events[m.pos .. m.pos + n]);
                m.pos += n;
                return n;
            },
            .fd => |fd| {
                const bytes = std.mem.sliceAsBytes(buf);
                const rc = linux.read(fd, bytes.ptr, bytes.len);
                switch (posix.errno(rc)) {
                    .SUCCESS => {
                        const got: usize = @intCast(rc);
                        if (got == 0) return Error.Closed;
                        return got / @sizeOf(evdev.InputEvent);
                    },
                    .AGAIN => return Error.WouldBlock,
                    .INTR => return Error.WouldBlock,
                    .NODEV => return Error.Closed,
                    else => return Error.Unexpected,
                }
            },
        }
    }
};

fn ev(t: u16, c: u16, v: i32) evdev.InputEvent {
    return .{ .time = .{ .sec = 0, .usec = 0 }, .type = t, .code = c, .value = v };
}

test "memory source yields events then reports Closed" {
    const data = [_]evdev.InputEvent{
        ev(evdev.EV_REL, evdev.REL_X, 5),
        ev(evdev.EV_SYN, evdev.SYN_REPORT, 0),
    };
    var src = EventSource{ .memory = .{ .events = &data, .pos = 0 } };
    var buf: [8]evdev.InputEvent = undefined;
    const n = try src.read(&buf);
    try std.testing.expectEqual(@as(usize, 2), n);
    try std.testing.expectEqual(@as(i32, 5), buf[0].value);
    try std.testing.expectError(Error.Closed, src.read(&buf));
}

test {
    std.testing.refAllDecls(@This());
}
