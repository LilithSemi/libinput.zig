//! Opt-in integration test: create a virtual relative mouse via /dev/uinput,
//! emit a motion, and read it back through the real fd path. Skips cleanly when
//! /dev/uinput is unavailable (CI sandboxes, cross builds, unprivileged hosts).

const std = @import("std");
const linux = std.os.linux;
const posix = std.posix;
const libinput = @import("libinput");
const evdev = libinput.evdev;
const Context = libinput.Context;

// IOC encoding (asm-generic/ioctl.h): dir[31:30] size[29:16] type[15:8] nr[7:0]
const IOC_NONE: u32 = 0;
const IOC_WRITE: u32 = 1;
const UINPUT_TYPE: u32 = 0x55; // 'U'

fn uioc(dir: u32, nr: u32, size: u32) u32 {
    return (dir << 30) | (size << 16) | (UINPUT_TYPE << 8) | nr;
}

// struct uinput_setup { struct input_id id; char name[80]; __u32 ff_effects_max; }
// Matches the kernel layout: id(8) + name(80) + ff_effects_max(4) = 92 bytes.
const UinputSetup = extern struct {
    id: evdev.InputId,
    name: [80]u8,
    ff_effects_max: u32,
};

// uinput ioctl request codes (linux/uinput.h)
const UI_DEV_CREATE: u32 = uioc(IOC_NONE, 1, 0);
const UI_DEV_DESTROY: u32 = uioc(IOC_NONE, 2, 0);
const UI_DEV_SETUP: u32 = uioc(IOC_WRITE, 3, @sizeOf(UinputSetup));
const UI_SET_EVBIT: u32 = uioc(IOC_WRITE, 100, @sizeOf(i32));
const UI_SET_KEYBIT: u32 = uioc(IOC_WRITE, 101, @sizeOf(i32));
const UI_SET_RELBIT: u32 = uioc(IOC_WRITE, 102, @sizeOf(i32));

const DEVICE_NAME = "libinput.zig-test";
const MOTION_VAL: i32 = 5;

test "uinput virtual mouse roundtrips a motion event" {
    // Open /dev/uinput. Skip if unavailable (ENOENT, EACCES, ENODEV, ...).
    const ufd_rc = linux.openat(
        linux.AT.FDCWD,
        "/dev/uinput",
        linux.O{ .ACCMODE = .WRONLY, .NONBLOCK = true, .CLOEXEC = true },
        0,
    );
    if (posix.errno(ufd_rc) != .SUCCESS) return;
    const ufd: posix.fd_t = @intCast(ufd_rc);
    var device_created = false;
    defer {
        if (device_created) _ = linux.ioctl(ufd, UI_DEV_DESTROY, 0);
        _ = linux.close(ufd);
    }

    // Set required capability bits. A failure means the environment forbids
    // uinput operations, so skip rather than fail.
    if (posix.errno(linux.ioctl(ufd, UI_SET_EVBIT, @as(usize, evdev.EV_REL))) != .SUCCESS) return;
    if (posix.errno(linux.ioctl(ufd, UI_SET_EVBIT, @as(usize, evdev.EV_KEY))) != .SUCCESS) return;
    if (posix.errno(linux.ioctl(ufd, UI_SET_RELBIT, @as(usize, evdev.REL_X))) != .SUCCESS) return;
    if (posix.errno(linux.ioctl(ufd, UI_SET_KEYBIT, @as(usize, evdev.BTN_LEFT))) != .SUCCESS) return;

    // Describe the virtual device and create it.
    var setup = UinputSetup{
        .id = .{ .bustype = 3, .vendor = 0x1234, .product = 0x5678, .version = 1 },
        .name = [_]u8{0} ** 80,
        .ff_effects_max = 0,
    };
    @memcpy(setup.name[0..DEVICE_NAME.len], DEVICE_NAME);
    if (posix.errno(linux.ioctl(ufd, UI_DEV_SETUP, @intFromPtr(&setup))) != .SUCCESS) return;
    if (posix.errno(linux.ioctl(ufd, UI_DEV_CREATE, 0)) != .SUCCESS) return;
    device_created = true;

    // Scan /dev/input for the created eventN node, retrying with 10ms gaps so
    // udev has a chance to create the node on devtmpfs.
    var ep_buf = [_]u8{0} ** 64;
    var ep_len: usize = 0;

    find: for (0..20) |_| {
        scan: {
            const dir_rc = linux.openat(
                linux.AT.FDCWD,
                "/dev/input",
                linux.O{ .ACCMODE = .RDONLY, .DIRECTORY = true, .CLOEXEC = true },
                0,
            );
            if (posix.errno(dir_rc) != .SUCCESS) break :scan;
            const dir_fd: posix.fd_t = @intCast(dir_rc);
            defer _ = linux.close(dir_fd);

            var dents: [4096]u8 align(@alignOf(linux.dirent64)) = undefined;
            while (true) {
                const n = linux.getdents64(dir_fd, &dents, dents.len);
                if (posix.errno(n) != .SUCCESS or n == 0) break;
                var off: usize = 0;
                while (off < n) {
                    const d: *linux.dirent64 = @ptrCast(@alignCast(&dents[off]));
                    const name_ptr: [*:0]const u8 = @ptrCast(&d.name);
                    const ename = std.mem.span(name_ptr);
                    if (std.mem.startsWith(u8, ename, "event")) {
                        const prefix = "/dev/input/";
                        const total = prefix.len + ename.len;
                        if (total < ep_buf.len) {
                            var p_buf = [_]u8{0} ** 64;
                            @memcpy(p_buf[0..prefix.len], prefix);
                            @memcpy(p_buf[prefix.len..][0..ename.len], ename);
                            const path_z: [*:0]const u8 = @ptrCast(&p_buf);
                            const efd_rc = linux.openat(
                                linux.AT.FDCWD,
                                path_z,
                                linux.O{ .ACCMODE = .RDONLY, .NONBLOCK = true, .CLOEXEC = true },
                                0,
                            );
                            if (posix.errno(efd_rc) == .SUCCESS) {
                                const efd: posix.fd_t = @intCast(efd_rc);
                                defer _ = linux.close(efd);
                                var nb: [128]u8 = undefined;
                                const nrc = linux.ioctl(
                                    efd,
                                    evdev.EVIOCGNAME(nb.len),
                                    @intFromPtr(&nb),
                                );
                                if (posix.errno(nrc) == .SUCCESS) {
                                    const got_name = std.mem.sliceTo(&nb, 0);
                                    if (std.mem.eql(u8, got_name, DEVICE_NAME)) {
                                        @memcpy(ep_buf[0..total], p_buf[0..total]);
                                        ep_len = total;
                                        break :find;
                                    }
                                }
                            }
                        }
                    }
                    off += @as(usize, d.reclen);
                }
            }
        }
        const ts = linux.timespec{ .sec = 0, .nsec = 10 * 1_000_000 };
        _ = linux.nanosleep(&ts, null);
    }

    if (ep_len == 0) return; // device node not found, skip.

    // Open the virtual device through the libinput context.
    var ctx = Context.init(std.testing.allocator);
    defer ctx.deinit();
    const h = ctx.addDevice(ep_buf[0..ep_len]) catch return;
    h.setAccelProfile(.flat);
    h.setAccelSpeed(0.0);

    // Inject a REL_X motion and a SYN_REPORT through the uinput fd.
    const ev_rel = evdev.InputEvent{
        .time = .{ .sec = 0, .usec = 0 },
        .type = evdev.EV_REL,
        .code = evdev.REL_X,
        .value = MOTION_VAL,
    };
    const ev_syn = evdev.InputEvent{
        .time = .{ .sec = 0, .usec = 0 },
        .type = evdev.EV_SYN,
        .code = evdev.SYN_REPORT,
        .value = 0,
    };
    if (posix.errno(linux.write(ufd, std.mem.asBytes(&ev_rel).ptr, @sizeOf(evdev.InputEvent))) != .SUCCESS) return;
    if (posix.errno(linux.write(ufd, std.mem.asBytes(&ev_syn).ptr, @sizeOf(evdev.InputEvent))) != .SUCCESS) return;

    // Pump the event queue and verify the roundtrip.
    ctx.dispatch() catch return;
    _ = ctx.next(); // consume .device_added
    const got = ctx.next() orelse return;
    if (got != .pointer_motion) return;
    try std.testing.expectEqual(
        @as(f64, @floatFromInt(MOTION_VAL)),
        got.pointer_motion.dx_unaccel,
    );
}
