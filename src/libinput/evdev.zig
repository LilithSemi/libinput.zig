//! Linux evdev ABI: struct layouts, event codes, and ioctl request builders.

const std = @import("std");

test "InputEvent has kernel ABI layout" {
    // On 64-bit Linux, input_event is timeval(16) + type(2) + code(2) + value(4) = 24.
    try std.testing.expectEqual(@as(usize, 24), @sizeOf(InputEvent));
    try std.testing.expectEqual(@as(usize, 0), @offsetOf(InputEvent, "time"));
    try std.testing.expectEqual(@as(usize, 16), @offsetOf(InputEvent, "type"));
    try std.testing.expectEqual(@as(usize, 20), @offsetOf(InputEvent, "value"));
}

test "testBit reads a little-endian byte bitmask" {
    var mask = [_]u8{0} ** 4;
    mask[0] = 0b0000_0100; // bit 2 set
    mask[1] = 0b0000_0001; // bit 8 set
    try std.testing.expect(testBit(&mask, 2));
    try std.testing.expect(testBit(&mask, 8));
    try std.testing.expect(!testBit(&mask, 3));
}

test "EVIOCGBIT encodes direction/size/type/nr" {
    // _IOC(READ=2, 'E'=0x45, 0x20+EV_REL, len). Compare against the shape,
    // not a hardcoded magic, by decoding the fields back out.
    const req = EVIOCGBIT(EV_REL, 8);
    try std.testing.expectEqual(@as(u32, 8), (req >> 16) & 0x3fff); // size field
    try std.testing.expectEqual(@as(u32, 0x45), (req >> 8) & 0xff); // 'E'
    try std.testing.expectEqual(@as(u32, 0x20 + EV_REL), req & 0xff); // nr
    try std.testing.expectEqual(@as(u32, 2), (req >> 30) & 0x3); // _IOC_READ
}

pub const TimeVal = extern struct { sec: isize, usec: isize };

pub const InputEvent = extern struct {
    time: TimeVal,
    type: u16,
    code: u16,
    value: i32,
};

pub const InputId = extern struct {
    bustype: u16,
    vendor: u16,
    product: u16,
    version: u16,
};

pub const EV_SYN = 0x00;
pub const EV_KEY = 0x01;
pub const EV_REL = 0x02;
pub const EV_ABS = 0x03;
pub const EV_MSC = 0x04;
pub const EV_MAX = 0x1f;

pub const SYN_REPORT = 0;
pub const SYN_DROPPED = 3;

pub const REL_X = 0x00;
pub const REL_Y = 0x01;
pub const REL_HWHEEL = 0x06;
pub const REL_WHEEL = 0x08;
pub const REL_WHEEL_HI_RES = 0x0b;
pub const REL_HWHEEL_HI_RES = 0x0c;
pub const REL_MAX = 0x0f;

pub const BTN_LEFT = 0x110;
pub const BTN_RIGHT = 0x111;
pub const BTN_MIDDLE = 0x112;
pub const BTN_SIDE = 0x113;
pub const BTN_EXTRA = 0x114;
pub const BTN_FORWARD = 0x115;
pub const BTN_BACK = 0x116;
pub const BTN_TASK = 0x117;
pub const KEY_MAX = 0x2ff;

pub fn nbits(bits: usize) usize {
    return (bits + 7) / 8;
}

pub fn testBit(arr: []const u8, bit: usize) bool {
    const byte = bit / 8;
    if (byte >= arr.len) return false;
    return (arr[byte] & (@as(u8, 1) << @intCast(bit % 8))) != 0;
}

// _IOC encoding (asm-generic/ioctl.h): dir[31:30] size[29:16] type[15:8] nr[7:0]
const IOC_NONE: u32 = 0;
const IOC_WRITE: u32 = 1;
const IOC_READ: u32 = 2;

fn ioc(dir: u32, typ: u32, nr: u32, size: u32) u32 {
    return (dir << 30) | (size << 16) | (typ << 8) | nr;
}

const EV = 0x45; // 'E'

pub fn EVIOCGID() u32 {
    return ioc(IOC_READ, EV, 0x02, @sizeOf(InputId));
}

pub fn EVIOCGNAME(len: usize) u32 {
    return ioc(IOC_READ, EV, 0x06, @intCast(len));
}

pub fn EVIOCGBIT(ev: u32, len: usize) u32 {
    return ioc(IOC_READ, EV, 0x20 + ev, @intCast(len));
}

pub fn EVIOCGRAB() u32 {
    return ioc(IOC_WRITE, EV, 0x90, @sizeOf(c_int));
}
