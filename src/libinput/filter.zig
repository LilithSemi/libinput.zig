//! Pointer acceleration filter: a faithful port of libinput's motion filter.
//!
//! Ported from libinput 1.31.0 (`src/filter.c`, `src/filter-mouse.c`,
//! `src/filter-trackpoint.c`, `src/filter-flat.c`, `src/filter-private.h`).
//! All internal time is microseconds and all velocities are units/us, matching
//! the C source. `filter()` takes the timestamp as a parameter and never reads a
//! clock, so it is pure and deterministic.
//!
//! Fidelity note: libinput's flat profile (`accelerator_filter_flat`) applies a
//! constant factor to the *device* delta and does NOT DPI-normalize. We keep
//! that behavior faithfully. Only the adaptive/linear (mouse) path normalizes to
//! 1000dpi. The trackpoint path scales by a device multiplier instead of dpi.

const std = @import("std");

/// Acceleration profile. `flat` is a motion-independent constant factor,
/// `adaptive` is libinput's velocity-dependent curve.
pub const Profile = enum { flat, adaptive };

/// Device class. Selects which adaptive curve is used.
pub const Kind = enum { mouse, trackpoint };

// constants from the libinput source

/// `#define DEFAULT_MOUSE_DPI 1000` (libinput-util.h).
const DEFAULT_MOUSE_DPI: f64 = 1000.0;

/// Motion history ring size. libinput uses 16 trackers when velocity averaging
/// is enabled (`trackers_init(&filter->trackers, 16)`).
const MOTION_TRACKER_HISTORY_SIZE = 16;

/// `#define MOTION_TIMEOUT usec_from_millis(1000)` in filter.c (1000ms).
const MOTION_TIMEOUT_US: u64 = 1_000_000;

/// `const double MAX_VELOCITY_DIFF = v_usec_from_millis(1)` -> 0.001 units/us.
const MAX_VELOCITY_DIFF: f64 = 0.001;

/// `#define DEFAULT_THRESHOLD v_usec_from_millis(0.4)` -> 0.0004 units/us.
const DEFAULT_THRESHOLD: f64 = 0.4 / 1000.0;
/// `#define MINIMUM_THRESHOLD v_usec_from_millis(0.2)` -> 0.0002 units/us.
const MINIMUM_THRESHOLD: f64 = 0.2 / 1000.0;
/// `#define DEFAULT_ACCELERATION 2.0`.
const DEFAULT_ACCELERATION: f64 = 2.0;
/// `#define DEFAULT_INCLINE 1.1`.
const DEFAULT_INCLINE: f64 = 1.1;

/// Trackpoint delta smoothener: `pointer_delta_smoothener_create(usec_from_millis(10), usec_from_millis(10))`.
const TRACKPOINT_SMOOTHENER_THRESHOLD_US: u64 = 10_000;
const TRACKPOINT_SMOOTHENER_VALUE_US: u64 = 10_000;

/// Default trackpoint multiplier (from the quirks system, 1.0 when unset).
const DEFAULT_TRACKPOINT_MULTIPLIER: f64 = 1.0;

// Direction octant bits (libinput-private.h `enum directions`).
const DIR_N: u32 = 1 << 0;
const DIR_NE: u32 = 1 << 1;
const DIR_E: u32 = 1 << 2;
const DIR_SE: u32 = 1 << 3;
const DIR_S: u32 = 1 << 4;
const DIR_SW: u32 = 1 << 5;
const DIR_W: u32 = 1 << 6;
const DIR_NW: u32 = 1 << 7;
const UNDEFINED_DIRECTION: u32 = 0xff;

/// Convert a velocity in units/us to units/ms (`v_us2ms`).
inline fn v_us2ms(units_per_us: f64) f64 {
    return units_per_us * 1000.0;
}

/// Port of `xy_get_direction` (libinput-private.h): a bitmask of the octant(s)
/// the vector points into, used to reject velocity samples that changed heading.
fn getDirection(x: f64, y: f64) u32 {
    var dir: u32 = UNDEFINED_DIRECTION;
    if (@abs(x) < 2.0 and @abs(y) < 2.0) {
        if (x > 0.0 and y > 0.0) {
            dir = DIR_S | DIR_SE | DIR_E;
        } else if (x > 0.0 and y < 0.0) {
            dir = DIR_N | DIR_NE | DIR_E;
        } else if (x < 0.0 and y > 0.0) {
            dir = DIR_S | DIR_SW | DIR_W;
        } else if (x < 0.0 and y < 0.0) {
            dir = DIR_N | DIR_NW | DIR_W;
        } else if (x > 0.0) {
            dir = DIR_NE | DIR_E | DIR_SE;
        } else if (x < 0.0) {
            dir = DIR_NW | DIR_W | DIR_SW;
        } else if (y > 0.0) {
            dir = DIR_SE | DIR_S | DIR_SW;
        } else if (y < 0.0) {
            dir = DIR_NE | DIR_N | DIR_NW;
        }
        // x == 0 && y == 0 -> UNDEFINED_DIRECTION
    } else {
        const pi = std.math.pi;
        var r = std.math.atan2(y, x);
        // r = [0 .. 2pi), 0 == North, then scaled into [0 .. 8).
        r = @mod(r + 2.5 * pi, 2.0 * pi);
        r *= 4.0 / pi;
        const d1: u5 = @intCast(@mod(@as(i64, @intFromFloat(r + 0.9)), 8));
        const d2: u5 = @intCast(@mod(@as(i64, @intFromFloat(r + 0.1)), 8));
        dir = (@as(u32, 1) << d1) | (@as(u32, 1) << d2);
    }
    return dir;
}

/// One motion-history sample. `dx`/`dy` accumulate the delta *since* this
/// tracker was the current one (libinput adds each new delta to every tracker).
const Tracker = struct {
    dx: f64 = 0.0,
    dy: f64 = 0.0,
    time_us: u64 = 0,
    dir: u32 = 0,
};

/// Speed->constant mapping for the trackpoint (`speed_factor` in filter-trackpoint.c).
fn trackpointSpeedFactor(speed: f64) f64 {
    const s = speed + 1.0; // map [-1, 1] to [0, 2]
    return 435837.2 + (0.04762636 - 435837.2) / (1.0 + std.math.pow(f64, s / 240.4549, 2.377168));
}

pub const Filter = struct {
    kind: Kind,
    profile: Profile,
    dpi: u32,
    speed: f64,

    trackers: [MOTION_TRACKER_HISTORY_SIZE]Tracker,
    cur_tracker: usize,
    last_velocity: f64, // units/us

    // Adaptive/linear (mouse) parameters, recomputed from `speed`.
    threshold: f64, // 1000dpi units/us
    accel: f64, // unitless max factor
    incline: f64,

    // Flat parameter and trackpoint parameters, recomputed from `speed`.
    flat_factor: f64,
    tp_speed_factor: f64,
    multiplier: f64,

    // Trackpoint uses a delta smoothener, mouse does not.
    use_smoothener: bool,

    pub fn init(kind: Kind, dpi: u32) Filter {
        var self = Filter{
            .kind = kind,
            .profile = .adaptive,
            .dpi = dpi,
            .speed = 0.0,
            .trackers = [_]Tracker{.{}} ** MOTION_TRACKER_HISTORY_SIZE,
            .cur_tracker = 0,
            .last_velocity = 0.0,
            .threshold = DEFAULT_THRESHOLD,
            .accel = DEFAULT_ACCELERATION,
            .incline = DEFAULT_INCLINE,
            .flat_factor = 1.0,
            .tp_speed_factor = 1.0,
            .multiplier = DEFAULT_TRACKPOINT_MULTIPLIER,
            .use_smoothener = (kind == .trackpoint),
        };
        self.recomputeParams();
        return self;
    }

    /// Recompute every speed-derived parameter so the active profile/kind always
    /// has the right constants regardless of setSpeed/setProfile ordering.
    fn recomputeParams(self: *Filter) void {
        // accelerator_set_speed (filter-mouse.c)
        var threshold = DEFAULT_THRESHOLD - (0.25 / 1000.0) * self.speed;
        if (threshold < MINIMUM_THRESHOLD) threshold = MINIMUM_THRESHOLD;
        self.threshold = threshold;
        self.accel = DEFAULT_ACCELERATION + self.speed * 1.5;
        self.incline = DEFAULT_INCLINE + self.speed * 0.75;

        // accelerator_set_speed_flat (filter-flat.c)
        self.flat_factor = @max(0.005, 1.0 + self.speed);

        // trackpoint_accelerator_set_speed (filter-trackpoint.c)
        self.tp_speed_factor = trackpointSpeedFactor(self.speed);
    }

    /// Set the user speed setting, clamped to [-1, 1], and recompute parameters.
    pub fn setSpeed(self: *Filter, speed: f64) void {
        self.speed = std.math.clamp(speed, -1.0, 1.0);
        self.recomputeParams();
    }

    pub fn setProfile(self: *Filter, p: Profile) void {
        self.profile = p;
    }

    /// Clear the motion history (SYN_DROPPED / device idle). Ports the effect of
    /// `trackers_reset`/`trackers_init`: history emptied, velocity forgotten.
    pub fn reset(self: *Filter) void {
        self.trackers = [_]Tracker{.{}} ** MOTION_TRACKER_HISTORY_SIZE;
        self.cur_tracker = 0;
        self.last_velocity = 0.0;
    }

    fn trackerByOffset(self: *Filter, offset: usize) *Tracker {
        const n = MOTION_TRACKER_HISTORY_SIZE;
        const index = (self.cur_tracker + n - offset) % n;
        return &self.trackers[index];
    }

    /// Port of `trackers_feed`: add the new delta to every tracker, then advance
    /// the ring and reset the new current tracker to zero at `time_us`.
    fn feedTrackers(self: *Filter, dx: f64, dy: f64, time_us: u64) void {
        for (&self.trackers) |*t| {
            t.dx += dx;
            t.dy += dy;
        }
        self.cur_tracker = (self.cur_tracker + 1) % MOTION_TRACKER_HISTORY_SIZE;
        self.trackers[self.cur_tracker] = .{
            .dx = 0.0,
            .dy = 0.0,
            .time_us = time_us,
            .dir = getDirection(dx, dy),
        };
    }

    /// Port of `calculate_trackers_velocity`: distance / time (units/us), with a
    /// +1us guard and optional smoothener clamp for jittery timestamps.
    fn trackerVelocity(self: *Filter, t: *const Tracker, time_us: u64) f64 {
        var tdelta = time_us - t.time_us;
        tdelta += 1;
        if (self.use_smoothener and tdelta < TRACKPOINT_SMOOTHENER_THRESHOLD_US) {
            tdelta = TRACKPOINT_SMOOTHENER_VALUE_US;
        }
        return std.math.hypot(t.dx, t.dy) / @as(f64, @floatFromInt(tdelta));
    }

    /// Port of `trackers_velocity_after_timeout`.
    fn velocityAfterTimeout(self: *Filter, t: *const Tracker) f64 {
        return self.trackerVelocity(t, t.time_us + MOTION_TIMEOUT_US);
    }

    /// Port of `trackers_velocity`: average recent samples that share direction,
    /// aren't stale, and don't differ too much in velocity. Returns units/us.
    fn trackersVelocity(self: *Filter, time_us: u64) f64 {
        var result: f64 = 0.0;
        var initial_velocity: f64 = 0.0;
        var dir: u32 = self.trackerByOffset(0).dir;

        var offset: usize = 1;
        while (offset < MOTION_TRACKER_HISTORY_SIZE) : (offset += 1) {
            const t = self.trackerByOffset(offset);

            // Bug: time running backwards.
            if (t.time_us > time_us) break;

            // Stop if too far away in time.
            const tdelta = time_us - t.time_us;
            if (tdelta > MOTION_TIMEOUT_US) {
                if (offset == 1) result = self.velocityAfterTimeout(t);
                break;
            }

            const velocity = self.trackerVelocity(t, time_us);

            // Stop if direction changed.
            dir &= t.dir;
            if (dir == 0) {
                if (offset == 1) result = velocity;
                break;
            }

            // Average the first two events unconditionally, then reject outliers.
            if (initial_velocity == 0.0 or offset <= 2) {
                result = velocity;
                initial_velocity = velocity;
            } else {
                if (@abs(initial_velocity - velocity) > MAX_VELOCITY_DIFF) break;
                result = velocity;
            }
        }

        return result; // units/us
    }

    /// Port of `pointer_accel_profile_linear` (filter-mouse.c). `speed_in` is a
    /// velocity in normalized 1000dpi units/us.
    fn pointerAccelProfileLinear(self: *Filter, speed_in: f64) f64 {
        const max_accel = self.accel;
        const threshold = self.threshold;
        const incline = self.incline;
        var factor: f64 = undefined;

        if (v_us2ms(speed_in) < 0.07) {
            factor = 10.0 * v_us2ms(speed_in) + 0.3;
        } else if (speed_in < threshold) {
            factor = 1.0;
        } else {
            factor = incline * v_us2ms(speed_in - threshold) + 1.0;
        }

        return @min(max_accel, factor);
    }

    /// Port of `trackpoint_accel_profile` (filter-trackpoint.c). `velocity` in units/us.
    fn trackpointAccelProfile(self: *Filter, velocity: f64) f64 {
        const v = v_us2ms(velocity); // units/ms
        var factor = 10.06254 + (0.3 - 10.06254) / (1.0 + std.math.pow(f64, v / 0.9205459, 1.15363));
        factor *= self.tp_speed_factor;
        return factor;
    }

    /// Port of `calculate_acceleration_simpsons`: Simpson's-rule average of the
    /// linear profile over [last_velocity, velocity].
    fn calculateAccelerationSimpsons(self: *Filter, velocity: f64, last_velocity: f64) f64 {
        var factor = self.pointerAccelProfileLinear(velocity);
        factor += self.pointerAccelProfileLinear(last_velocity);
        factor += 4.0 * self.pointerAccelProfileLinear((last_velocity + velocity) / 2.0);
        return factor / 6.0;
    }

    /// Accelerate one delta sample. `time_ms` is the event timestamp, converted
    /// to microseconds internally to match the ported math.
    pub fn filter(self: *Filter, dx: f64, dy: f64, time_ms: u64) struct { dx: f64, dy: f64 } {
        const time_us = time_ms * 1000;

        switch (self.profile) {
            .flat => {
                // accelerator_filter_flat: constant factor on the device delta,
                // no DPI normalization. Trackpoints scale by their multiplier.
                const base_x = if (self.kind == .trackpoint) dx * self.multiplier else dx;
                const base_y = if (self.kind == .trackpoint) dy * self.multiplier else dy;
                return .{ .dx = base_x * self.flat_factor, .dy = base_y * self.flat_factor };
            },
            .adaptive => switch (self.kind) {
                .mouse => {
                    // normalize_for_dpi, then accelerator_filter_linear.
                    const scale = DEFAULT_MOUSE_DPI / @as(f64, @floatFromInt(self.dpi));
                    const nx = dx * scale;
                    const ny = dy * scale;
                    self.feedTrackers(nx, ny, time_us);
                    const velocity = self.trackersVelocity(time_us);
                    const accel_factor = self.calculateAccelerationSimpsons(velocity, self.last_velocity);
                    self.last_velocity = velocity;
                    return .{ .dx = nx * accel_factor, .dy = ny * accel_factor };
                },
                .trackpoint => {
                    // trackpoint_accelerator_filter.
                    const mx = dx * self.multiplier;
                    const my = dy * self.multiplier;
                    self.feedTrackers(mx, my, time_us);
                    const velocity = self.trackersVelocity(time_us);
                    const f = self.trackpointAccelProfile(velocity);
                    return .{ .dx = mx * f, .dy = my * f };
                },
            },
        }
    }
};

test "golden: hand-computed adaptive output for a single fast sample" {
    // Fresh mouse, adaptive, speed 0, dpi 1000. First sample: dx=100 at t=1ms.
    // velocity = 100/1001 units/us (>threshold), last_velocity = 0.
    // Simpson's rule over the linear profile:
    //   prof(velocity)      -> capped at accel = 2.0
    //   prof(0)             -> 10*0 + 0.3      = 0.3
    //   prof(velocity/2)    -> capped at accel = 2.0
    //   factor = (2.0 + 0.3 + 4*2.0) / 6 = 10.3/6
    // output.dx = 100 * 10.3/6 = 1030/6.
    var g = Filter.init(.mouse, 1000);
    g.setProfile(.adaptive);
    g.setSpeed(0.0);
    const out = g.filter(100.0, 0.0, 1);
    try std.testing.expectApproxEqAbs(out.dx, 1030.0 / 6.0, 1e-9);
    try std.testing.expectApproxEqAbs(out.dy, 0.0, 1e-12);
}

test "flat profile scales unaccelerated delta by a constant" {
    var f = Filter.init(.mouse, 1000);
    f.setProfile(.flat);
    f.setSpeed(0.0);
    const out = f.filter(10.0, 0.0, 1);
    const out2 = f.filter(1.0, 0.0, 100);
    try std.testing.expectApproxEqAbs(out.dx / 10.0, out2.dx / 1.0, 1e-9);
}

test "adaptive profile accelerates faster motion more than slow motion" {
    var slow = Filter.init(.mouse, 1000);
    slow.setProfile(.adaptive);
    slow.setSpeed(0.0);
    var slow_gain: f64 = 0;
    var t: u64 = 0;
    while (t < 50) : (t += 10) slow_gain = slow.filter(1.0, 0.0, t).dx / 1.0;

    var fast = Filter.init(.mouse, 1000);
    fast.setProfile(.adaptive);
    fast.setSpeed(0.0);
    var fast_gain: f64 = 0;
    t = 0;
    while (t < 50) : (t += 10) fast_gain = fast.filter(30.0, 0.0, t).dx / 30.0;

    try std.testing.expect(fast_gain > slow_gain);
}

test "higher DPI is normalized to feel the same as 1000dpi" {
    var lo = Filter.init(.mouse, 1000);
    var hi = Filter.init(.mouse, 2000);
    lo.setProfile(.adaptive);
    hi.setProfile(.adaptive);
    lo.setSpeed(0.0);
    hi.setSpeed(0.0);
    var lo_out: f64 = 0;
    var hi_out: f64 = 0;
    var t: u64 = 0;
    while (t < 50) : (t += 10) {
        lo_out = lo.filter(10.0, 0.0, t).dx;
        hi_out = hi.filter(20.0, 0.0, t).dx;
    }
    try std.testing.expectApproxEqRel(lo_out, hi_out, 0.1);
}
