// Shared utilities for RobotPU micro:bit TypeScript code.
//
// IMPORTANT (MakeCode): all .ts files share one global scope.
// Keep helpers in a namespace to avoid collisions and to reduce duplicated code.

namespace robotpuUtils {
    export function clampInt(v: number, lo: number, hi: number): number {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    export function clampFloat(v: number, lo: number, hi: number): number {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    export function deg2rad(deg: number): number {
        return (deg * Math.PI) / 180.0;
    }

    export function rad2deg(rad: number): number {
        return (rad * 180.0) / Math.PI;
    }

    // Standard 1st-order low-pass:
    // - alpha close to 0 => heavy smoothing
    // - alpha close to 1 => follow measurements quickly
    export function lowPass(prev: number, meas: number, alpha: number): number {
        return (1 - alpha) * prev + alpha * meas;
    }

    export function lowPass2D(prevX: number, prevY: number, measX: number, measY: number, alpha: number): number[] {
        return [
            lowPass(prevX, measX, alpha),
            lowPass(prevY, measY, alpha),
        ];
    }

    // Wrap an angle to [-pi, +pi] for stable heading error computation.
    export function wrapToPi(rad: number): number {
        while (rad > Math.PI) rad -= 2 * Math.PI;
        while (rad < -Math.PI) rad += 2 * Math.PI;
        return rad;
    }
}
