// RobotPU odometry reference implementation (micro:bit-friendly JavaScript).
//
// This file is a direct port of the Python/Numpy version (SE(2) homogeneous transforms),
// but implemented using plain JavaScript arrays (no numpy dependency).
//
// NOTE: This file is compiled in a MakeCode-like environment where all .ts files share
// one global namespace. To avoid "function re-define" problems, helpers are kept as
// static methods on Odometry (instead of top-level functions).

class Odometry {
    public axisHalfDistanceMm: number;
    public currentTransformation: number[][];

    static rot2(thetaRad: number): number[][] {
        let c = Math.cos(thetaRad);
        let s = Math.sin(thetaRad);
        return [
            [c, -s],
            [s, c],
        ];
    }

    static se2(R: number[][], t: number[]): number[][] {
        return [
            [R[0][0], R[0][1], t[0]],
            [R[1][0], R[1][1], t[1]],
            [0, 0, 1],
        ];
    }

    static trans2(tx: number, ty: number): number[][] {
        return [
            [1, 0, tx],
            [0, 1, ty],
            [0, 0, 1],
        ];
    }

    static matMul3(A: number[][], B: number[][]): number[][] {
        return [
            [
                A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0],
                A[0][0] * B[0][1] + A[0][1] * B[1][1] + A[0][2] * B[2][1],
                A[0][0] * B[0][2] + A[0][1] * B[1][2] + A[0][2] * B[2][2],
            ],
            [
                A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0],
                A[1][0] * B[0][1] + A[1][1] * B[1][1] + A[1][2] * B[2][1],
                A[1][0] * B[0][2] + A[1][1] * B[1][2] + A[1][2] * B[2][2],
            ],
            [
                A[2][0] * B[0][0] + A[2][1] * B[1][0] + A[2][2] * B[2][0],
                A[2][0] * B[0][1] + A[2][1] * B[1][1] + A[2][2] * B[2][1],
                A[2][0] * B[0][2] + A[2][1] * B[1][2] + A[2][2] * B[2][2],
            ],
        ];
    }

    static rotateAboutPivot(deltaYawRad: number, pivotXYmm: number[]): number[][] {
        let px = pivotXYmm[0];
        let py = pivotXYmm[1];
        return Odometry.matMul3(
            Odometry.matMul3(
                Odometry.trans2(px, py),
                Odometry.se2(Odometry.rot2(deltaYawRad), [0, 0])
            ),
            Odometry.trans2(-px, -py)
        );
    }

    static updateOdometry(TworldRobot: number[][], stepTransformationMatrix: number[][]): number[][] {
        return Odometry.matMul3(TworldRobot, stepTransformationMatrix);
    }

    static deg2rad(deg: number): number {
        return (deg * Math.PI) / 180.0;
    }

    static rad2deg(rad: number): number {
        return (rad * 180.0) / Math.PI;
    }

    static identity3(): number[][] {
        return [
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
        ];
    }

    constructor(axisHalfDistanceMm: number = 25.0) {
        this.axisHalfDistanceMm = axisHalfDistanceMm;
        this.currentTransformation = Odometry.identity3();
    }

    update(transformationMatrix: number[][]): void {
        // Apply a general SE(2) step transform (e.g., external correction).
        this.currentTransformation = Odometry.updateOdometry(this.currentTransformation, transformationMatrix);
    }

    leftStep(yawAngleDeg: number): void {
        // Apply one walking step where the left leg is the support pivot.
        this.update(Odometry.rotateAboutPivot(Odometry.deg2rad(yawAngleDeg), [-this.axisHalfDistanceMm, 0.0]));
    }

    rightStep(yawAngleDeg: number): void {
        // Apply one walking step where the right leg is the support pivot.
        this.update(Odometry.rotateAboutPivot(Odometry.deg2rad(yawAngleDeg), [this.axisHalfDistanceMm, 0.0]));
    }

    getPosition(): { x_mm: number; y_mm: number; theta_deg: number } {
        // Return (x,y,theta) extracted from the SE(2) matrix.
        let xMm = this.currentTransformation[0][2];
        let yMm = this.currentTransformation[1][2];
        let thetaDeg = Odometry.rad2deg(Math.atan2(this.currentTransformation[1][0], this.currentTransformation[0][0]));
        return { x_mm: xMm, y_mm: yMm, theta_deg: thetaDeg };
    }

    reset(): void {
        this.currentTransformation = Odometry.identity3();
    }
}

