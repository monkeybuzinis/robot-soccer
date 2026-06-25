# stages-code/ — Bug Log

This directory holds the staged MVP test files (per `plan.txt`) used to bring up
the soccer robot incrementally: Stage 1 (I2C/utils) → Stage 2 (odometry +
search) → Stage 3 (approach + hold at kick point) → Stage 4 (kick) → Stage 5
(A*/pure-pursuit/Behavior Tree).

Each stage file is built from `professor-code/`'s reference files, ported to
match the API actually exported by the installed `pxt-robotpu` extension, then
iteratively fixed against real hardware logs. This file records every bug
found and how it was fixed, in the order discovered, so the reasoning isn't
lost once the code itself moves on.

## Global fixes — needed on every professor-code/ file

`professor-code/` targets an older/renamed build of the extension. Every file
ported from it needs these same fixes before it compiles or runs correctly:

1. **Namespace**: `robotPu` → `robotPuPro`. The installed extension exports
   `robotPuPro`; `robotPu` doesn't exist.
2. **Casing**: `robotPu.ServoTargets()` → `robotPuPro.servoTargets()`
   (lowercase `s`).
3. **Head servo range**: professor-code clamps head yaw/pitch targets to
   `[-45, 45]` as an offset from 0. The real `robotPuPro.servoStep()` takes an
   **absolute** `[0, 179]` target with `90` = neutral — a `[-45, 45]` target
   would clamp the head hard against one physical limit.
4. **Array API**: MakeCode's array type has no `.splice()` — use
   `.removeAt()` instead (only matters once Stage 5's A* file is ported).

## Stage 2 (`stage2.ts`) — Odometry & Active Search

| # | Bug | Symptom | Fix |
|---|-----|---------|-----|
| 1 | `gst`/Mode race | `robotPuPro.locationArray()` stayed frozen at `(0,0,0)` even while manually driving the robot with the remote. | Other loops call `servoStep()`/`setModeVar()` every 20ms, which forces `gst = Mode.API` on (effectively) every tick. The extension's internal `stateMachine()` has no dispatch entry for API mode, so a remote-set `gst = 5` (joystick) never survives long enough for `joystick()`'s `walk()` call to run. Fixed by calling `robotPuPro.walk()` directly from script code (the same call path `joystick()` uses internally) — this runs under whatever mode is currently forced, so it always executes and updates odometry. |
| 2 | Lost-timer bug | A ball that's physically gone but still being reported as a stale last-known position never timed out, so the head never started searching. | professor-code's `trackBall()`-equivalent logic refreshed the "last seen" timer on any packet with `count > 0`, including `STALE` ones. Fixed: only a fresh, non-stale packet resets the miss counter; stale/missing/wrong-type packets all count as a miss, and after `BALL_STALE_CYCLE_LIMIT` consecutive misses the ball is declared lost regardless of how many more stale repeats keep arriving. |
| 3 | Head-vibration bug | `BALL_TRACK`/`SEARCHING` flipped every other cycle, vibrating the head, even while the ball was still valid. | The search branch fired any time the current I2C read returned a non-ball packet type (e.g. a goal packet interleaved on the bus), not just when the ball was actually timed out. Fixed by gating the search call on `!ballValid` explicitly, so a single interleaved goal-type packet can no longer flip the state. |

## Stage 3 (`stage3.ts`) — Approach the Ball & Take Up the Kick Position

Built from `professor-code/robotpu-soccer-mvp.ts` + `robotpu-localmap.ts` +
`robotpu-kalman-filter.ts` + `robotpu-viewpoint.ts`. These files have real
bugs/mismatches against the actual installed extension and against each
other, found and fixed in this order:

| # | Bug | Symptom | Fix |
|---|-----|---------|-----|
| 1 | Degrees vs radians | professor-code's `camToNow()` fed `theta` straight from `locationArray()[2]` into `Math.cos`/`Math.sin`, which expect radians — but `locationArray()` returns `theta_deg` in **degrees** (confirmed on hardware: e.g. `theta_deg=106.51`). | Convert to radians (`deg2rad()`) before every trig call. |
| 2 | Unit mismatch | professor-code divides camera `x_mm`/`y_mm` by 1000 to get meters, then mixes that with `locationArray()`'s raw-millimeter pose in the same add/subtract — a 1000x scale bug. | This file works in millimeters end-to-end (matching `locationArray()`'s native unit); Kalman Q/R constants are professor-code's meter-tuned values × 1e6 (variance scales with the square of the unit). |
| 3 | Lost-timer bug (same class as Stage 2 #2) | A ball/goal reported as stale-last-known never timed out. | Lost timer and the stored `cam2D`/pose used for the frame transform only reset on fresh (non-stale) detections. |
| 4 | Head-vibration bug (same class as Stage 2 #3) | Both ball and goal detection run continuously here, so this mattered more than in Stage 2. | Search is only triggered from inside the ball branch once the ball is actually timed out, never just because the current I2C read happened to return a goal-type packet. |
| 5 | Eye LED always on | "The LED light keeps being on, we don't need it" — not caused by this file's own code. | Root-caused to `pxt-robotpu`'s `RobotPu.pcb` class, which defaults `eyeIsOn = true` and runs a background blink animation. Fixed with one call at boot: `robotPuPro.setEyeBrightness(0)`. |
| 6 | Ceiling/wall noise read as goal | While the head was pitched up, the camera misread ceiling lights/walls as the goal (e.g. `x_mm=1701 y_mm=4580`, far outside any real field). | Added `MAX_DETECT_DIST_MM = 2500` plausibility filter on raw camera distance for both ball and goal; implausible detections are rejected and logged (`GOAL_REJECTED ... implausible -- likely ceiling/wall noise`). |
| 7 | Walked on ball-only detection | Robot chased the ball alone with "no defined path" before the goal was even visible. | Body now only ever moves once **both** ball and goal are valid and a real kick point can be computed; head tracking/searching still runs independently of this gate. |
| 8 | Head-tracking lost the ball while walking | Robot looked up and lost sight of the ball mid-approach. | The original tracking gain/sign/structure here still lost the ball on hardware. Replaced wholesale with the gain/sign/decay-on-dropout logic from `robotpu-followball.ts` (confirmed working on hardware): gain `0.2` (not `0.08`), no stale-distance scaling, decay-based follow-through on brief dropout, and search only triggered by a real lost-timeout (decoupled from packet type, so it can't be re-triggered by an interleaved goal packet). |
| 9 | Kalman filter frame bug | Robot walked toward the ball/goal but kept oscillating, and `walkMode` never left `0` — it never reached/held the kick pose even with both ball and goal continuously valid. | professor-code's Kalman filters smoothed positions **after** transforming them into the robot's *current* frame (`{C_now}`), which shifts every cycle as the robot walks. That conflates the robot's own motion with the tracked object's velocity — a perfectly stationary ball looks like it has nonzero velocity every time the robot itself moves, since the relative coordinates shift even though the ball didn't. Fixed by filtering in the fixed **odometry/world frame** (`camToOdom()`), where a stationary object genuinely has ~0 velocity, then re-projecting the filtered estimate into `{C_now}` fresh every cycle with the live pose (`odomToNow()`) — correct even on cycles with no new detection. |
| 10 | Ground-plane distance unreliable near the robot | After fix #9, the robot approached correctly but then walked into/past the ball instead of stopping — `distKick` stayed stuck well above `KICK_DIST_MM` even as head pitch pinned at its downward limit (135°), i.e. the ball was right at the robot's feet. | A single downward-pitched camera's ground-plane distance projection is only reliable far from the robot; near the camera's blind spot, small pitch errors blow up into large distance errors. Added a second, physical stop trigger: once head pitch is pinned within `NEAR_BALL_PITCH_DEG` (10°) of its downward limit while tracking the ball, the planner treats that as "ball is right here" and exits approach mode regardless of what the (unreliable, at that range) computed distance says. |

## Still open / worth tuning

- `NEAR_BALL_PITCH_DEG = HEAD_PITCH_MAX - 10` is a first guess — may need
  tuning based on how close the robot actually is when pitch first pins at
  that value on hardware.
- The `LocalGrid` class in `stage3.ts` is debug-visualization only (marks
  robot/ball/goal/kick-point cells) — it is **not** used for path planning.
  Grid-based pathfinding is Stage 5's job (A* + pure pursuit), not pulled
  forward into Stage 3.
- `updateControl()`'s tuning constants (`vMax`, `turnMax`, lead/lateral
  offsets, `TURN_GAIN`) are carried over from `robotpu-viewpoint.ts` and may
  need adjustment for this robot's actual turning radius and walk speed.
