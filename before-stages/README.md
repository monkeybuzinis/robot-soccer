# RobotPU Soccer — Final Project

Single integrated MakeCode TypeScript program: **[robotpu-soccer-final.ts](robotpu-soccer-final.ts)**.

This file replaces the separate snippets (`robotpu-soccer-mvp.ts`, `robotpu-kalman-filter.ts`,
`robotpu-localmap.ts`, `robotpu-search-soccer.js`, `robotpu-i2c-cam.ts`, `robotpu-A-star.ts`,
`robotpu-viewpoint.ts`) with one self-contained pipeline. **Use only this one `.ts` file in the
MakeCode project** — the others declare the same global names and will collide with it.

Behavior implemented end-to-end: identify ball & goal → **wait until both are confirmed** →
navigate to a kick position behind the ball → orient toward the goal → contact/push the ball →
score, repeating every cycle so missed attempts are retried automatically (see
[robotpu_soccer.pdf](robotpu_soccer.pdf), Chapters 2–7).

For isolated detection testing (no walking/kicking at all), see
**[robotpu-detect-ball-goal.ts](robotpu-detect-ball-goal.ts)** — a standalone Step-1 file that
just prints every I2C packet and head-tracks the ball, so camera/I2C problems can be diagnosed
without the rest of the pipeline in the way.

## Strategy: detect both, then plan — never chase blind

The robot does **not** walk toward the ball just because the ball alone is visible. Approaching
the ball without knowing where the goal is can leave the robot on the wrong side of it to ever
kick it goal-ward. `btRoot()`'s fallback order is:
1. `btCelebrate()` — ball already judged to be in the goal (`condScored()`); overrides every other branch once latched, stops all movement.
2. `btScore()` — needs **both** `ball_valid` and `goal_valid`; only this branch ever plans a path and kicks.
3. `btHoldForGoal()` — ball seen, goal not yet: body holds still (`actionHoldForGoal()`), head keeps tracking the ball.
4. `actionSearchBallBT()` — nothing seen: head sweeps to reacquire.

`walkSpeed`/`walkTurn` are only ever set to nonzero values inside branch 2.

## How each grading-rubric item is implemented

| Rubric item | Where in `robotpu-soccer-final.ts` |
|---|---|
| Detect/follow ball & goal | `trackBall()` (line 619) — parses the ESP32 I2C packet, drives head yaw/pitch to keep the ball centered, stores raw detections. |
| Navigate to kick position | `computeKickPoint()` (552), `actionApproachKickPoint()` (789), `btNavigateToKick()` (913) |
| Contact ball with goal orientation | `actionAlignToGoal()` (809), `condAlignedToGoal()` (766) — rotates in place until heading to the goal is within `ALIGN_HEADING_TOL` (~14°) before kicking. |
| Push ball toward goal | `actionKick()` (825) → sets `walkMode = 2`, executed in the actuator loop (1084) via `robotPuPro.kick()`. The Behavior Tree re-ticks every cycle, so if the ball isn't yet scored it re-approaches and kicks again. |
| Score detection | `condScored()` (775) + `btCelebrate()` (922) — see "Score detection" section below; this is a position heuristic, not a real sensor event. |
| Obstacle avoidance (alpha) | `condPathClear()` (748) + `actionReplan()` (875) treat the ball/goal cells as occupied and re-route via `astarGrid()` (232) when the direct line is blocked. **Limitation:** no sonar packet format was available in the source files, so only ball/goal cells are marked as obstacles — see "Known limitations" below. |

## Mapping

- `LocalGrid` (class, line 178): a 10×10, 0.05 m/cell occupancy grid **anchored at the robot**
  (robot is always cell-relative `(0,0)`, current frame). `index()`/`center()` convert between
  metric local-frame coordinates and grid cells.
- The main loop (line 1009) clears the grid every cycle and marks the ball cell (`2`), goal cell
  (`3`), and computed kick cell (`4`) for debugging and for `condPathClear()` to query.
- A separate disposable `LocalGrid` is built inside `actionReplan()` (875) with the ball/goal
  cells marked as obstacles (`1`), which is what `astarGrid()` actually plans against.

## Localization (per-detection, not global)

There is no global localization — robot pose lives only in the **odometry frame** produced by
`robotPuPro.locationArray()` (read in `getPoseO()`, line 521), used purely to relate two points in
time, not to fix an absolute position. This matches Chapter 3's design choice: not enough fixed
landmarks for SLAM-style global localization, so everything is planned in a frame anchored to
"now."

- **Latency compensation**: `camToNow()` (541) takes a detection captured at time `t_cam`
  (`ball_cam2D`/`goal_cam2D`, stored together with the odometry pose at receipt time in
  `trackBall()`), and re-expresses it in the *current* local frame using the odometry delta
  between `t_cam` and `now`.
- **Smoothing**: `Kalman1DConstVel`/`Kalman2DConstVel` (lines 311, 376) run a constant-velocity
  Kalman filter per axis on the latency-compensated ball/goal positions (`ballKF`, `goalKF`,
  predict+update in the main loop, 1009). The goal uses tiny process noise (it's stationary); the
  ball switches to larger process noise for `BALL_POSTKICK_MS` after a kick so the filter can
  track the ball rolling away instead of lagging behind it.

## Navigation

- **Kick-point geometry**: `computeKickPoint()` (552) places the target `KICK_BACKOFF_M` behind
  the ball, on the goal→ball line, so walking into that point and pushing forward sends the ball
  toward the goal rather than sideways.
- **Path planning**: `astarGrid()` (232) is a 4-connected A* over the `LocalGrid`, replanned
  every cycle from the robot's current cell `(0,0)` to the kick-point cell. Only the first
  waypoint of the returned path is ever executed (`actionReplan()`, 875), consistent with
  "re-plan every cycle" rather than committing to a stale long path.
- **Direct-vs-replan choice**: `btNavigateToKick()` (913) is a Behavior-Tree Fallback: it first
  tries walking straight at the kick point (`condPathClear` + `actionApproachKickPoint`); only if
  that line is blocked does it fall back to the A* replan.

## Control

- **Pose-to-pose controller**: `updateControl()` (571) is a heading-aware "virtual target"
  controller — it aims slightly ahead of and to the side of the literal target so the robot's gait
  naturally arcs into the desired final heading instead of stopping to turn in place, then
  produces `[walkSpeed, walkTurn]` for `robotPuPro.walk()`.
- **Alignment**: once at the kick point, `actionAlignToGoal()` (809) switches `walkMode = 1`
  (back-up-and-turn) and rotates until the heading error to the goal is below
  `ALIGN_HEADING_TOL`, instead of using `updateControl()` (avoids drifting into the ball while
  turning).
- **Actuation**: the actuator loop (1084) is a thin `basic.forever` that maps `walkMode` to
  `robotPuPro.walk(walkSpeed, walkTurn)` / `robotPuPro.walk(-walkSpeed, walkTurn)` /
  `robotPuPro.kick()`, and stamps `lastKickMs` so the Kalman ball filter knows to expect faster
  motion right after a kick.
- **Decision layer**: `btSequence()`/`btFallback()` (717, 725) implement the two Behavior-Tree
  composites from Chapter 7. `btRoot()` (947) is ticked once per planning cycle and is the *only*
  place that decides between celebrating, scoring, holding for the goal, or head-scanning search —
  there is no other if/else planner left in the file (see "Strategy" above for the fallback order).

## Score detection

There's no "ball entered the goal" event in the I2C packet format, so scoring is inferred from
position: `condScored()` (line 775) compares the filtered ball/goal positions and latches `scored
= true` once their distance drops below `SCORE_DIST_M` (0.12m — tuned a bit under the goal's
~0.297m mouth width so the ball merely passing nearby doesn't false-positive). Once latched it
never resets in this demo. `btCelebrate()` (922) is checked **first** in `btRoot()`'s fallback
chain, so once scored it overrides scoring/holding/searching and the robot just stops — there's
nothing left to do. `actionCelebrate()` (836) announces `robotPuPro.talk("Goal!")` exactly once
(guarded by `celebrated`, separate from the `scored` latch) so it doesn't repeat every 20ms tick.

**Caveat**: this is a position heuristic, not a verified "ball crossed the goal line" check — a
ball that rolls close to the goal without actually going in could trigger a false score. Tighten
`SCORE_DIST_M` if you see that in testing.

## Known limitations (carried over from the PDF, Chapter 6 §7)

- No sonar packet format was provided in the source files, so true obstacle avoidance isn't
  wired up; the only "obstacle" the A* planner currently knows about is the ball/goal cells
  themselves (intentionally simplistic, matches the PDF's stated alpha limitation).
- `robotPuPro.locationArray()` actually returns `[x_mm, y_mm, theta_deg]` (millimeters +
  degrees, per the real [pxt-robotpu](https://github.com/robotgyms/pxt-robotpu) source, not
  meters/radians as the original snippet files assumed). `getPoseO()` (line 533) converts both
  before anything else in the pipeline touches the pose. The *sign* of `theta_deg` (which way is
  "turning left") is still field-tunable via `TURN_GAIN`/`kTurn` — verify on the actual robot
  (PDF Chapter 6 §8 testing checklist) and flip those signs if the robot turns the wrong way.
- "Score" is a position heuristic, not a real sensor event — see the "Score detection" section
  above for the caveat about false positives from a near-miss ball.
- Head yaw/pitch (`currentYaw`/`currentPitch`) are **absolute servo angles in degrees**, clamped
  to `HEAD_YAW_MIN/MAX` and `HEAD_PITCH_MIN/MAX` (90° ± 45°, since `pxt-robotpu`'s
  `PCB.servoStep()` clamps to an absolute `[0,179]` range with 90° = looking straight ahead —
  confirmed against the real extension source). An earlier version of this file clamped them to
  `[-45, 45]` as if 0° were neutral, which pinned the head near one physical extreme regardless
  of what the tracking math computed (symptom: head stuck looking the same direction no matter
  what). If the head still drifts to one side/extreme, double check `HEAD_YAW_CENTER`/
  `HEAD_PITCH_CENTER` against your robot's actual mechanical neutral.
- Ball detection failing while goal detection works is **not a microbit-code issue** — it means
  the ESP32-S3's ball color/size thresholds aren't tuned for your ball/lighting. Connect to the
  camera's Wi-Fi AP (`RobotPU-*****`, password `robot1234`), open `https://192.168.4.1`, and
  adjust the soccer ball RGB color and diameter detection parameters there until the bounding box
  shows up around the real ball (PDF Chapter 1 §2.1).

## Tuning knobs

All in the constants block at the top of `robotpu-soccer-final.ts`: grid resolution (`GRID_N`,
`GRID_RES_M`), kick geometry (`KICK_BACKOFF_M`, `KICK_DIST_M`, `ALIGN_HEADING_TOL`), turn gain
(`TURN_GAIN`, flip sign if the robot turns the wrong way), and the Kalman noise terms
(`GOAL_Q_*`, `BALL_Q_*_PRE/POST`, `*_R_FRESH/STALE`) — see PDF Chapter 5 §9 for tuning intuition.
