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
| 4 | Search pitch biased toward the ceiling, guessed direction wrong | Active head-scan search pointed up at the ceiling/walls instead of down at the floor. | `HEAD_PITCH_GROUND_BIAS` was an unverified `+15` guess; changed to `-35` (search center below 90) based on a hardware conclusion reached during `stage3.ts` testing at the time. **That conclusion was later reversed** (see `stage3.ts` bug #16) and `stage2.ts` was never revisited — treat this fix as unconfirmed/possibly backwards again; see "Still open" below. |
| 5 | Search crept one degree at a time | Search sweep was visibly very slow on hardware. | `servoStep()` speed in `searchBall()`'s search calls was `1` (slowest), far below the live-tracking calls elsewhere (`8`). Matched to `8`. |

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
| 5 | Eye LED always on (first attempt) | "The LED light keeps being on, we don't need it" — not caused by this file's own code. | Root-caused to `pxt-robotpu`'s `RobotPu.pcb` class, which defaults `eyeIsOn = true` and runs a background blink animation. First fix: `robotPuPro.setEyeBrightness(0)` at boot — **superseded by bug #11 below**, which found this call alone doesn't actually work. |
| 6 | Ceiling/wall noise read as goal | While the head was pitched up, the camera misread ceiling lights/walls as the goal (e.g. `x_mm=1701 y_mm=4580`, far outside any real field). | Added `MAX_DETECT_DIST_MM = 2500` plausibility filter on raw camera distance for both ball and goal; implausible detections are rejected and logged (`GOAL_REJECTED ... implausible -- likely ceiling/wall noise`). |
| 7 | Walked on ball-only detection | Robot chased the ball alone with "no defined path" before the goal was even visible. | Body now only ever moves once **both** ball and goal are valid and a real kick point can be computed; head tracking/searching still runs independently of this gate. |
| 8 | Head-tracking lost the ball while walking | Robot looked up and lost sight of the ball mid-approach. | The original tracking gain/sign/structure here still lost the ball on hardware. Replaced wholesale with the gain/sign/decay-on-dropout logic from `robotpu-followball.ts` (confirmed working on hardware): gain `0.2` (not `0.08`), no stale-distance scaling, decay-based follow-through on brief dropout, and search only triggered by a real lost-timeout (decoupled from packet type, so it can't be re-triggered by an interleaved goal packet). |
| 9 | Kalman filter frame bug | Robot walked toward the ball/goal but kept oscillating, and `walkMode` never left `0` — it never reached/held the kick pose even with both ball and goal continuously valid. | professor-code's Kalman filters smoothed positions **after** transforming them into the robot's *current* frame (`{C_now}`), which shifts every cycle as the robot walks. That conflates the robot's own motion with the tracked object's velocity — a perfectly stationary ball looks like it has nonzero velocity every time the robot itself moves, since the relative coordinates shift even though the ball didn't. Fixed by filtering in the fixed **odometry/world frame** (`camToOdom()`), where a stationary object genuinely has ~0 velocity, then re-projecting the filtered estimate into `{C_now}` fresh every cycle with the live pose (`odomToNow()`) — correct even on cycles with no new detection. |
| 10 | Ground-plane distance unreliable near the robot | After fix #9, the robot approached correctly but then walked into/past the ball instead of stopping — `distKick` stayed stuck well above `KICK_DIST_MM` even as head pitch pinned near its downward limit, i.e. the ball was right at the robot's feet. | A single downward-pitched camera's ground-plane distance projection is only reliable far from the robot; near the camera's blind spot, small pitch errors blow up into large distance errors. Added a second, physical stop trigger: once head pitch is pinned within `NEAR_BALL_PITCH_DEG` (10°) of its downward limit while tracking the ball, the planner treats that as "ball is right here" and exits approach mode regardless of what the (unreliable, at that range) computed distance says. |
| 11 | Eye LED still on after bug #5's fix | LED stayed lit even with `setEyeBrightness(0)` called at boot. | `setEyeBrightness(0)` only sets a brightness *scalar* — the extension's `RobotPu` constructor calls `pcb.eyesCtl(1)` once at boot, a **digital** pin write at full brightness that bypasses the scalar entirely. The only thing that would normally turn it back off is the background `blink()` animation, which never runs here because this script forces `gst = Mode.API` every cycle (`blink()` only fires while `gst` is in `[0,5]`). Fixed by overriding the analog pins directly at boot: `robotPuPro.leftEyeBright(0)` / `robotPuPro.rightEyeBright(0)`. |
| 12 | `KICK_DIST_MM` too small | Robot's computed `distKick` never converged below ~280-300mm even when the ball was visibly at the robot's feet/in contact, so it kept "approaching" indefinitely. | Recalibrated `KICK_DIST_MM` from `110` to `320` based on the hardware-observed convergence floor; `APPROACH_OFFSET_START_MM` raised from its old value to `450` to match (so the arc-in still starts well before the new, larger stop distance). |
| 13 | Yaw search accumulation/saturation | Search yaw ratcheted to its clamp within 1-2 held frames instead of holding a fixed offset from center. | `searchBall()` built its yaw target cumulatively on the *live* yaw (`liveYaw + offset`) every frame the step was held (`scanFrameCounter` keeps a step active for `SCAN_WAIT_FRAMES` cycles), re-adding the same offset on top of an already-shifted position each cycle. Fixed by making the target absolute: `HEAD_YAW_CENTER + offset.y * search_gain`, matching how pitch was already computed. |
| 14 | Search blocked by interleaved goal packets | Search made no progress on some cycles for no apparent reason. | `searchBall()` was only ever called from inside `trackPacket()`'s `SOCCER_BALL`-type branch. Both ball and goal detection run continuously and the camera interleaves packet types, so on any I2C cycle that happened to return a `SOCCER_GOAL` packet, `trackPacket()` never touched ball state at all — search simply didn't run that cycle, regardless of `ball_valid`. Fixed by removing the inline calls and centralizing into the main planner loop as `if (!ball_valid) searchBall()`, called once per cycle independent of which packet type arrived. |
| 15 | Pitch direction convention was backwards | Robot's head visibly searched/tracked **above** level even while computed pitch values were, under the then-assumed convention, supposed to be at or below level. | The earlier "confirmed on hardware: increasing pitch = up" conclusion (used for bug #10 and the original `HEAD_PITCH_OPERATING_MAX` ceiling) was based on ambiguous evidence — a head pinned at one extreme while losing the ball is equally explainable either direction. A fresh hardware log settled it: search pitch confined to 62.5–72.5° (entirely *below* the assumed-level value of 90) still visibly faced above level — only consistent with **lower** pitch being up. Reversed globally: `HEAD_PITCH_MIN` (45) = up extreme, `HEAD_PITCH_CENTER` (90) = level, `HEAD_PITCH_MAX` (135) = down extreme (toward the robot's own feet). The old operating *ceiling* became an operating *floor* (`HEAD_PITCH_OPERATING_MIN = HEAD_PITCH_CENTER`) so pitch can structurally never go above level; `NEAR_BALL_PITCH_DEG`, the search range, and the pin-detection threshold (`PITCH_PIN_UP_MARGIN_DEG`/`PITCH_PIN_UP_CYCLE_LIMIT`) were all flipped to match. |
| 16 | `camToOdom()` never corrected for head yaw | Large, erratic swings in `ball_now`/`goal_now`/`kickPt` (positions computed at 2000+mm on what's supposed to be a ~1m field), worse the more the head was deflected from center. | `camToOdom()` only ever rotated the camera-frame measurement by the robot **body's** heading (`locationArray()[2]`) — it never applied the head's own yaw deflection. The camera's `(x_mm, y_mm)` is reported in the camera's own frame, which differs from the body's forward direction by the head's current yaw; per `robotpu_soccer.pdf` (Ch.3 §1.1/§2.1), treating it directly as body-frame is only valid "when head yaw/pitch stays near zero during walking" — violated here by design, since this script actively swings the head ±45° to track/search. Fixed by snapshotting the head yaw at the moment of each detection (`ball_head_yaw_deg`/`goal_head_yaw_deg`) and rotating the camera-frame offset by `headYawDeg - HEAD_YAW_CENTER` before applying the body's own rotation. **Rotation sign is unverified on hardware** — if positions swing *more* with head deflection after this fix instead of less, flip the sign. |
| 17 | Approach steering turned the wrong way | Robot veered left during approach instead of walking toward the ball, and lost tracking because of it (head couldn't keep up with the body's own unwarranted rotation). | `updateControl()`'s turn gain (`kTurn = -2.0`) had a comment flagging it as unverified ("if robot turns the wrong direction, flip...") since launch — never confirmed on hardware until now. Confirmed backwards; flipped to `kTurn = 2.0`. Only affects `walkMode 0` (approach) — the separate `TURN_GAIN` constant used by `walkMode 1` (post-arrival align-to-goal) is untouched and still unverified. |
| 18 | Robot walked into the goal and kept pushing, never stopping | With no real ball anywhere in the scene, the device log still showed `BALL_TRACK yaw=...` climbing smoothly and continuously (45 → 91° over many cycles) — genuine head-tracking behavior, meaning the camera was confidently, falsely locking onto something else as "the ball" (`VALID`, non-`STALE`, plausible-distance packets — not fake/hardcoded data in this file; re-confirmed the same way as the earlier goal-alone investigation). `walkMode 0` then drove the robot toward a kick point computed from that phantom ball, and since Stage 3 has zero obstacle/contact sensing by design, nothing noticed when it physically hit the goal — it just kept commanding forward walk indefinitely. | Root cause (the camera's false-positive ball detection) is on the ESP32 side and out of this file's reach. Added a stall safety stop instead: track odometry displacement per cycle while `walkMode == 0` and commanded `walkSpeed` is non-trivial; if position hasn't advanced more than `STALL_DIST_EPS_MM` for `STALL_CYCLE_LIMIT` (~1s) consecutive cycles, force `walkSpeed`/`walkTurn` to 0 and drop `ball_valid` so the planner re-searches instead of continuing to push. Doesn't fix *why* a bad target appeared, but stops the robot from grinding against an obstacle regardless of the reason — the underlying camera false-positive is still open, see below. |
| 19 | Robot walked only a few steps then stopped well short of the ball, then long search pauses before walking 2 more steps and stopping again | **Confirmed** by the new `pose x=... y=... theta=...` debug line (hardware log `microbit-console-2026-06-27T05-12-22-106Z.txt`). `poseNow` from `robotPuPro.locationArray()` doesn't drift smoothly while walking — it holds the *exact same* value (e.g. `x=0 y=0 theta=0`) for 15-30+ consecutive planner cycles (0.3-0.6s) while `walkMode==0` and the robot is actively trying to close real distance, then snaps to a new value that isn't a continuation of a trajectory (e.g. `theta=35.4 -> 18.4 -> 35.4 -> -54.6`, a ~90° swing between two samples, while `x`/`y` only ever moved a few mm total). This is worse than the originally-suspected single-frame glitch: every `ball_now`/`goal_now`/`kickPt` computed during a frozen stretch assumes the robot hasn't moved when it's mid-stride, and `camToOdom()` snapshots this same stale pose to fix each detection's position in the ODOM frame — so the Kalman filters' own belief about where the ball/goal actually are gets built from a wrong understanding of where the robot was standing, not just a one-cycle steering error. This explains "still far from ball" better than a single bad frame would. | Root cause now believed to be `robotPuPro.locationArray()` itself not refreshing every cycle while walking (possibly a fixed internal refresh interval, or needing some other call/trigger to integrate continuously) — this lives in the `pxt-robotpu` extension/hardware, not in `stage3.ts`. The `arrivedCycles` debounce and the stall detector (bug #18) both still help as damage control regardless of cause, but neither fixes the underlying stale-pose problem. Needs investigation at the extension level next. The separate "long pause before re-searching" symptom is most likely just the existing `SEARCH_PATTERN`'s full-sweep time (~2.4s at `SCAN_WAIT_FRAMES=12`, 10 steps, 20ms/cycle) — not investigated further this round, see "Still open" below. |

## Still open / worth tuning

- `NEAR_BALL_PITCH_DEG = HEAD_PITCH_MAX - 10` (125°, under the corrected
  pitch convention from bug #15) is a first guess — may need tuning based on
  how close the robot actually is when pitch first pins at that value on
  hardware.
- **Camera reports a confident, smoothly-tracked "ball" with no real ball in
  the scene.** First reported as the goal-alone-detection issue right after
  bug #15's pitch fix landed; re-confirmed more directly afterwards with the
  ball fully removed (`BALL_TRACK yaw=` climbing continuously 45→91° over
  many cycles — real head-tracking, not noise). Checked both times that this
  isn't fake/hardcoded data in this file (`ball_cam2D_mm`/`ball_valid` are
  only ever set from genuine, `VALID`/non-`STALE`/plausible-distance I2C
  packets in `trackPacket()`), so the false positive itself is on the
  ESP32's ball-color detector — out of this file's reach to fix directly.
  Bug #18's stall safety stop limits the *damage* (stops the robot from
  pushing into an obstacle because of it) but doesn't address the detector
  mistaking something else for the ball in the first place.
- **`locationArray()` appears to stall for 15-30+ cycles at a time while
  walking, then jump (bug #19, confirmed) — needs extension/hardware-level
  investigation.** Not fixable from inside `stage3.ts`; the open question is
  whether `robotPuPro.locationArray()` has a refresh-rate limit, needs a
  different call pattern to integrate odometry continuously while walking,
  or there's some other explanation. Until resolved, every position estimate
  this file computes (ball, goal, kick point) is only as fresh as the last
  pose update, which can lag the robot's actual walking progress by over half
  a second.
- **Search re-acquisition feels slow after losing the ball.** Likely just
  `SEARCH_PATTERN`'s full-sweep time (10 steps × `SCAN_WAIT_FRAMES=12` cycles
  × 20ms/cycle ≈ 2.4s for one full sweep, longer if it takes more than one
  sweep to land back on the ball) rather than a bug — not measured directly
  against hardware-observed pause length yet. Worth trying a smaller
  `SCAN_WAIT_FRAMES` or biasing the first search step toward the last known
  bearing before assuming this needs a structural fix.
- `stage2.ts`'s `HEAD_PITCH_GROUND_BIAS` (bug #4) and its surrounding
  comments are based on the pitch-direction conclusion that bug #15 later
  reversed for `stage3.ts`. `stage2.ts` was never revisited after that
  reversal (work this session stayed scoped to `stage3.ts` per direct
  instruction) — its pitch search direction should be re-verified on
  hardware before `stage2.ts` is used again.
- The `LocalGrid` class in `stage3.ts` is debug-visualization only (marks
  robot/ball/goal/kick-point cells) — it is **not** used for path planning.
  Grid-based pathfinding is Stage 5's job (A* + pure pursuit), not pulled
  forward into Stage 3.
- `updateControl()`'s tuning constants (`vMax`, `turnMax`, lead/lateral
  offsets, `TURN_GAIN`) are carried over from `robotpu-viewpoint.ts` and may
  need adjustment for this robot's actual turning radius and walk speed.

## Stage 4 (`stage4.ts`) — Perform the Kick

Identical to `stage3.ts` up through reaching/aligning at the kick pose. The
only behavioral change, per `plan.md`'s Stage 4 description: instead of
`walkMode == 2` holding position, the actuator loop calls `robotPuPro.kick()`
repeatedly until the motion completes, then drops the ball lock so the
planner re-detects and re-approaches (covers a missed kick).

| # | Bug | Symptom | Fix |
|---|-----|---------|-----|
| 1 | Calling `kick()` every cycle in `walkMode 2` would replay the kick motion indefinitely | `walkMode` stays `2` every cycle as long as ball+goal stay valid and aligned — without guarding against it, the actuator loop would call `robotPuPro.kick()` forever once aligned, not just once per approach. | Added `kickActive` (set once `kick()` returns nonzero, i.e. mid-motion) and `kickJustFinished` (set the cycle `kick()` returns `0` *after* having been active) in the actuator loop. The planner loop consumes `kickJustFinished` each cycle and force-drops `ball_valid`, so the planner falls back to searching/re-approaching instead of the actuator loop calling `kick()` again while still aligned. Confirmed working on hardware — `microbit-console-2026-06-27T00-54-33-146Z.txt` shows clean `AT_KICK_POSE` → kick → `KICK_DONE -- dropping ball lock to re-approach` → re-search cycles repeating, with no replayed/repeated kicks. |
| 2 | `KICK_DIST_MM` too large for kicking (inherited from Stage 3's bug #12, which tuned it against where the *approach* should stop, not where the foot can reach the ball) | First hardware kick test: robot reached the kick pose but kicked >10cm behind the ball, with a weak/glancing result. Log shows every `AT_KICK_POSE` firing with `distKick` in the 240-290mm band (e.g. `dist=281.97` at the exact transition), not anywhere near `KICK_BACKOFF_MM`'s intended 50mm. `headPitch` was only ~118.7° at that same moment — below `NEAR_BALL_PITCH_DEG` (125) — so the *distance* condition alone was triggering "arrived," not the pitch-based proximity cue; once `walkMode` leaves 0, neither `walkMode 1` (backs up while aligning) nor `walkMode 2` (holds/kicks) closes any further distance, so the robot kicked from wherever it happened to be the cycle `distKick` first dipped under 320. | Lowered `KICK_DIST_MM` from `320` to `150` in `stage4.ts` only (left `stage3.ts` untouched, since 320 was correct for that file's different goal of just stopping without kicking). `KICK_BACKOFF_MM` itself was already correct (50mm/5cm) and was *not* the cause despite that being the user's first guess. |
| 3 | `nearBallByPitch` fires on a single saturated reading, short-circuiting the approach before `distKick` ever reaches `KICK_DIST_MM` | Second hardware kick test (after bug #2's fix, `KICK_DIST_MM=150`): user reports the kick still connects too far from the ball, wants it within ~1cm. Log shows `distKick` *was* decreasing smoothly in `walkMode 0` the whole time (e.g. 175.46 → 168.22 → 164.36 → 164.52mm, no floor in sight, unlike bug #12's fear) — but every `AT_KICK_POSE` still clustered at `distKick`=175-205mm, all above the 150 threshold. Root cause: the transition at `dist=166.23` fired with `headPitch=135` (`HEAD_PITCH_MAX`, the head's hard mechanical down-limit) — `nearBallByPitch` saturates as soon as the ball gets low enough in frame for the head to hit its physical tilt limit, which happens well before the robot is actually close, and a single such reading was enough to count as "arrived." | Lowered `KICK_DIST_MM` further, `150` → `60` (distance is shown to behave well in this range, not floored). Added `pitchNearCycles`/`NEAR_BALL_PITCH_CYCLE_LIMIT` (25 cycles, ~0.5s) so `nearBallByPitch` must hold for a sustained run, not one frame, before it can short-circuit the approach — a momentary mechanical-limit pin no longer counts. |
| 4 | Kick repeatedly abandoned mid-motion by `walkMode` flicker | `microbit-console-2026-06-27T-stage4-2.txt`: `walkMode` flickered `2→1→2→1` every 2-3 cycles (~40-60ms). The planner recomputes `walkMode` from scratch every cycle purely off `headingGoal`'s instantaneous value with no hysteresis, and `headingGoal` jitters across the 0.25 threshold from ordinary sensor/Kalman noise — so `robotPuPro.kick()`'s multi-step motion never got to hold `walkMode==2` long enough to reach a strike position. Confirmed by zero `KICK_DONE` lines despite 9 separate `AT_KICK_POSE` attempts in that log; matches the user's "kick is small" report (the leg barely starts moving before `walkMode==1` takes back over and calls `walk()` instead). | Made a started kick stick in the actuator loop regardless of what the planner does next cycle: once `kickActive` is set (`kick()` returned nonzero at least once), keep calling `kick()` every cycle until it itself reports done, independent of `walkMode`. |
| 5 | `nearBallByPitch` (bug #3's override) fired far from the ball, with no distance bound at all | `microbit-console-2026-06-27T19-40-59-450Z.txt`: pitch sustained above `NEAR_BALL_PITCH_DEG` (125°) for 30+ consecutive cycles (130-133°) while `distKick` sat at 500-550mm the whole time — `pitchConfirmed` alone declared "arrived" hundreds of mm short of the ball, matching the user's "turned left, missed the ball" report. | Gated the pitch override behind a real distance bound, `NEAR_BALL_PITCH_DIST_GATE_MM = 200`, so it can only shortcut the approach once already plausibly close, not from anywhere on the field. |
| 6 | `updateControl()`'s lead/lateral lookahead offset grew **toward** the target instead of collapsing near it | `microbit-console-2026-06-27T20-24-58-288Z.txt`: `offsetGain` (and the lead/lateral it drives) rose toward its maximum as `dist` shrank toward `0` — backwards from what a clean final approach needs — so the lookahead point used for heading could be up to 100mm to the side and 180mm past a target only ~150mm away. Robot's heading swung ~90° in a single step right as `distKick` bottomed out near 150mm, then `distKick` climbed every cycle afterward and never recovered: turned away, missed the ball, kept walking forward into a wall instead of stopping or correcting. | Added a second, independent `finalApproachTaper` (`FINAL_STRAIGHT_RANGE_MM = 150`) that collapses lead/lateral to 0 as `dist` approaches the stop distance, so the final approach is always a direct line at the target regardless of the long-range curve-in shaping. Also added `walkMode 3` (explicit user request): once `distKick` grows for `MISS_CYCLE_LIMIT` consecutive cycles while still approaching, back the robot away from the missed kick point while re-orienting to face it, then hand back to the normal `walkMode 0` approach for a fresh attempt instead of plowing forward blind. |
| 7 | `KICK_DIST_MM` (60mm) sat below `distKick`'s own measurement floor — "arrived" never fired at all | Two new logs (`2.txt`/`3.txt`): both 100% `walkMode 0` start to finish, `AT_KICK_POSE` never printed once in either. In `2.txt` the user confirms the robot made real physical contact (dribbled the ball into the goal, ball between its legs) while `distKick` logged 100-160mm the *entire* approach — never anywhere near the 60mm threshold. That's a calibration floor, not noise: odometry/camera-projection lag accumulating at close range puts a persistent ~100-150mm gap between computed `distKick` and true contact. | Raised `KICK_DIST_MM` to `150` to match the observed real-contact band. Also lowered `KICK_BACKOFF_MM` from `50` to `10` per explicit request — the kick stance point should sit just 1cm behind the ball, ball always between that point and the goal (`computeKickPoint()`'s direction logic was already correct; only the magnitude needed to shrink). |
| 8 | 150mm still wasn't enough stopping margin | `microbit-console-2026-06-27T21-09-40-430Z.txt` (taken with `KICK_DIST_MM=150`): close approach reads `dist=204.55 → dist=181.83` (one 20ms cycle, never dropping below 150), then the very next reading jumps back **up** to 205.68 and keeps climbing — user confirms the robot touched/pushed the ball through this exact stretch. Two compounding causes: (1) a quadruped gait can't stop mid-stride — a committed step finishes its swing even after `walk(0,0)` is called; (2) `ballKF`'s process noise (`BALL_Q_POS`/`BALL_Q_VEL`) is tuned for a stationary/rolling ball pre-contact, so the filtered `ball_now` lags behind a ball that's already being pushed — `kickPt` kept reporting "still ~200mm away" mid-push, i.e. the robot "didn't recognize it was already at the kick position." | First attempt: raised `KICK_DIST_MM` to `250` for more stopping margin — **superseded by bug #9 below, this was the wrong direction.** |
| 9 | Bug #8's fix (`KICK_DIST_MM=250`) solved the wrong problem | Explicit user correction: the robot's kick has almost no reach — it can only connect with the ball from ~1cm away. A 250mm "arrived" radius means the robot stops approaching and starts the kick sequence while still ~25cm short of the ball; no amount of approach-stopping margin fixes that, since the kick itself can't cover that gap — it just whiffs at thin air. | Reverted `KICK_DIST_MM` to a tight value matching the kick's real reach (`30`, later tightened further by the user to `10`). Since this reopens bug #7/#8's problem (`distKick`'s own ~100-200mm floor/lag may never satisfy a tight threshold even at true contact), added `contactDetected`: a second, independent "arrived" signal based on physical resistance — commanded forward motion with ~zero real displacement over a short ~120ms window (`CONTACT_CYCLE_LIMIT = 6`) — gated to only count once already plausibly close (`CONTACT_GATE_DIST_MM = 400`) so it reads as "just hit the ball" rather than "stuck/lost" (still `STALL_DETECTED`'s job, on its own ~1s fuse, for the cases this gate excludes, e.g. genuinely walking into a wall from across the field). |
| 10 | Bug #6's miss-recovery (`walkMode 3`) never fired despite a real, sustained miss | Same log as bug #8: `distKick` climbed for a long stretch overall but wasn't strictly monotonic every single 20ms cycle — occasional non-growing cycles (jitter/sign-flips as ball or goal crossed the robot's heading) kept resetting `missCycles` straight to `0` before it ever reached `MISS_CYCLE_LIMIT`, so `walkMode` never reached `3` despite a clearly sustained miss in that log. | Only clear `missCycles` outright on a cycle that's genuinely closing in (a real miss never looks like that, even briefly); a flat/noisy cycle now decays the counter by one instead of zeroing it, so the streak survives occasional jitter without masking real recovery. |

### Still open / worth tuning (Stage 4 specific)

- Bug #19 from Stage 3 (the `locationArray()` pose freeze-then-jump) is
  carried over unmodified into `stage4.ts` and still needs the same
  extension-level investigation — kicking doesn't change that diagnosis.
- Whether one missed kick should retry immediately (current behavior, via
  `kickJustFinished` dropping the ball lock) or whether some attempts/cooldown
  limit is needed has not been tested on hardware yet.
- `contactDetected` (bug #9) is untested on hardware. `KICK_DIST_MM` is now
  `10` (tightened by the user past this session's `30`) specifically so that
  the kick stance point matches the kick's real ~1cm reach — by design,
  `distKick` may rarely or never satisfy `distKick <= KICK_DIST_MM` on its
  own at that precision, so `contactDetected` (physical resistance, not
  geometry) is expected to be the *primary* arrival trigger going forward,
  not a rare backstop. Watch the next log for whether `CONTACT_CYCLE_LIMIT`
  (~120ms) and `CONTACT_GATE_DIST_MM` (400mm) need tuning — too short/tight
  risks false triggers from a single noisy cycle, too long/loose reopens
  bug #8's overshoot-into-the-ball problem.
- **New, not-yet-diagnosed report (no hardware log captured yet for this
  specific run):** with the current code (`KICK_DIST_MM=10` +
  `contactDetected`), the user reports the robot still walks/veers left even
  when started already on the kick line — squarely on the same x-axis as
  the ball and goal, with no initial lateral offset to correct for. This is
  notable because it's the same *visible symptom* ("turns/walks left") as
  bug #5 and bug #6 above, both of which were already root-caused and fixed
  this session — so either this is a fresh, separate bias source, or one of
  those fixes is incomplete. Worth checking first against a fresh log,
  before assuming a new root cause: Stage 3 bug #17 only ever confirmed
  `updateControl()`'s internal `kTurn` sign for `walkMode 0` — the separate
  `TURN_GAIN` constant (used for the post-arrival align-to-goal turn, and
  for `walkMode 3`'s back-up-and-reorient turn) is still flagged unverified
  in its own comment and has never been independently confirmed on
  hardware. Stage 3 bug #16's `camToOdom()` head-yaw rotation sign is also
  still flagged unverified — a wrong sign there would bias `ball_now`/
  `kickPt` laterally in a fixed direction any time the head is off-center
  while tracking, which would show up exactly as a consistent pull to one
  side even from a dead-on start.
