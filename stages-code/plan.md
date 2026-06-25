# Autonomous Soccer Robot: 5-Stage Deployment Blueprint

To successfully build and deploy your autonomous soccer robot following your professor's criteria, you should implement the project in five distinct developmental stages. This **"MVP-First" (Minimum Viable Product)** roadmap ensures you test the hardware and low-level math before layer-stacking the complex pathfinding and behavior trees.

Here is the complete project blueprint, organized by stage, detailing exactly which files are active, what happens in that stage, and how to verify it works.

---

## 🛠️ Before You Start: Required Fixes on Every `professor-code/` File

> ⚠️ **Important:** `professor-code/` is the professor's raw reference implementation. It still targets an older/renamed build of the extension, so every file needs the same two fixes already applied in this repo's `robotpu-followball.ts` and `stages-code/stage2.ts` before it will compile or run correctly against the `pxt-robotpu/` extension actually installed in this project:

* **Namespace:** `robotPu` $\rightarrow$ `robotPuPro` (the installed extension exports `robotPuPro`; `robotPu` does not exist).
* **Casing:** `robotPu.ServoTargets()` $\rightarrow$ `robotPuPro.servoTargets()` (lowercase `s`).
* **Servo Targets:** Head servo targets are **ABSOLUTE degrees** in $[0,179]$ with $90 =$ neutral, not a small offset clamped to $[-45,45]$.
* **Array Methods:** MakeCode's array type has no `.splice()`—use `.removeAt()` instead (only matters once you reach Stage 5's A* file).

---

## 📍 Stage 1: Hardware Drivers, Data Parsing & Fundamental Math

> **Objective:** Establish basic communication with the external sensors and set up the mathematical groundwork required for spatial geometry.

| Detail | Description |
| :--- | :--- |
| **Files to Attach / Keep Active** | <ul><li>`professor-code/robotpu-i2c-cam.ts` (Camera protocol handler)</li><li>`professor-code/robotpu-utils.ts` (Shared math engine)</li></ul> |
| **What Happens** | <ul><li>The micro:bit opens communication channels over the I2C multiplexer bus to the ESP32 camera.</li><li>It starts pulling 18-byte data packets and decodes raw 16-bit millimeter positions for targeted colors (orange ball, white goal paper).</li><li>`robotpu-utils.ts` provides clamping and radian/degree conversions.</li></ul> |
| **How to Verify** | Open your serial monitor. You should see steady printouts mapping `x_mm` and `y_mm` changes when you wave the orange ball in front of the camera lens. |

---

## 📍 Stage 2: Odometry Tracking & Active Sensor Searching

> **Objective:** Enable the robot to keep track of its own physical positioning in space and actively search for objects when they disappear from sight.

* **Files to Add to the Project:**
    * `professor-code/robotpu-search-soccer.js` (Head camera servo pan/tilt routine)
* **Note on `robotpu-odometry.ts`:** Do **NOT** wire this class into the live robot. `pxt-robotpu` already runs this exact $\text{SE}(2)$ step-tracker internally and every `robotPuPro.walk()` call updates it automatically—just read the result with `robotPuPro.locationArray()` (this is already confirmed working in `stages-code/stage2.ts`). Keep `robotpu-odometry.ts` around only as a reference for the math, e.g. to sanity-check `locationArray()` output by hand.

| Detail | Description |
| :--- | :--- |
| **What Happens** | <ul><li>Every time a leg completes a step, the extension's internal odometry updates coordinate displacement ($x, y, \theta$), readable via `robotPuPro.locationArray()`.</li><li>If the data packet shows the target object is invalid (lost tracking), a state transition hands control over to the head servo loop to sweep looking for the target.</li></ul> |
| **How to Verify** | Walk the robot forward 3 steps manually. Verify `locationArray()` accurately changes its physical coordinate values without freezing or throwing errors. |

---

## 📍 Stage 3: Approach the Ball & Take Up the Kick Position (The "MVP" Grade B/B+)

> **Objective:** Compensate for data latency, smooth out reading errors, calculate where to stand to score, drive there, and STOP—no kicking yet.

* **Files to Add to the Project:**
    * `professor-code/robotpu-localmap.ts` (Ground plane grid projection + kick point computation)
    * `professor-code/robotpu-kalman-filter.ts` (Constant-velocity tracking estimation)
    * `professor-code/robotpu-viewpoint.ts` (Arc steering motion control)
    * `professor-code/robotpu-soccer-mvp.ts` (Core logic calculator)
* **Important:** `robotpu-soccer-mvp.ts`'s main loop already contains **BOTH** stages 3 and 4 combined—it computes the kick point, drives to it (`walkMode` 0/1), aligns to face the goal, and then (`walkMode` 2) calls `robotPuPro.kick()`. For this stage, **comment out / disable the `walkMode == 2` branch** (the `robotPuPro.kick()` call) so the robot stops and holds position once it reaches the kick pose, instead of kicking. Re-enabling that branch is exactly what Stage 4 below does.

| Detail | Description |
| :--- | :--- |
| **What Happens** | <ul><li>The raw camera data coordinates are projected down onto a flat local ground plane map.</li><li>The 2D Kalman filters smooth out rapid twitches or dropped frames if the camera shakes while walking.</li><li>`soccer-mvp` computes a vector line from the center of the goal, through the ball, and places a target "Kick Pose" coordinate behind the ball (`computeKickPoint()`, `KICK_BACKOFF_M`).</li><li>The viewpoint controller reads this pose and outputs walking speeds to steer the robot in a sweeping arc so it automatically squares up to face the goal.</li></ul> |
| **How to Verify** | The robot should track the ball, walk around it in a smooth curve, stand directly behind it facing the goal, and stop—holding that position (no kick motion) even as detections keep arriving. |

---

## 📍 Stage 4: Perform the Kick

> **Objective:** From the kick pose established in Stage 3, strike the ball so it travels into the goal.

* **Files to Add to the Project:** None new—same files as Stage 3.
* **What Changes from Stage 3:** Re-enable the `walkMode == 2` branch in `professor-code/robotpu-soccer-mvp.ts`'s actuator loop: once the robot is within `KICK_DIST_M` of the kick point and aligned to the goal heading, call `robotPuPro.kick()` every cycle until it returns 0 (the extension's `kick()` must be called repeatedly to complete the motion—see `pxt-robotpu/main.ts`).

| Detail | Description |
| :--- | :--- |
| **What Happens** | <ul><li>The robot holds the Stage 3 approach/align behavior unchanged.</li><li>Once aligned and at the kick distance, `walkMode` flips to 2 and the actuator loop drives `robotPuPro.kick()` to completion.</li><li>After the kick finishes, the loop falls back to re-detecting the ball/goal so it can re-approach for another attempt if the first kick missed.</li></ul> |
| **How to Verify** | From the Stage 3 hold position, the robot performs the kick motion and the ball is pushed toward/into the goal. Confirm the kick triggers only once per approach (no repeated kicking while still in `walkMode` 0/1). |

---

## 📍 Stage 5: Advanced Obstacle Avoidance & The Behavior Tree (The "A/A+" Grade)

> **Objective:** Prevent collisions with field obstacles, manage logical decisions cleanly, and replace the flat if/else planner from Stages 3-4 with a Behavior Tree.

| Detail | Description |
| :--- | :--- |
| **Files to Add to the Project** | <ul><li>`professor-code/robotpu-A-star.ts` (Grid pathfinding)</li><li>`professor-code/robotpu-pure-pursuit.ts` (Multi-waypoint tracking)</li><li>`robotpu-soccer-final.ts` (This repo's already-fixed, `robotPuPro`-correct integrated Behavior Tree script—located at the root of the repo, not `professor-code/`)</li></ul> |
| **What Happens** | <ul><li>Instead of driving blindly towards the kick point, the local ground plane maps out grid obstacles (like opponent robots).</li><li>A-star finds the shortest cell pathway around the obstacles.</li><li>Pure-pursuit steps through those path waypoints sequentially, feeding lookahead coordinates directly into the movement system.</li><li>The Behavior Tree structured loop systematically evaluates condition nodes (`condBallVisible`, `condScored`) to seamlessly toggle between searching, chasing, approaching, kicking, and celebrating without massive nested code statements.</li></ul> |
| **How to Verify** | The robot navigates around obstacles, stops precisely when it hits the arrival threshold behind the ball, triggers its striking macro to score, and flashes a celebration icon. |

---

## ⚠️ Critical Compilation Step for the Final Deployment

> Once you reach Stage 5, you must merge the logic blocks into a single workspace.

Because separate files in MakeCode share a global namespace, keeping all these files separate inside your folder simultaneously will cause "Duplicate Identifier" compiler crashes due to overlapping definitions of constants like `SIZE`, variables like `walkSpeed`, or classes like `Pose2D`. 

`robotpu-soccer-final.ts` is already structurally packed to contain these variables together—**ensure it is the lone master file in your final compiled deployment!**
