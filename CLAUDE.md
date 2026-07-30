# ME101 Wall-E (VEX-E) — Project Handoff

Group 20: Daniel Chen, Jintai Li, Luke Liu, Sam Zhang. VEX IQ robot that autonomously
searches a field for trash, scoops it, compresses it, and dumps it in a disposal box.

## ⚠️ Read this before doing anything else

**This repo has diverged badly from `origin/main`, and there is a lot of uncommitted
work sitting in the working directory right now.** If you're moving to a new computer,
do NOT just `git clone` the GitHub repo fresh — you will lose everything below unless
you either commit it first or physically carry over this whole folder (not just `.git`).

- Local `main` has 4 commits origin doesn't have (`a056625`..`46aab2c`) — compress
  state wiring, the zigzag `pathFind`, the disposal-box/odometry work, single-scoop fix.
- `origin/main` has ~7 commits local doesn't have — a teammate's parallel rewrite,
  including a whole alternate file `COPY_PASTE_THIS_Sam_did_something.cpp` committed
  to the repo, with a genuinely different (and in some ways better — current-sensing
  compress, unified `seesColor()`) architecture. See "Teammate's alternate version" below.
- **`COPY_PASTE_THIS.cpp` and `VEX_E.iqcpp` both have uncommitted changes right now**
  (everything from the disposal-box feature through the v17 detection-range tuning —
  see version history below). Nothing past `46aab2c` is committed yet.
- You do not have push access to `jintai522/ME101_Wall-E` (account `smzhng` gets a 403).
  A real merge with the teammate's origin commits, plus sorting out push access, is
  still an open task.

**Recommended first step on the new machine**: don't re-clone. Copy this entire
`wall-e` folder (including the uncommitted state) to the new computer, or at minimum
commit everything locally before you leave this machine.

## Repo layout

- `ME101_Wall-E/` — the actual git repo (this file lives here).
  - `COPY_PASTE_THIS.cpp` — the real source of truth for the robot logic. Edit this one.
  - `VEX_E.iqcpp` — the VEXcode project file (JSON-wrapped). VEXcode opens *this* file
    directly. See the sync gotcha below — it is NOT safe to assume this matches
    `COPY_PASTE_THIS.cpp` unless you just re-synced it.
  - `COPY_PASTE_THIS_Sam_did_something.cpp` — a teammate's independent rewrite (pushed
    to origin, not yet pulled locally as of this writing). See summary below.
- `wall-e/` (parent folder, **not part of the git repo** — will NOT transfer via git):
  - `compression test.iqcpp` — standalone test project for the compressor motor
    (`compress()` degree/current-stall logic), built before it was ported into the main file.
  - `hue test.iqcpp` — standalone optical sensor calibration tool: prints live
    hue/brightness/color to the screen + Console, used to tune `GREEN_HUE_MIN/MAX`
    and `RED_HUE_MIN/MAX` against real tape.
  - `arms test.iqcpp` — standalone test for two mirrored arm motors (port 4 + port 10,
    one declared reversed since they're mounted facing opposite directions), swings
    180° out and back.
  - **These three files must be manually copied to the new computer** — they aren't
    tracked by git at all.

## Critical gotcha: VEXcode autosave clobbers direct file edits

`VEX_E.iqcpp` is a JSON file (`textContent` field holds the actual C++ as an escaped
string). When Claude edits it directly while VEXcode has the project open, **VEXcode
periodically autosaves its own stale in-memory copy back over the file** — this
happened repeatedly throughout this session (confirmed via file timestamps: the sync
would land, then get silently reverted minutes later).

**Workflow that's been in use**: edit `COPY_PASTE_THIS.cpp` (plain text, safe), then
splice its logic into `VEX_E.iqcpp`'s `textContent` via a PowerShell script that
preserves `VEX_E.iqcpp`'s own VEXcode-generated header/device config:

```powershell
$vexPath = "C:\Users\samzh\Desktop\wall-e\ME101_Wall-E\VEX_E.iqcpp"
$copyPath = "C:\Users\samzh\Desktop\wall-e\ME101_Wall-E\COPY_PASTE_THIS.cpp"

$vexJson = Get-Content -Raw $vexPath | ForEach-Object { $_ -replace "^\xEF\xBB\xBF", "" } | ConvertFrom-Json
$copyText = (Get-Content -Raw $copyPath) -replace "`r`n", "`n"

$marker = "#pragma endregion VEXcode Generated Robot Configuration"
$vexHeader = $vexJson.textContent.Substring(0, $vexJson.textContent.IndexOf($marker) + $marker.Length)
$copyIdx = $copyText.IndexOf($marker)
$copyBody = $copyText.Substring($copyIdx + $marker.Length)

$vexJson.textContent = $vexHeader + $copyBody
$outJson = $vexJson | ConvertTo-Json -Depth 20 -Compress
[System.IO.File]::WriteAllText($vexPath, $outJson, (New-Object System.Text.UTF8Encoding($false)))
```

Then validate with Node (`JSON.parse` + check for expected strings/functions).

**This still isn't reliable** — the clobbering happened 4-5 times this session. The
user was mid-discussion about switching to a fully manual workflow (close VEXcode
before edits, or paste `COPY_PASTE_THIS.cpp` into the editor by hand) when the
computer switch came up. **Recommend deciding on and committing to one workflow before
continuing**, and always verify with the version marker below after any sync.

## Version marker convention

`CODE_VERSION` (a `const char*` near the top of the custom code section) gets bumped
every time a meaningful change is made, and is printed to the brain screen at startup
(`Code v17-...` / `Press to start`) so you can visually confirm the robot is running
what you think it's running — this was added specifically because of the autosave
clobbering problem. Current version as of this file: `v17-detect-60mm`. Keep bumping it.

## Current architecture of `COPY_PASTE_THIS.cpp`

**Devices**: `LeftMotor`/`RightMotor` (drive, ports 7/12), `Scooper8` (port 8),
`MotorCompress9` (port 9), `MotorDoor3` (port 3, declared but not yet used in logic),
`Distance2` (port 2, object detection), `Optical11` (port 11, tape detection),
`BrainInertial` (heading).

**State machine** (`run_state` in `main()`):
- `1` — `pathFind()`: infinite zigzag search. Drives forward until green tape
  (`driveUntilGreen`), turns 90°, shifts a fixed `SHIFT_DISTANCE` (250mm), turns 90°
  again, alternating L-L/R-R each row transition. Runs forever until an object is found
  — there is no row limit or field-width constant anymore (removed per user request).
- `2` — `scoopermove()`: single scoop attempt (back up, lower scooper, drive forward in
  stages, raise scooper), then transitions to state 3.
- `3` — `compress()`: pushes compressor `reverse` then `forward` (hardware-confirmed
  direction — reverse = extend/open, forward = compress inward), then a blind clearance
  drive so it doesn't immediately re-detect the same spot, then → state 4.
- `4` — `goToDisposalBox()`: computes the vector to (X=0, Y=`fieldLength/2`) using
  odometry, turns to absolute 90° (right, relative to the very first starting heading),
  drives until red tape (`driveUntilRed`), then `dumpTrash()` (lower scooper, push
  compressor out via `reverse`, retract via `forward`), then ends the program.

**Odometry**: `posX`/`posY`/`totalHeadingDeg` are dead-reckoning estimates (wheel
encoders + inertial), updated by `trackMove()` (called from `finishDrive()`) and
`rotateRobot()`. `fieldLength` is discovered at runtime — measured the first time a
row completes cleanly (reached green tape, not cut short by an object) — since the box
size varies and can't be hardcoded. **Known limitation**: if the first object is found
before ever completing one full clean row, `fieldLength` stays 0 and the disposal-box
math breaks. Not yet handled.

**Heading-drift fix** (the most recent architectural change): `BrainInertial.resetRotation()`
now only happens once, at program start. All drive functions correct toward
`totalHeadingDeg` (a persistent absolute reference) instead of a per-call "0" — the old
behavior silently let misalignment compound every leg. `snapHeadingToGrid()` also
re-corrects to the nearest 90° multiple every time tape is reached, as an active
checkpoint against the tape itself.

**Speed ramp**: first pass through a row is always 1x (`PATH_SPEED`, currently 45)
since `fieldLength` isn't known yet. Every pass after that goes 2x (`FAST_MULTIPLIER`)
until `SLOWDOWN_START_FRACTION` (80%) of the known length, then eases back to 1x for
the last stretch so it doesn't overshoot the tape.

**Smooth stop**: `finishDrive(fromSpeed)` ramps power down in `STOP_RAMP_STEP` (10%)
increments before the final brake, instead of slamming to a stop — this was causing
wobble/misalignment, especially when stopping from 2x speed. `scoopermove()` was also
changed to use heading-corrected `driveForDuration()` instead of raw uncorrected motor
commands, so a misaligned approach doesn't get worse during the scoop.

**Object detection range**: `SCOOP_DETECT_MIN`/`MAX` (currently 10mm/60mm) — started at
10-30mm (too short, detected too late), widened to 150mm (caught the ground — sensor
sits slightly angled down), currently at 60mm as a middle ground. Still likely needs
real-world tuning.

**Tape detection**: `seesGreenTape()`/`seesRedTape()` use hue-range checks (not exact
color match, which was unreliable) — `GREEN_HUE_MIN/MAX` (75–160°) and `RED_HUE_MIN/MAX`
(340–20°, wraps around 0). Both show a live hue/brightness/color readout on the brain
screen (throttled via `STATUS_REFRESH_INTERVAL` so it doesn't slow the drive loop).

## Teammate's alternate version (`COPY_PASTE_THIS_Sam_did_something.cpp`)

Pushed to origin, not yet reconciled with local work. Worth cherry-picking ideas from:
- **Current-sensing compress** (`compressRobot`): pushes until `current(amp) < 0.4`
  becomes false (senses resistance) rather than a fixed degree target — more adaptive.
- **Blue tape marks the dump zone** (not red) — checked as a secondary condition
  inside `driveUntilGreen()` itself.
- **Unified `seesColor(hueMin, hueMax, colourCode)`** instead of separate green/red functions.
- Uses a **fixed-maneuver return path** (no odometry) — back up, turn, drive to green
  line, cross it, turn, drive to blue zone. Simpler but less adaptive to field-size
  variance than our odometry approach.
- An unused `movedoor()` function (current-sensing door control) — not wired into `main()`.

## Open items / next steps

1. Reconcile local commits with origin's diverged history (real merge needed, not a
   simple push) — and separately, resolve GitHub push access (`smzhng` gets 403 on
   `jintai522/ME101_Wall-E`).
2. Decide on a stable VEXcode sync workflow (see gotcha above) — this cost significant
   back-and-forth this session.
3. `fieldLength`-unknown edge case in `goToDisposalBox()` (see Odometry above).
4. Discussed but not implemented: using a second/outer tape color as a field-boundary
   marker. Leaning toward using it to discover field *width* the same way `fieldLength`
   is discovered, rather than a full position-recovery system — see chat history if
   picking this up.
5. Real-world tuning still needed: `SCOOP_DETECT_MIN/MAX`, `GREEN_HUE_MIN/MAX`,
   `RED_HUE_MIN/MAX`, `STOP_RAMP_STEP`, scoop-lowering duration in `dumpTrash()` (2s
   placeholder, untested).
