# FPV drone mode

Mario can deploy an FPV quadcopter and fly it around the level. The drone runs
real Betaflight firmware through [libfpv](../libfpv), so rates, angle mode,
arming and crashes behave like a real 5" quad. Mario stays where he is as the
pilot: the line-of-sight camera and the analog video link are measured from him,
so the picture gets noisy when you fly far away or behind walls.

The world is at real scale (100 units per metre, so Mario is about 1.6 m tall).

## What the drone does

- **Collision.** It collides with the whole level, plus moving platforms,
  doors, Whomps and anything else with a collision model. Hitting lava or water
  destroys the propellers.
- **Collecting.** Flying through coins (yellow, red and blue), stars, keys and
  1-ups collects them. A star ends the flight and Mario does his star dance.
- **Enemies.** Goombas, Koopas, Bullies, Mr. Blizzards, Bob-ombs and breakable
  boxes are knocked out when you hit them faster than 2 m/s.
- **Rooms and draw distance.** Both follow the drone, so objects near it are
  drawn even when they are far from Mario.
- **Recall.** The drone is recalled when Mario is hurt badly, dies, starts a
  cutscene, or the level or area changes.
- **GPS.** Mario is the GPS home point, so the OSD's home arrow and home
  distance (enable them in the Configurator's OSD tab) always lead back to him.

## Kamikaze drone

With libfpv built with `-DLIBFPV_KAMIKAZE=ON` and the game with `FPV_KAMIKAZE=1`
(see Building), the drone can carry a warhead. Without them nothing changes.

- **Installing.** The bomb is not installed by default: the drone flies like
  in any other build until it is ticked in F8 > Fly > "Bomb installed". It
  stays on every new or repaired drone until the game is closed (it is not
  saved).
- **Arming.** The motors only arm with the bomb arm switch (AUX7) off. Take
  off, flip it, and the warhead is live (OSD `BOMB_ARMED`; with an arming delay
  set in F8 > Fly, `ARMING 3` first). K toggles it on the keyboard, X (West)
  on a gamepad. The bomb status is a Betaflight OSD element ("Custom message
  1"): move or hide it in F8 > OSD > "Bomb status" or the Configurator.
- **Detonation.** A live warhead goes off on any touch, including a light tap
  on the ground and any enemy or boss, or on the explode switch (AUX8; X held
  on the keyboard, the right shoulder on a gamepad). With the bomb switch off
  nothing explodes.
- **Blast.** A burst of the game's explosions. Every enemy within 5 m is
  slain: through the game's own defeat where an attack has one (so it drops
  its coins and the level counts it), and otherwise on the spot. Bob-ombs go
  off too. Each boss in range takes one hit, the way Mario hurts it: King
  Bob-omb, King Whomp, the Big Boos, Wiggler, Eyerok's hands and Bowser lose
  one health, and the big bullies are blown away (into the lava if it is
  close). Mario is hurt if he stands in the explosion itself.
- **After the explosion.** Options > Controls > "After the Explosion" (or the
  button in F8 > Fly) picks the ending. **Signal Lost** (default) keeps the
  goggles on screen for 1 s: static with libfpv's `SIGNAL LOST`, and none of
  the game's HUD over it, like a real lost feed. Then Mario takes over again.
  **Arcade** shows the blast from the pilot's side, short of walls, for 1 s,
  then a new drone takes off from the launch point. Drone Reset gives a new
  drone at once instead, and Drone Toggle recalls it.
- **Payload, arming delay and fuse.** The warhead is real weight (by default
  60 % of the quad). F8 > Fly tunes it, the arming delay (none by default) and
  the impact fuse.

## Controls

| Action | Keyboard | Gamepad |
|---|---|---|
| Deploy / recall the drone | F | Back / Select |
| Repair and respawn at the launch point | R | D-pad up |
| Camera: FPV / chase / line of sight | V | D-pad right |
| libfpv overlay (radio setup, presets, rates, camera, Betaflight CLI) | F8 | |
| Pause | Space (Start) | Start |

All three drone actions can be rebound in Options > Controls ("Drone
Toggle", "Drone Reset", "Drone Camera").

To fly with the keyboard, Enter arms, W/S sets the throttle, A/D yaws and the
arrow keys pitch and roll (self-levelling). T toggles turtle mode.

With a gamepad (mode 2), A arms, the left stick gives throttle (centre is idle)
and yaw, and the right stick gives pitch and roll. Y toggles turtle mode.

For the real feel, plug in a radio (EdgeTX, OpenTX, ELRS, Crossfire in USB
joystick mode) and map it in F8 > Radio. Besides arm, angle, beeper and
turtle, the wizard can bind an air mode switch (AUX5) and an OSD disable
switch (AUX6). With OSD disable on, the FPV view is clean: the Betaflight OSD
and the game's HUD and text are hidden. The pause menu and the F8 overlay
still show. Betaflight Configurator can connect to `ws://localhost:5761`.

F8 > Fly also has buttons to deploy or recall the drone, respawn it and switch
the camera.

While the drone is up, Mario holds a radio whose sticks follow yours. He watches
the drone, turns around when it flies behind him, and leans into the turns.
He ignores all input except Start. Settings,
EEPROMs and logs are kept in the game's user folder
(`%APPDATA%\sm64ex\libfpv-data`, `%APPDATA%\sm64ex\fpv.log`).

## Framerate

The game logic still runs at 30 Hz, but the picture is not capped: every
displayed frame interpolates between the last two game frames (based on the
60 FPS patch in `enhancements/`), so the framerate follows your monitor with
Vertical Sync on, or goes as high as the GPU allows with it off. The drone is
simulated once per displayed frame, and its cameras and model use its live
pose, so the view responds at the full framerate. "Uncapped Framerate" in
Options > Display (`uncapped_framerate` in the config) switches back to 30 FPS.

## Building

Build libfpv first (see its README), then build sm64ex with `FPV_DRONE=1`.
`LIBFPV_DIR` defaults to `../libfpv`. `LIBFPV_BUILD` defaults to its `build/win`
folder for Windows builds, `build/mac` on macOS and `build/linux` otherwise.

For the kamikaze drone, build libfpv with `-DLIBFPV_KAMIKAZE=ON` into the same
folder with a `-kamikaze` suffix (`build/mac-kamikaze`...) and add
`FPV_KAMIKAZE=1`. When switching between the two variants, `make clean` (or
touch the files that include `libfpv.h`): `fpv_frame` has two more channels
in kamikaze builds.

```sh
cmake -S ../libfpv -B ../libfpv/build/mac-kamikaze -DLIBFPV_KAMIKAZE=ON && cmake --build ../libfpv/build/mac-kamikaze -j
make FPV_DRONE=1 FPV_KAMIKAZE=1
```

```sh
# Linux or macOS
make FPV_DRONE=1

# Windows, cross-compiled from Linux with MinGW-w64 (see the README for the dependencies)
make WINDOWS_BUILD=1 CROSS=x86_64-w64-mingw32- TARGET_BITS=64 FPV_DRONE=1 \
     SDLCONFIG=$HOME/mingw-deps/x86_64-w64-mingw32/bin/sdl2-config
```

`libfpv.dll` (or `libfpv.so.0`, or `libfpv.0.dylib`) is copied next to the executable, and the
game needs it at run time. Only the OpenGL renderer draws the lens effect, the
OSD and the overlay (other renderers tell libfpv there is no lens pass).

## Self test

`SM64_FPV_TEST=<level>[,chase|los][,overlay][,dive][,clean][,pilot[=<degrees>]][,kamikaze[,arcade]]` boots through the debug
level select into a level (9 = Bob-omb Battlefield, 12 = Jolly Roger Bay). It
then deploys the drone, flies a scripted autopilot through a coin line and a
Goomba, recalls the drone and deploys it again. It writes telemetry to `fpv.log`
and screenshots to `fpv_shot*.bmp` in the user folder. `clean` turns the OSD
disable switch on after 6 s. `pilot` films Mario during the flight instead of
a drone camera, from in front of him or `<degrees>` around him.
`kamikaze` (kamikaze builds) installs the bomb, turns the bomb arm switch on after take-off and
strikes the Goomba, then plays the signal lost ending and redeploys (or the
arcade ending with `arcade`, which respawns the drone). It saves `fpv_shot_boom0.bmp`, `fpv_shot_boom1.bmp`
and `fpv_shot_after.bmp`.

## How it hooks the game

All new code is in `src/pc/fpv/fpv_sm64.c` and `src/pc/gfx/gfx_fpv_gl.c`.
Changes to existing files are guarded by `#ifdef FPV_DRONE`:

- **Camera.** `rendering_graph_node.c` takes the full camera orientation from
  libfpv, so the view can roll and flip. It also takes the FOV (re-widened for the
  fisheye at the window's aspect) and near plane.
- **Skybox.** In a drone camera, `skybox.c` wraps the skybox tiles around the
  camera as a sphere.
- **Lens, OSD and overlay.** `area.c` puts a marker in the display list after
  the 3D scene. `gfx_pc.c` applies the fisheye/analog noise post-process there,
  before the HUD, and draws the OSD and overlay at the end of the frame.
- **Collision.** `surface_load.c` reports the static surfaces of each area, and
  every object collision model while it is tangible.
- **Culling.** `behavior_script.c`, `surface_load.c` and the room switch in
  `object_helpers.c` cull by the closer of Mario and the drone.
- **UI.** `area.c` drops the HUD and text labels from the display list while
  the OSD disable switch is on in the FPV view, and over the static after a
  kamikaze blast.
- **Input.** Three new binds are added to the configuration, the options menu
  and the keyboard/SDL2 controllers. Window events are forwarded to libfpv in
  `gfx_sdl2.c`.
- **Audio.** `pc_main.c` mixes in the synthesized motor sound.
- **Kamikaze.** Everything else is in `fpv_sm64.c` under `LIBFPV_KAMIKAZE`;
  `options_menu.c` adds the ending choice (`fpv_boom_ending` in the config).
- **Framerate.** `pc_main.c` runs a game frame every 33 ms and calls
  `fpv_sm64_frame_tick` before each displayed frame. `rendering_graph_node.c`
  takes the drone camera live when it blends a frame, moves the drone model to
  its live pose, and skips frustum culling, because the view can turn between
  game frames.
