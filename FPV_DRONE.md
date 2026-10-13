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
joystick mode) and map it in F8 > Radio. Betaflight Configurator can connect to
`ws://localhost:5761`.

Mario ignores all input while the drone is up, except Start. Settings,
EEPROMs and logs are kept in the game's user folder
(`%APPDATA%\sm64ex\libfpv-data`, `%APPDATA%\sm64ex\fpv.log`).

## Building

Build libfpv first (see its README), then build sm64ex with `FPV_DRONE=1`.
`LIBFPV_DIR` defaults to `../libfpv`. `LIBFPV_BUILD` defaults to its `build/win`
folder for Windows builds, `build/mac` on macOS and `build/linux` otherwise.

```sh
# Linux or macOS
make FPV_DRONE=1

# Windows, cross-compiled from Linux with MinGW-w64
make WINDOWS_BUILD=1 CROSS=x86_64-w64-mingw32- CC=x86_64-w64-mingw32-gcc \
     CXX=x86_64-w64-mingw32-g++ TARGET_BITS=64 FPV_DRONE=1
```

`libfpv.dll` (or `libfpv.so.0`, or `libfpv.0.dylib`) is copied next to the executable, and the
game needs it at run time. Only the OpenGL renderer draws the lens effect, the
OSD and the overlay.

## Self test

`SM64_FPV_TEST=<level>[,chase|los][,overlay][,dive]` boots through the debug
level select into a level (9 = Bob-omb Battlefield, 12 = Jolly Roger Bay). It
then deploys the drone, flies a scripted autopilot through a coin line and a
Goomba, recalls the drone and deploys it again. It writes telemetry to `fpv.log`
and screenshots to `fpv_shot*.bmp` in the user folder.

## How it hooks the game

All new code is in `src/pc/fpv/fpv_sm64.c` and `src/pc/gfx/gfx_fpv_gl.c`.
Changes to existing files are guarded by `#ifdef FPV_DRONE`:

- **Camera.** `rendering_graph_node.c` takes the full camera orientation from
  libfpv, so the view can roll and flip. It also takes the FOV and near plane.
- **Skybox.** In a drone camera, `skybox.c` wraps the skybox tiles around the
  camera as a sphere.
- **Lens, OSD and overlay.** `area.c` puts a marker in the display list after
  the 3D scene. `gfx_pc.c` applies the fisheye/analog noise post-process there,
  before the HUD, and draws the OSD and overlay at the end of the frame.
- **Collision.** `surface_load.c` reports the static surfaces of each area, and
  every object collision model while it is tangible.
- **Culling.** `behavior_script.c`, `surface_load.c` and the room switch in
  `object_helpers.c` cull by the closer of Mario and the drone.
- **Input.** Three new binds are added to the configuration, the options menu
  and the keyboard/SDL2 controllers. Window events are forwarded to libfpv in
  `gfx_sdl2.c`.
- **Audio.** `pc_main.c` mixes in the synthesized motor sound.
