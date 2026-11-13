# FPV drone mode

Mario can deploy an FPV quadcopter and fly it around the level. The drone runs
real Betaflight firmware through [libfpv](../libfpv), so rates, angle mode,
arming and crashes behave like a real 5" quad. Mario stays where he is as the
pilot: the line-of-sight camera and the analog video link are measured from him,
so the picture gets noisy when you fly far away or behind walls.

The FPV view looks like goggles: libfpv's video feed (F8 > Camera > "Video
feed", both on by default) draws the analog link's snow, sparkles, torn and
rolling lines and colour loss, or a digital link's breakup, and the camera's
resolution and 4:3 aspect with black bars, the analog OSD inside the noisy
signal. Turned off, the view is the fisheye alone. The game's coin, lives and
star counters are hidden in the FPV view (the OSD owns it); health and the
timer stay.

The world is at real scale (100 units per metre, so Mario is about 1.6 m tall).

## What the drone does

- **Collision.** It collides with the whole level, plus moving platforms,
  doors, Whomps and anything else with a collision model. Water is no
  obstacle: the drone goes in, slowed down, and each prop under the surface
  wears away (a whole prop in about a second). Lava is a floor that burns the
  props near it (in about half a second).
- **Damage.** Whatever would hurt Mario hurts the props: flames burn all four
  (about half a prop a second), the game's explosions (Bob-ombs, Bowser's
  mines) break all four within 3 m, and enemies, shots, Amps, Bubba and the
  clams break the prop nearest to them. F8 > Drone > "Props and motors take
  damage" turns all of it off.
- **Wind.** Every level has a steady breeze of its own (up to 6 m/s on Rainbow
  Ride, 5 m/s in Snowman's Land and on Tall, Tall Mountain), never dead calm;
  libfpv adds the gusts, the stronger wind higher up and the shelter behind
  walls. Interiors (the castle, Big Boo's Haunt, Hazy Maze Cave, the slides...)
  are calm. The Fwooshes and Snowman's Land's snowman blow the drone away as
  they blow Mario away, and explosions shove it. F8 > Fly > Wind turns it off
  or scales it.
- **Collecting.** Flying through coins (yellow, red and blue), stars, keys and
  1-ups collects them. A star ends the flight and Mario does his star dance.
- **Enemies.** Goombas, Koopas, Bullies, Mr. Blizzards, Bob-ombs and breakable
  boxes are knocked out when you hit them faster than 2 m/s.
- **Rooms and draw distance.** Both follow the drone, so objects near it are
  drawn even when they are far from Mario.
- **The world around the drone.** Objects spawn, appear, wake up and aim by
  their distance to Mario (coin formations, Pokeys, Whomps, Fwoosh clouds...).
  So, as gtaiv-fpv does with Niko, once the drone is 20 m from Mario his
  object stands in at the drone while the other objects update, until it is
  back within 10 m (F8 > Fly > Pilot turns the stand-in off and sets both
  distances). The level comes alive around the drone, and enemies there
  go for it. Mario himself stays where he is: enemies can't hurt him at the
  drone, nobody starts a conversation with the stand-in, and his own dust,
  splashes and stars stay with him.
- **Move Mario here.** Y recalls the drone and puts Mario on the ground under
  it, facing where it faced. Over lava, a death plane, deadly quicksand or the
  void it buzzes and the flight goes on.
- **Pilot view.** F gives Mario back to you: the game's camera, HUD and
  controls, while the drone stays in the world, armed or not, flown by the
  radio (it keeps flying if it was in the air, or falls if you let go of the
  sticks). Mario's controls come back once F is released. F7 goes back to the
  drone in the view you had. Meanwhile the drone still collects, attacks and
  gets damaged, the line-of-sight camera, the video and RC links and the ear
  follow Mario wherever he walks, and R respawns the drone in front of him.
  Move Mario Here (Y) works from the pilot view too.
- **Recall.** The drone is recalled when Mario is hurt badly, dies, starts a
  cutscene (not in the pilot view, where he goes through doors and reads signs),
  or the level or area changes.
- **GPS.** Mario is the GPS home point, so the OSD's home arrow and home
  distance (enable them in the Configurator's OSD tab) always lead back to him.

## Controls

| Action | Keyboard | Gamepad |
|---|---|---|
| Launch / recall the drone, back to it after F | F7 | Back / Select |
| Back to Mario, the drone left flying (pilot view) | F | D-pad left |
| Repair and respawn (in front of Mario after F) | R | D-pad up |
| Camera: FPV / chase / line of sight | V | D-pad right |
| Recall the drone and move Mario under it | Y | D-pad down |
| libfpv overlay (radio setup, presets, rates, camera, Betaflight CLI) | F8 | |
| Pause | Space (Start) | Start |

All five drone actions can be rebound in Options > Controls ("Drone
Launch", "Pilot View", "Drone Reset", "Drone Camera", "Move Mario Here"). F7
and F are the keys of every libfpv game. A config saved before the pilot view
had Drone Launch (then Drone Toggle) on F; it moves to F7 on the first start.
They are also game controls for the radio: F8 > Radio > "Set up game
controls" puts them on switches or buttons, and there the pilot view control
also goes back to the drone, so both hands stay on the sticks.

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

F8 > Fly also has buttons to launch or recall the drone, go back to Mario,
respawn it, switch the camera and move Mario under it.

While you fly from the drone's views, Mario ignores all input except Start.
While the radio is in use (armed, or a stick or switch moved in the last 3 s)
he holds a radio whose sticks follow yours, watches the drone, turns around
when it flies behind him and leans into the turns; otherwise he idles. F8 >
Fly > Pilot turns the pose off.

Radio mappings, EEPROMs and settings are kept in libfpv's folder shared by
every libfpv game (`%APPDATA%\libfpv`, `~/Library/Application Support/libfpv`,
`~/.config/libfpv`, or `LIBFPV_DATA_DIR`), so they follow you from game to
game. The first start copies an older `libfpv-data` from the game's user
folder there, unless the shared folder already exists. The log stays in the
game's user folder (`%APPDATA%\sm64ex\fpv.log`). F8 > About names the build
(commit, dirty, branch) next to libfpv's, for bug reports.

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

```sh
# Linux or macOS
make FPV_DRONE=1

# Windows, cross-compiled from Linux with MinGW-w64 (see the README for the dependencies)
make WINDOWS_BUILD=1 CROSS=x86_64-w64-mingw32- TARGET_BITS=64 FPV_DRONE=1 \
     SDLCONFIG=$HOME/mingw-deps/x86_64-w64-mingw32/bin/sdl2-config
```

`libfpv.dll` (or `libfpv.so.0`, or `libfpv.0.dylib`) is copied next to the executable, and the
game needs it at run time. Only the OpenGL renderer draws the lens, the video
feed, the OSD and the overlay (other renderers tell libfpv there is no lens
pass and no video feed). The video feed comes from libfpv's
`docs/renderers/fpv_video_gl2.h` (OpenGL 2.1 / GLES 2), on the include path
with `FPV_DRONE=1`.

To start every new player from your own Betaflight setup, pass a `diff all`
file as `FPV_BF_CONFIG=<file>`. It is imported on the first run and the
player's later changes are kept.

### Web

The game, the mod and libfpv compile into one WebAssembly module (libfpv's
`docs/WEB.md`). Build libfpv for the web first, then the game with
`TARGET_WEB=1`:

```sh
source /path/to/emsdk/emsdk_env.sh
emcmake cmake -S ../libfpv -B ../libfpv/build/web && cmake --build ../libfpv/build/web -j
make TARGET_WEB=1 FPV_DRONE=1 -j
python3 -m http.server -d build/us_web 8080   # then open http://localhost:8080/sm64.us.f3dex2e.html
```

Web builds default to the `diff all` of libfpv's web demo
(`FPV_BF_CONFIG=../libfpv/assets/configs/hammerill.diff.txt`); pass
`FPV_BF_CONFIG=` to start from libfpv's defaults instead. `build/us_web` then
holds the page: the `.html`, `.js` and `.wasm`, ready for any static host
(rename the `.html` to `index.html` if you like). The page is
`web/shell.html`.

In the browser, the game's config, the drone settings, EEPROMs and radio
mappings are kept in the browser's IndexedDB (`/libfpv`), and the save file in
its local storage. Radios connect through the Gamepad API: move a stick so the
browser shows it to the page. Betaflight Configurator cannot connect; the
Betaflight CLI is in F8 > Import & CLI.

## Self test

`SM64_FPV_TEST=<level>[,chase|los][,overlay][,dive][,clean][,pilot[=<degrees>]][,here][,far][,fview][,wind]` boots through the debug
level select into a level (9 = Bob-omb Battlefield, 12 = Jolly Roger Bay). It
then deploys the drone, flies a scripted autopilot through a coin line and a
Goomba, recalls the drone and deploys it again. It writes telemetry to `fpv.log`
and screenshots to `fpv_shot*.bmp` in the user folder. `clean` turns the OSD
disable switch on after 6 s. `pilot` films Mario during the flight instead of
a drone camera, from in front of him or `<degrees>` around him.
`here` ends the first flight with Move Mario Here instead of a recall.
`fview` presses Pilot View 3 s into the first flight and Drone Launch (back to
the drone) at 6 s, logging Mario's action and the view. Test flights are in
still air unless `wind` gives them the level's breeze. `far`
moves the first drone 30 m out after 2 s and logs the stand-in.

## How it hooks the game

All new code is in `src/pc/fpv/fpv_sm64.c` and `src/pc/gfx/gfx_fpv_gl.c`.
Changes to existing files are guarded by `#ifdef FPV_DRONE`:

- **Camera.** `rendering_graph_node.c` takes the full camera orientation from
  libfpv, so the view can roll and flip. It also takes the FOV (re-widened for the
  fisheye at the window's aspect) and near plane.
- **Skybox.** In a drone camera, `skybox.c` wraps the skybox tiles around the
  camera as a sphere.
- **Lens, OSD and overlay.** `area.c` puts a marker in the display list after
  the 3D scene. `gfx_pc.c` draws the goggles there, before the HUD: libfpv's
  video feed (`gfx_fpv_gl_video`, the OSD included), or with its effects off
  the fisheye alone, with the OSD at the end of the frame. The overlay is
  drawn at the end of the frame.
- **Collision.** `surface_load.c` reports the static surfaces of each area, and
  every object collision model while it is tangible.
- **Culling.** `behavior_script.c`, `surface_load.c` and the room switch in
  `object_helpers.c` cull by the closer of Mario and the drone.
- **Stand-in.** `cur_obj_update` in `behavior_script.c` moves Mario's object
  to the drone around each object's update while it follows the drone, and
  `mario_ready_to_speak` in `mario_actions_cutscene.c` says no meanwhile.
- **UI.** `area.c` drops the HUD and text labels from the display list while
  the OSD disable switch is on in the FPV view.
- **Input.** Four new binds are added to the configuration, the options menu
  and the keyboard/SDL2 controllers. Window events are forwarded to libfpv in
  `gfx_sdl2.c`.
- **Audio.** `pc_main.c` mixes in the synthesized motor sound. While libfpv
  hears the drone from the pilot in the FPV and chase views, `geo_process_object`
  in `rendering_graph_node.c` positions every object's sounds from Mario's head
  facing the drone instead of the drone camera.
- **Pilot.** `act_fpv_piloting` in `mario_actions_stationary.c` plays the pose
  only while the radio is in use, and Mario leaves it in the pilot view.
- **Wind.** `cur_obj_spawn_strong_wind_particles` reports the Fwooshes' and the
  snowman's gusts.
- **Framerate.** `pc_main.c` runs a game frame every 33 ms and calls
  `fpv_sm64_frame_tick` before each displayed frame. `rendering_graph_node.c`
  takes the drone camera live when it blends a frame, moves the drone model to
  its live pose, and skips frustum culling, because the view can turn between
  game frames.
