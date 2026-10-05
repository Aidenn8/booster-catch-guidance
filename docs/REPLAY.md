# The report page and 3D replay

[`docs/replay/index.html`](replay/index.html) is the study as a single web page: an interactive
3D replay of six simulated landing burns, followed by the write-up with its diagrams, tables
and figures. It is published at **https://aidenn8.github.io/booster-catch-guidance/** and also
works when opened from disk.

- [What it shows](#what-it-shows)
- [Controls](#controls)
- [What is simulated and what is drawn](#what-is-simulated-and-what-is-drawn)
- [How the page is built](#how-the-page-is-built)
- [Testing the page](#testing-the-page)
- [Recording the README media](#recording-the-readme-media)
- [Hosting](#hosting)

## What it shows

**Six scenarios**, each flown by the convex MPC and the baseline from the same initial state
through the same disturbances (and, in the engine-failure scenarios, by the MPC with the
engine-out reserve as well):

| Scenario | Disturbance | MPC | Baseline | MPC + reserve |
|---|---|---|---|---|
| Nominal | none | caught | caught | – |
| Crosswind 15 m/s | steady wind from the north with gusts | caught | caught | – |
| Late ignition | burn starts 200 m lower | caught | safe miss | – |
| Overshoot | arrives 160 m further downrange | caught | caught | – |
| Engine out at 5 s | a centre engine fails at 5 s | pad strike | safe miss | caught |
| Engine out at 10 s | a centre engine fails at 10 s, inside the commit window | pad strike | safe miss | caught |

**In the 3D view:**

* the booster bodies, engine plumes, the tower with its arms, the launch mount, pad, beach and sea;
* each booster's trajectory so far (solid) and its full trajectory (faint dashed), in the
  controller's colour (blue: convex MPC, orange: baseline, pink: MPC with the engine-out reserve);
* the MPC's current plan, a light dashed line ahead of its booster, which jumps at every re-plan;
* a caption with the time, the flight phase and the scenario's disturbances, and altitude
  tags on each booster.

**Below the view:** a telemetry table (phase, altitude, descent rate, lateral speed, thrust
and live engines, propellant) with each controller's final result, and a log–log plot of
descent rate against height above the catch point with the terminal gate marked.

## Controls

| Action | Mouse / trackpad | Keyboard | Touch |
|---|---|---|---|
| Play / pause | play button, or "Play replay" | `Space` | tap play |
| Step one second | – | `,` and `.` | – |
| Scrub | drag the timeline (dots mark catches, misses and engine failures) | – | drag |
| Rotate the view | drag | – | one-finger drag |
| Pan the view | right-drag, or the on-screen arrows | arrow keys or `W` `A` `S` `D` | two-finger drag |
| Zoom (towards the cursor) | scroll, or the on-screen `+` / `−` | `Q` / `E`, `+` / `−` | pinch |
| Re-centre on a point | double-click it | – | – |
| Back to the default camera | **Reset view** button (top right) | `R` | tap Reset view |
| Follow a booster | click its row in the telemetry table | `Enter` on a focused row | tap its row |

**Camera modes:**

* **Auto** (default): cuts between a chase shot, a wide tower shot and a side-on view of the
  arms as the flight progresses, following the selected booster.
* **Follow**: tracks the selected booster while you rotate and zoom around it; your angle is kept.
* **Tower**: a side-on close-up of the arms, through the gap between the booster and the tower.
* **Wide**: the whole trajectory from far away.
* **Free**: stays wherever you put it. Moving the view in Auto, Tower or Wide switches to Free;
  Reset view returns to Auto.

**Options:** playback speed (¼×, ½×, 1×) and lighting (sunset, day, night, with floodlights,
aviation lights and stars).

## What is simulated and what is drawn

The simulator is a point-mass model, so some of what you see is a rendering choice:

| Simulated (from `flights.json`) | Rendering choice |
|---|---|
| Centre-of-mass position and velocity | Body attitude: the body leans about a third of the way into the thrust direction (more only if 15° of engine gimbal would not cover the rest), smoothed over 0.75 s |
| Thrust vector and number of live engines | Plume direction (along the true thrust, so the plumes deflect relative to the body like gimballed engines), length and brightness |
| Propellant, flight mode, outcome | A clearance guard keeps the drawn body at least 1 m from the tower face; the tower check in the simulation uses the centre of mass plus the booster's radius |
| The MPC's plans | Terrain, sea, clouds, smoke and debris |

Drawing the whole 71 m body at the full thrust tilt about its centre of mass would swing the
engine end several metres into the tower in the picture, even though the simulated booster
never comes within 1.75 m of it; the partial lean avoids that artefact.

Particle effects (ground blast, steam, crash fireball) are computed as a function of replay
time, so they look the same when scrubbing backwards or forwards.

## How the page is built

```
scripts/export_replay.py   flies the six scenarios       ->  docs/replay/flights.json
docs/replay/template.html  page shell, styles, 3D engine, interface (hand-written)
docs/replay/paper.html     the report body: sections, diagrams, tables (hand-written)
docs/images/*.png          figures from scripts/make_figures.py
scripts/build_replay.py    inlines all of the above      ->  docs/replay/index.html
```

`build_replay.py` replaces `/*FLIGHTS_JSON*/` in the template with the flight data and
`<!--FINDINGS-->` with the report body, and inlines every `src="img:<name>.png"` figure as a
`data:` URI. The result is one HTML file (about 1.6 MB) with no local dependencies. Passing a
path as an argument also writes a copy without the `<html>`/`<head>`/`<body>` wrapper, for
hosts that add their own.

**External resources**, all from public CDNs: three.js r128 and its examples
(OrbitControls, Sky, post-processing, fat lines), MathJax 3 for the two equations, and the
Inter and IBM Plex Mono fonts from Google Fonts.

**Rendering:** physically based materials lit by a sun and a sky environment map, shadow
maps, a half-float render target with 4× multisampling, bloom on bright plumes and lamps, and
a final pass with ACES tone mapping, a slight vignette and dither. Terrain combines a fine
sand texture with a large-scale map of scrub, tidal flats and dunes. The sea is a shader
with analytic waves, Fresnel reflection, sun glint and surf along the shore.

**Editing:** change `template.html` or `paper.html`, then run `python scripts/build_replay.py`.
Never edit `index.html` directly; it is overwritten on every build.

## Testing the page

Two headless-browser checks (install `requirements-dev.txt` and run
`python -m playwright install chromium` first):

```bash
python scripts/check_replay.py      # console errors, layout overflow, frame rate, equations typeset,
                                    # screenshots of 12 moments, cameras and lighting, desktop and phone
python scripts/check_camera.py      # 12 checks of the camera controls (zoom to cursor, pan, keys,
                                    # pad, double-click, Reset view, Follow keeping your angle)
```

Screenshots go to `work/` (ignored by git). The checks expect a GPU-capable Chromium; on macOS
they use Metal through ANGLE.

## Recording the README media

```bash
python scripts/record_media.py      # docs/media/catch.gif and docs/media/hero.jpg
```

Frames are rendered by scrubbing the replay clock rather than by screen-recording playback, so
the result does not depend on how fast the machine renders. The final-pass dither is switched
off for the GIF (per-pixel noise defeats GIF compression).

## Hosting

GitHub Pages serves the `docs/` folder of the `main` branch. `docs/index.html` redirects to
`replay/`, and `docs/.nojekyll` turns off Jekyll processing. Any static host works: the page
is a single file.
