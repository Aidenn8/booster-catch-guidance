"""Exercise the replay's camera controls in a headless browser and check each one moves the
view the way it should: scroll-zoom to the cursor, right-drag and key panning, the on-screen
pad, double-click aim, reset, and Follow keeping up with the booster.

    <python with playwright> scripts/check_camera.py
"""
from pathlib import Path

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parent.parent
page_path = ROOT / "docs" / "replay" / "index.html"
out = ROOT / "work" / "camera_check"
out.mkdir(parents=True, exist_ok=True)
STATE = """() => { const r = window.__replay, c = r.camera.position, g = r.controls.target;
  return { mode: r.mode, cam: [c.x, c.y, c.z], tgt: [g.x, g.y, g.z], dist: c.distanceTo(g), t: r.t }; }"""
results = []


def check(name, ok, detail):
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")


with sync_playwright() as p:
    b = p.chromium.launch(headless=True, args=["--ignore-gpu-blocklist", "--enable-gpu", "--use-angle=metal"])
    pg = b.new_page(viewport={"width": 1280, "height": 900})
    errors = []
    pg.on("pageerror", lambda e: errors.append(str(e)))
    pg.goto(page_path.as_uri())
    pg.wait_for_timeout(3000)
    # the replay sits below the title and abstract: bring the whole figure into view first
    pg.evaluate("document.querySelector('#fig-replay').scrollIntoView({block: 'start'})")
    pg.wait_for_timeout(300)
    box = pg.locator("#view").bounding_box()
    cx, cy = box["x"] + box["width"] / 2, box["y"] + box["height"] / 2
    # start on the tower shot, mid-flight
    pg.locator('#cams button[data-cam="tower"]').click()
    pg.locator("#scrub").evaluate("(el) => { el.value = 650; el.dispatchEvent(new Event('input')); }")
    pg.wait_for_timeout(2500)
    s0 = pg.evaluate(STATE)
    pg.locator("#stage").screenshot(path=str(out / "0_tower.png"))

    pg.mouse.move(cx, cy)
    for _ in range(8):
        pg.mouse.wheel(0, 120)
        pg.wait_for_timeout(40)
    pg.wait_for_timeout(400)
    s1 = pg.evaluate(STATE)
    check("scroll zooms out", s1["dist"] > s0["dist"] * 2, f"view distance {s0['dist']:.0f} -> {s1['dist']:.0f} m")
    check("manual move switches to Free", s1["mode"] == "free", f"mode {s1['mode']}")
    pg.locator("#stage").screenshot(path=str(out / "1_zoomed_out.png"))

    # zoom towards a point left of centre: the target should shift towards it
    for _ in range(4):
        pg.mouse.move(cx - 300, cy + 100)
        pg.mouse.wheel(0, -120)
        pg.wait_for_timeout(40)
    pg.wait_for_timeout(300)
    s2 = pg.evaluate(STATE)
    moved = sum((a - b) ** 2 for a, b in zip(s2["tgt"], s1["tgt"])) ** 0.5
    check("scroll zooms in towards the cursor", s2["dist"] < s1["dist"] and moved > 1, f"distance {s1['dist']:.0f} -> {s2['dist']:.0f} m, pivot moved {moved:.0f} m")

    # right-drag pans the whole view
    pg.mouse.move(cx, cy)
    pg.mouse.down(button="right")
    pg.mouse.move(cx + 200, cy + 120, steps=12)
    pg.mouse.up(button="right")
    pg.wait_for_timeout(400)
    s3 = pg.evaluate(STATE)
    dcam = sum((a - b) ** 2 for a, b in zip(s3["cam"], s2["cam"])) ** 0.5
    dtgt = sum((a - b) ** 2 for a, b in zip(s3["tgt"], s2["tgt"])) ** 0.5
    check("right-drag moves the view (camera and pivot together)", dcam > 5 and abs(dcam - dtgt) < 0.25 * dcam + 1,
          f"camera moved {dcam:.0f} m, pivot {dtgt:.0f} m")

    # arrow keys move the view up
    pg.locator("#view").click(position={"x": 30, "y": box["height"] / 2})
    y0 = pg.evaluate(STATE)["cam"][1]
    pg.keyboard.down("ArrowUp"); pg.wait_for_timeout(600); pg.keyboard.up("ArrowUp")
    pg.wait_for_timeout(200)
    y1 = pg.evaluate(STATE)["cam"][1]
    check("up-arrow key moves the view up", y1 > y0 + 3, f"camera height {y0:.0f} -> {y1:.0f} m")
    pg.locator("#stage").screenshot(path=str(out / "2_moved.png"))

    # on-screen pad: hold "left", then zoom out button
    xL0 = pg.evaluate(STATE)
    pg.locator('#pad [data-move="left"]').hover()
    pg.mouse.down(); pg.wait_for_timeout(500); pg.mouse.up()
    xL1 = pg.evaluate(STATE)
    dl = sum((a - b) ** 2 for a, b in zip(xL1["cam"], xL0["cam"])) ** 0.5
    check("pad arrow moves the view", dl > 3, f"camera moved {dl:.0f} m")
    d0 = xL1["dist"]
    pg.locator('#pad [data-move="out"]').hover()
    pg.mouse.down(); pg.wait_for_timeout(500); pg.mouse.up()
    d1 = pg.evaluate(STATE)["dist"]
    check("pad zoom-out button zooms out", d1 > d0 * 1.3, f"distance {d0:.0f} -> {d1:.0f} m")

    # double-click aims the pivot at the clicked spot
    t0 = pg.evaluate(STATE)["tgt"]
    pg.mouse.dblclick(cx + 150, cy + 60)
    pg.wait_for_timeout(700)
    t1 = pg.evaluate(STATE)["tgt"]
    da = sum((a - b) ** 2 for a, b in zip(t1, t0)) ** 0.5
    check("double-click re-aims the pivot", da > 1, f"pivot moved {da:.0f} m")

    # reset returns to Auto
    pg.locator("#reset-view").click()
    pg.wait_for_timeout(400)
    check("reset button returns to Auto", pg.evaluate(STATE)["mode"] == "auto", pg.evaluate(STATE)["mode"])

    # Follow: rotate around the booster, then play: the camera keeps up with it
    pg.locator('#cams button[data-cam="follow"]').click()
    pg.locator("#scrub").evaluate("(el) => { el.value = 100; el.dispatchEvent(new Event('input')); }")
    pg.wait_for_timeout(1200)
    pg.mouse.move(cx, cy); pg.mouse.down(); pg.mouse.move(cx + 160, cy, steps=10); pg.mouse.up()
    pg.wait_for_timeout(1800)     # let the drag's inertia (damping) settle
    f0 = pg.evaluate(STATE)
    check("dragging in Follow stays in Follow", f0["mode"] == "follow", f0["mode"])
    pg.locator("#play").click(); pg.wait_for_timeout(2500); pg.locator("#play").click()
    f1 = pg.evaluate(STATE)
    rel0 = [a - b for a, b in zip(f0["cam"], f0["tgt"])]
    rel1 = [a - b for a, b in zip(f1["cam"], f1["tgt"])]
    drop = f0["tgt"][1] - f1["tgt"][1]
    keep = sum((a - b) ** 2 for a, b in zip(rel0, rel1)) ** 0.5
    check("Follow tracks the booster and keeps your angle", drop > 50 and keep < 2,
          f"booster pivot fell {drop:.0f} m in {f1['t'] - f0['t']:.1f} s; camera offset changed {keep:.2f} m")
    pg.locator("#stage").screenshot(path=str(out / "3_follow.png"))
    print("page errors:", errors or "none")
    b.close()
print(f"{sum(results)}/{len(results)} checks passed; screenshots in {out}")
