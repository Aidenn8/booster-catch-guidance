"""Headless browser check of the replay page: console errors, layout, and screenshots of
the 3D view at several moments, camera modes and times of day.

    <python with playwright> scripts/check_replay.py [out_dir]
"""
import json
import sys
from pathlib import Path

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parent.parent
page_path = ROOT / "docs" / "replay" / "index.html"
out = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "work" / "replay_check"
out.mkdir(parents=True, exist_ok=True)

_fl = json.loads((ROOT / "docs" / "replay" / "flights.json").read_text())["flights"]
_eo = _fl[5]["runs"]
_tend = max(r["t"][-1] for r in _eo.values()) + 4.5
CRASH = int((_eo["mpc"]["t"][-1] + 0.9) / _tend * 1000)

SHOTS = [  # (scenario index, scrub 0-1000, camera / action, time of day, name)
    (0, 20, None, "sunset", "nominal_ignition"),
    (0, 420, None, "sunset", "nominal_descent"),
    (0, 640, "tower", "sunset", "nominal_tower_close"),
    (0, 700, None, "sunset", "nominal_arms"),
    (0, 790, None, "sunset", "nominal_caught"),
    (0, 500, "wide", "sunset", "nominal_wide"),
    (0, 560, None, "day", "nominal_day"),
    (0, 640, None, "night", "nominal_night"),
    (0, 760, "tower", "night", "nominal_night_arms"),
    (1, 640, "tower", "sunset", "crosswind_tower"),
    (5, CRASH, "follow0", "sunset", "engine10_mpc_crash"),
    (5, CRASH, "follow0", "night", "engine10_mpc_crash_night"),
]


def pages(pg, prefix, anchors=("", "#guidance", "#res-mc", "#res-eo", "#res-cost", "#lessons", "#references")):
    """Viewport screenshots of the top of the page and of a few sections (the page is too
    tall for one full-page capture)."""
    for i, a in enumerate(anchors):
        if a:
            pg.evaluate(f"document.querySelector('{a}').scrollIntoView({{block: 'start'}})")
        else:
            pg.evaluate("window.scrollTo(0, 0)")
        pg.wait_for_timeout(250)
        pg.screenshot(path=str(out / f"{prefix}_{i}.png"))
    pg.evaluate("window.scrollTo(0, 0)")


def scrub(pg, v):
    pg.locator("#scrub").evaluate(f"(el) => {{ el.value = {v}; el.dispatchEvent(new Event('input')); }}")


with sync_playwright() as p:
    b = p.chromium.launch(headless=True, args=["--ignore-gpu-blocklist", "--enable-gpu", "--use-angle=metal"])
    pg = b.new_page(viewport={"width": 1280, "height": 900}, color_scheme="dark")
    errors = []
    pg.on("console", lambda m: errors.append(m.text) if m.type in ("error", "warning") else None)
    pg.on("pageerror", lambda e: errors.append("PAGEERROR " + str(e)))
    pg.goto(page_path.as_uri())
    pg.wait_for_timeout(3500)
    pages(pg, "page_dark")
    maths = pg.evaluate("document.querySelectorAll('mjx-container').length")
    pg.locator("#stage").screenshot(path=str(out / "start.png"))
    for scn, frac, cam, tod, name in SHOTS:
        pg.locator(".scn").nth(scn).click()
        pg.wait_for_timeout(300)
        pg.locator(f'#tod button[data-tod="{tod}"]').click()
        if cam == "follow0":
            pg.locator("#cards tr").first.click()      # follow the MPC booster
            pg.locator('#cams button[data-cam="auto"]').click()
        elif cam:
            pg.locator(f'#cams button[data-cam="{cam}"]').click()
        scrub(pg, frac)
        pg.wait_for_timeout(1500)
        scrub(pg, frac)
        pg.wait_for_timeout(700)
        pg.locator("#stage").screenshot(path=str(out / f"{name}.png"))
        pg.locator('#cams button[data-cam="auto"]').click()
    overflow = pg.evaluate("document.documentElement.scrollWidth > document.documentElement.clientWidth")
    fps = pg.evaluate("""() => new Promise((res) => { let n = 0; const t0 = performance.now();
        function f(){ n++; if (performance.now() - t0 < 2000) requestAnimationFrame(f); else res(n / 2); } requestAnimationFrame(f); })""")
    print("dark 1280: console:", errors or "none", "| overflow:", overflow, "| fps ~", fps, "| equations typeset:", maths)
    pg.close()
    for scheme, (w, h) in (("light", (1280, 900)), ("light", (400, 860))):
        pg = b.new_page(viewport={"width": w, "height": h}, color_scheme=scheme)
        errs = []
        pg.on("pageerror", lambda e: errs.append(str(e)))
        pg.goto(page_path.as_uri())
        pg.wait_for_timeout(3000)
        pages(pg, f"page_{scheme}_{w}")
        scrub(pg, 760)
        pg.wait_for_timeout(900)
        pg.locator("#fig-replay").scroll_into_view_if_needed()
        pg.screenshot(path=str(out / f"page_{scheme}_{w}_replay.png"))
        overflow = pg.evaluate("document.documentElement.scrollWidth > document.documentElement.clientWidth")
        print(f"{scheme} {w}: page errors:", errs or "none", "| overflow:", overflow)
        pg.close()
    b.close()
print("screenshots in", out)
