"""Headless browser check of the replay page: console errors, layout, and screenshots of
the 3D view at several moments and camera modes.

    <python with playwright> scripts/check_replay.py [out_dir]
"""
import sys
from pathlib import Path

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parent.parent
page_path = ROOT / "docs" / "replay" / "index.html"
out = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "work" / "replay_check"
out.mkdir(parents=True, exist_ok=True)

import json  # noqa: E402

_fl = json.loads((ROOT / "docs" / "replay" / "flights.json").read_text())["flights"]
_eo = _fl[5]["runs"]
_tend = max(r["t"][-1] for r in _eo.values()) + 4.5
CRASH = int((_eo["mpc"]["t"][-1] + 0.9) / _tend * 1000)

SHOTS = [  # (tab index, scrub fraction 0-1000, camera button or None, name)
    (0, 20, None, "nominal_ignition"),
    (0, 420, None, "nominal_descent"),
    (0, 720, None, "nominal_arms"),
    (0, 790, None, "nominal_caught"),
    (0, 500, "overview", "nominal_overview"),
    (0, 640, "tower", "nominal_tower"),
    (5, 640, None, "engine10_mid"),
    (5, 900, "follow", "engine10_crash"),
    (5, CRASH, "mpcfollow", "engine10_mpc_crash"),
    (5, CRASH + 150, "mpcfollow", "engine10_mpc_smoke"),
    (1, 700, None, "crosswind_arms"),
]


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
    pg.screenshot(path=str(out / "page_dark.png"), full_page=True)
    for tab, frac, cam, name in SHOTS:
        pg.locator(".tab").nth(tab).click()
        pg.wait_for_timeout(300)
        if cam == "mpcfollow":
            pg.locator(".card .follow").first.click()   # follow the MPC booster
        elif cam:
            pg.locator(f'#cams button[data-cam="{cam}"]').click()
        scrub(pg, frac)
        pg.wait_for_timeout(1600 if cam else 900)
        scrub(pg, frac)          # re-seek so the director camera snaps to the shot for this moment
        pg.wait_for_timeout(700)
        pg.locator("#stage").screenshot(path=str(out / f"{name}.png"))
        pg.locator('#cams button[data-cam="auto"]').click()
    overflow = pg.evaluate("document.documentElement.scrollWidth > document.documentElement.clientWidth")
    fps = pg.evaluate("""() => new Promise((res) => { let n = 0; const t0 = performance.now();
        function f(){ n++; if (performance.now() - t0 < 2000) requestAnimationFrame(f); else res(n / 2); } requestAnimationFrame(f); })""")
    print("dark 1280: console:", errors or "none", "| overflow:", overflow, "| fps ~", fps)
    pg.close()
    for scheme, (w, h) in (("light", (1280, 900)), ("light", (400, 860))):
        pg = b.new_page(viewport={"width": w, "height": h}, color_scheme=scheme)
        errs = []
        pg.on("pageerror", lambda e: errs.append(str(e)))
        pg.goto(page_path.as_uri())
        pg.wait_for_timeout(3000)
        scrub(pg, 760)
        pg.wait_for_timeout(900)
        pg.screenshot(path=str(out / f"page_{scheme}_{w}.png"), full_page=True)
        overflow = pg.evaluate("document.documentElement.scrollWidth > document.documentElement.clientWidth")
        print(f"{scheme} {w}: page errors:", errs or "none", "| overflow:", overflow)
        pg.close()
    b.close()
print("screenshots in", out)
