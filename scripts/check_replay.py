"""Headless browser check of the replay page: console errors and screenshots.

    <python with playwright> scripts/check_replay.py [out_dir]
"""
import sys
from pathlib import Path

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parent.parent
page_path = ROOT / "docs" / "replay" / "index.html"
out = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "work" / "replay_check"
out.mkdir(parents=True, exist_ok=True)

with sync_playwright() as p:
    b = p.chromium.launch(headless=True, args=["--ignore-gpu-blocklist", "--enable-gpu", "--use-angle=metal"])
    for scheme, (w, h) in (("light", (1280, 900)), ("dark", (1280, 900)), ("light", (400, 860))):
        pg = b.new_page(viewport={"width": w, "height": h}, color_scheme=scheme)
        errors = []
        pg.on("console", lambda m: errors.append(m.text) if m.type == "error" else None)
        pg.on("pageerror", lambda e: errors.append(str(e)))
        pg.goto(page_path.as_uri())
        pg.wait_for_timeout(2500)
        tag = f"{scheme}_{w}"
        pg.screenshot(path=str(out / f"{tag}_start.png"), full_page=True)
        # jump to the late-ignition flight and scrub to the catch
        tabs = pg.locator(".tab")
        print(tag, "tabs:", tabs.count())
        tabs.nth(2).click()
        pg.locator("#scrub").evaluate("(el) => { el.value = 700; el.dispatchEvent(new Event('input')); }")
        pg.wait_for_timeout(800)
        pg.screenshot(path=str(out / f"{tag}_late_end.png"), full_page=True)
        pg.locator("#scrub").evaluate("(el) => { el.value = 600; el.dispatchEvent(new Event('input')); }")
        pg.locator("#cams button").nth(1).click()
        pg.wait_for_timeout(1200)
        pg.screenshot(path=str(out / f"{tag}_chase.png"))
        overflow = pg.evaluate("document.documentElement.scrollWidth > document.documentElement.clientWidth")
        print(tag, "console errors:", errors or "none", "| horizontal overflow:", overflow)
        print(tag, "result text:", pg.locator("#mpc-result").inner_text()[:120], "|",
              pg.locator("#baseline-result").inner_text()[:120])
        pg.close()
    b.close()
print("screenshots in", out)
