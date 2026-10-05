"""Record the README media from the replay page: an animated GIF of the nominal catch and a
still hero image. Frames are rendered deterministically by scrubbing the replay clock, so the
recording does not depend on how fast the machine renders.

    <python with playwright and pillow> scripts/record_media.py

Writes docs/media/catch.gif and docs/media/hero.jpg. Needs the built page
(scripts/build_replay.py) and a Chromium with WebGL (python -m playwright install chromium).
"""
import io
from pathlib import Path

from PIL import Image
from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parent.parent
PAGE = ROOT / "docs" / "replay" / "index.html"
OUT = ROOT / "docs" / "media"
OUT.mkdir(parents=True, exist_ok=True)
HIDE = ".pad, .hint, .resetbtn, .bigplay { display: none !important }"

GIF_W, GIF_FPS, SIM_STEP = 720, 12, 0.16        # 0.16 s of flight per frame at 12 fps = 1.9x speed


def set_time(pg, t_sec):
    pg.evaluate("""(t) => { const r = window.__replay, s = document.getElementById('scrub');
        s.value = Math.round(t / r.tEnd * 1000); s.dispatchEvent(new Event('input')); }""", t_sec)


with sync_playwright() as p:
    b = p.chromium.launch(headless=True, args=["--ignore-gpu-blocklist", "--enable-gpu", "--use-angle=metal"])
    pg = b.new_page(viewport={"width": 1280, "height": 900}, device_scale_factor=1)
    pg.goto(PAGE.as_uri())
    pg.wait_for_timeout(3500)
    pg.add_style_tag(content=HIDE)
    pg.evaluate("document.querySelector('#fig-replay').scrollIntoView({block: 'start'})")
    pg.locator(".scn").nth(0).click()
    pg.locator('#tod button[data-tod="sunset"]').click()
    pg.locator('#cams button[data-cam="auto"]').click()
    pg.evaluate("window.__replay.finalPass.uniforms.uDither.value = 0")   # per-pixel noise defeats GIF compression
    pg.wait_for_timeout(800)
    t_end = pg.evaluate("window.__replay.tEnd")
    stage = pg.locator("#stage")

    # animated GIF: ignition to two seconds after the catch
    frames, t = [], 0.0
    t_stop = t_end - 2.4
    while t <= t_stop:
        set_time(pg, t)
        pg.wait_for_timeout(70)
        img = Image.open(io.BytesIO(stage.screenshot(type="png"))).convert("RGB")
        frames.append(img.resize((GIF_W, round(img.height * GIF_W / img.width)), Image.LANCZOS))
        t += SIM_STEP
    for _ in range(GIF_FPS):                    # hold the last frame for a second
        frames.append(frames[-1])
    palette_src = Image.new("RGB", (frames[0].width, frames[0].height * 4))
    for k, i in enumerate((0, len(frames) // 3, 2 * len(frames) // 3, len(frames) - 1)):
        palette_src.paste(frames[i], (0, k * frames[0].height))
    pal = palette_src.quantize(colors=255, method=Image.Quantize.MEDIANCUT)
    q = [f.quantize(palette=pal, dither=Image.Dither.NONE) for f in frames]
    q[0].save(OUT / "catch.gif", save_all=True, append_images=q[1:], duration=round(1000 / GIF_FPS), loop=0, optimize=True)
    print(f"catch.gif: {len(frames)} frames, {(OUT / 'catch.gif').stat().st_size / 1e6:.1f} MB")

    # hero still: the arms closing, side-on tower view, at 2x resolution
    pg.close()
    pg = b.new_page(viewport={"width": 1280, "height": 900}, device_scale_factor=2)
    pg.goto(PAGE.as_uri())
    pg.wait_for_timeout(3500)
    pg.add_style_tag(content=HIDE + " .ov, #tags { display: none !important }")
    pg.evaluate("document.querySelector('#fig-replay').scrollIntoView({block: 'start'})")
    pg.locator('#cams button[data-cam="auto"]').click()
    t_end = pg.evaluate("window.__replay.tEnd")
    set_time(pg, t_end * 0.715)
    pg.wait_for_timeout(1500)
    set_time(pg, t_end * 0.715)
    pg.wait_for_timeout(900)
    img = Image.open(io.BytesIO(pg.locator("#stage").screenshot(type="png"))).convert("RGB")
    img = img.resize((1600, round(img.height * 1600 / img.width)), Image.LANCZOS)
    img.save(OUT / "hero.jpg", quality=88, optimize=True, progressive=True)
    print(f"hero.jpg: {img.size}, {(OUT / 'hero.jpg').stat().st_size / 1e3:.0f} kB")
    b.close()
