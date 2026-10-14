"""Build the self-contained replay page from the template, docs/replay/flights.json,
docs/replay/paper.html (the report body; optional) and the figures it references.

    python scripts/build_replay.py [artifact_out.html]

Writes docs/replay/index.html (a full HTML document, for GitHub Pages or opening locally)
and, if a path is given, the same page without the document wrapper (for hosts that add
their own <html>/<head>/<body>). Images referenced as src="img:<name>.png" in the findings
are inlined as data: URIs so the page is a single file.
"""
import base64
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from boostercatch import ROOT  # noqa: E402

D = ROOT / "docs" / "replay"
IMG = ROOT / "docs" / "images"
page = (D / "template.html").read_text().replace("/*FLIGHTS_JSON*/", (D / "flights.json").read_text())
findings = (D / "paper.html").read_text() if (D / "paper.html").exists() else ""


def inline(m):
    data = base64.b64encode((IMG / m.group(1)).read_bytes()).decode()
    return f'src="data:image/png;base64,{data}"'


findings = re.sub(r'src="img:([\w.-]+)"', inline, findings)
page = page.replace("<!--FINDINGS-->", findings)
doc = ('<!doctype html>\n<html lang="en">\n<head>\n<meta charset="utf-8">\n'
       '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">\n'
       '<link rel="icon" href="/favicon.ico" sizes="32x32">\n'
       '<link rel="icon" href="/favicon.svg" type="image/svg+xml">\n'
       '<link rel="apple-touch-icon" href="/apple-touch-icon.png">\n'
       '</head>\n<body>\n' + page + '\n</body>\n</html>\n')
(D / "index.html").write_text(doc)
print("wrote", D / "index.html", f"{len(doc) / 1e3:.0f} kB")
if len(sys.argv) > 1:
    Path(sys.argv[1]).parent.mkdir(parents=True, exist_ok=True)
    Path(sys.argv[1]).write_text(page)
    print("wrote", sys.argv[1])
