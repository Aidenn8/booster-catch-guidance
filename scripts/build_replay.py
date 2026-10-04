"""Build the self-contained replay page from the template and docs/replay/flights.json.

    python scripts/build_replay.py [artifact_out.html]

Writes docs/replay/index.html (a full HTML document, for GitHub Pages or opening locally)
and, if a path is given, the same page without the document wrapper (for hosts that add
their own <html>/<head>/<body>).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from boostercatch import ROOT  # noqa: E402

D = ROOT / "docs" / "replay"
page = (D / "template.html").read_text().replace("/*FLIGHTS_JSON*/", (D / "flights.json").read_text())
doc = ('<!doctype html>\n<html lang="en">\n<head>\n<meta charset="utf-8">\n'
       '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">\n'
       '</head>\n<body>\n' + page + '\n</body>\n</html>\n')
(D / "index.html").write_text(doc)
print("wrote", D / "index.html", f"{len(doc) / 1e3:.0f} kB")
if len(sys.argv) > 1:
    Path(sys.argv[1]).parent.mkdir(parents=True, exist_ok=True)
    Path(sys.argv[1]).write_text(page)
    print("wrote", sys.argv[1])
