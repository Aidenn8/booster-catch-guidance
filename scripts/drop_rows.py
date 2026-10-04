"""Remove a controller family's rows from results files so the resumable runner re-flies them
(used after a controller change that leaves the other controller's results valid).

    python scripts/drop_rows.py mpc results/mc_basin.jsonl [...]
"""
import json
import sys

family = sys.argv[1]
for path in sys.argv[2:]:
    try:
        rows = [json.loads(line) for line in open(path)]
    except FileNotFoundError:
        continue
    keep = [d for d in rows if d["controller"].split("[")[0] != family]
    with open(path, "w") as f:
        for d in keep:
            f.write(json.dumps(d) + "\n")
    print(f"{path}: kept {len(keep)} of {len(rows)}")
