#!/usr/bin/env python3
"""tpl/NN_name.h (//@sol ... //@stub ... //@end 마커) → quiz.h(stub) + solutions/quiz.h."""
import pathlib, sys
tpl_dir = pathlib.Path(sys.argv[1]); out = pathlib.Path(sys.argv[2])
for tpl in sorted(tpl_dir.glob("*.h")):
    name = tpl.stem
    sol, stub, mode = [], [], "both"
    for line in tpl.read_text().splitlines():
        s = line.strip()
        if s == "//@sol": mode = "sol"; continue
        if s == "//@stub": mode = "stub"; continue
        if s == "//@end": mode = "both"; continue
        if mode in ("both", "sol"): sol.append(line)
        if mode in ("both", "stub"): stub.append(line)
    assert mode == "both", f"{tpl}: unterminated marker"
    (out / name).mkdir(parents=True, exist_ok=True)
    (out / "solutions" / name).mkdir(parents=True, exist_ok=True)
    (out / name / "quiz.h").write_text("\n".join(stub) + "\n")
    (out / "solutions" / name / "quiz.h").write_text("\n".join(sol) + "\n")
    print("generated", name)
