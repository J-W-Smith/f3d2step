"""Minimal STEP reader used by the tests to compare our output with Fusion's own exports."""
import re
from collections import Counter


def parse(path):
    text = open(path, encoding="latin1").read()
    data = text.split("DATA;", 1)[1].rsplit("ENDSEC;", 1)[0]
    data = re.sub(r"/\*.*?\*/", "", data, flags=re.S)
    ents = {}
    for m in re.finditer(r"#(\d+)\s*=\s*(.*?);\s*(?=#\d+\s*=|$)", data, re.S):
        body = " ".join(m.group(2).split())
        mm = re.match(r"([A-Z0-9_]+)\s*\((.*)\)$", body)
        ents[int(m.group(1))] = (mm.group(1), _args(mm.group(2))) if mm else ("COMPLEX", body)
    return ents


def _args(s):
    out, stack, tok, in_str = [], [], "", False
    cur = out
    for ch in s:
        if in_str:
            tok += ch
            in_str = ch != "'"
        elif ch == "'":
            in_str = True
            tok += ch
        elif ch == "(":
            new = []
            cur.append(new)
            stack.append(cur)
            cur = new
        elif ch == ")":
            if tok.strip():
                cur.append(tok.strip())
            tok = ""
            cur = stack.pop()
        elif ch == ",":
            if tok.strip():
                cur.append(tok.strip())
            tok = ""
        else:
            tok += ch
    if tok.strip():
        cur.append(tok.strip())
    return out


def ref(x):
    return int(x[1:])


def summary(path):
    """Faces as (signature, orientation, vertices), vertex set, and edges not shared correctly."""
    E = parse(path)

    def pt(i):
        return tuple(round(float(v), 3) + 0.0 for v in E[i][1][1])

    faces, uses, verts = [], Counter(), set()
    for k, (t, a) in E.items():
        if t == "VERTEX_POINT":
            verts.add(pt(ref(a[1])))
        if t != "ADVANCED_FACE":
            continue
        same = a[3] == ".T."
        st, sa = E[ref(a[2])]
        vs = set()
        for b in a[1]:
            for oe in E[ref(E[ref(b)][1][1])][1][1]:
                o = E[ref(oe)][1]
                ec = ref(o[3])
                uses[(ec, o[4])] += 1
                ea = E[ec][1]
                vs |= {pt(ref(E[ref(ea[1])][1][1])), pt(ref(E[ref(ea[2])][1][1]))}
        if st == "PLANE":
            ax = E[ref(sa[1])][1]
            z = tuple(float(v) for v in E[ref(ax[2])][1][1])
            sig, orient = ("PLANE",), tuple(round(c if same else -c, 6) + 0.0 for c in z)
        elif st in ("CYLINDRICAL_SURFACE", "CONICAL_SURFACE", "SPHERICAL_SURFACE"):
            sig, orient = (st, round(float(sa[2]), 3)), same
        elif st == "TOROIDAL_SURFACE":
            sig, orient = (st, round(float(sa[2]), 3), round(float(sa[3]), 3)), same
        elif st == "OFFSET_SURFACE":
            sig, orient = (st, round(float(sa[2]), 4)), same
        else:
            sig, orient = (st,), same
        faces.append((sig, orient, frozenset(vs)))
    bad = sorted({ec for (ec, o), c in uses.items()
                  if c != 1 or uses.get((ec, ".F." if o == ".T." else ".T."), 0) != 1})
    return faces, verts, bad
