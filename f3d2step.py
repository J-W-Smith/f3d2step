"""f3d2step - prototype converter from Fusion .f3d archives to STEP (AP214).

Reads the final B-rep bodies stored as Autodesk ShapeManager (ACIS-style)
binary in Breps.BlobParts/*.smbh and writes them as MANIFOLD_SOLID_BREPs.

Supported geometry: plane, cylinder/cone, sphere, torus surfaces;
line, circle/ellipse curves. Spline surfaces/curves are reported, not converted.

Usage: python f3d2step.py input.f3d [output.step]
"""
import math
import struct
import sys
import zipfile
from datetime import datetime
from pathlib import Path

CM_TO_MM = 10.0  # Fusion stores model geometry in centimetres


# ---------------------------------------------------------------- SAB reader

class Rec:
    __slots__ = ("idx", "type", "f")

    def __init__(self, idx, type_, fields):
        self.idx, self.type, self.f = idx, type_, fields

    def ptr(self, i):
        return self.f[i][1]


def read_sab(data):
    """Return entity records (index -> Rec) of the main, current-state section."""
    if not data.startswith(b"ASM BinaryFile4"):
        raise ValueError("not an ASM binary B-rep stream")
    p = 15 + 16  # magic + 4 int32 header fields
    n = len(data)
    records, cur = [], []

    def take(fmt):
        nonlocal p
        v = struct.unpack_from(fmt, data, p)
        p += struct.calcsize(fmt)
        return v

    while p < n:
        t = data[p]
        p += 1
        if t in (0x07, 0x0D, 0x0E, 0x12):
            ln = data[p]; p += 1
            s = data[p:p + ln].decode("latin1"); p += ln
            kind = {0x07: "str", 0x0D: "id", 0x0E: "sub", 0x12: "str"}[t]
            cur.append((kind, s))
        elif t == 0x08:
            ln, = take("<H"); cur.append(("str", data[p:p + ln].decode("latin1"))); p += ln
        elif t == 0x09:
            ln, = take("<I"); cur.append(("str", data[p:p + ln].decode("latin1"))); p += ln
        elif t == 0x04:
            cur.append(("int", take("<i")[0]))
        elif t == 0x0C:
            cur.append(("ptr", take("<i")[0]))
        elif t == 0x15:
            cur.append(("enum", take("<i")[0]))
        elif t == 0x06:
            cur.append(("dbl", take("<d")[0]))
        elif t in (0x13, 0x14):
            cur.append(("vec", take("<3d")))
        elif t == 0x16:
            cur.append(("vec2", take("<2d")))
        elif t == 0x0A:
            cur.append(("bool", False))
        elif t == 0x0B:
            cur.append(("bool", True))
        elif t in (0x0F, 0x10):
            cur.append(("brace", t == 0x0F))
        elif t == 0x11:
            # record 0 also carries the file preamble (product strings, unit
            # scale, tolerances) ahead of 'asmheader'; _name() skips past it
            records.append(cur)
            cur = []
            if records and _name(records[-1]).startswith(("Begin-of", "End-of")):
                break
        else:
            raise ValueError("unknown tag 0x%02x at byte %d" % (t, p - 1))
        if cur and _name(cur).startswith(("Begin-of", "End-of")):
            break

    out = {}
    for i, toks in enumerate(records):
        name = _name(toks)
        if name.startswith(("Begin-of", "End-of")):
            break
        k = next(j for j, x in enumerate(toks) if x[0] == "id" and x[1] == name.split("-")[-1])
        out[i] = Rec(i, name, toks[k + 1:])
    out["subtypes"] = _subtype_table(out)
    return out


def _subtype_table(recs):
    """Number every '{ type ... }' block in write order; '{ ref N }' points back here."""
    table = []
    for i in sorted(k for k in recs if isinstance(k, int)):
        f = recs[i].f
        stack = []
        for j, tok in enumerate(f):
            if tok == ("brace", True):
                is_ref = j + 1 < len(f) and f[j + 1] == ("id", "ref")
                if is_ref:
                    stack.append(None)
                else:
                    stack.append(len(table))
                    table.append(None)
                stack[-1] = (stack[-1], j)
            elif tok == ("brace", False):
                num, start = stack.pop()
                if num is not None:
                    table[num] = f[start + 1:j]
    return table


def block_at(f, i):
    """Tokens inside the brace block that opens at f[i]."""
    depth = 0
    for j in range(i, len(f)):
        if f[j] == ("brace", True):
            depth += 1
        elif f[j] == ("brace", False):
            depth -= 1
            if depth == 0:
                return f[i + 1:j]
    raise ValueError("unbalanced subtype block")


def resolve(block, table):
    while block and block[0] == ("id", "ref"):
        block = table[block[1][1]]
    return block


def _name(toks):
    parts = []
    for kind, v in toks:
        if kind == "sub":
            parts.append(v + "-")
        elif kind == "id":
            parts.append(v)
            break
        elif parts:
            break
    return "".join(parts)


# ------------------------------------------------------------- vector utils

def v_scale(v, s): return tuple(c * s for c in v)
def v_len(v): return math.sqrt(sum(c * c for c in v))
def v_unit(v):
    l = v_len(v)
    if l == 0:
        raise ValueError("zero-length direction")
    return tuple(c / l for c in v)
def v_neg(v): return tuple(-c for c in v)


# ------------------------------------------------------------- STEP writer

def r(x):
    s = "%.15G" % x
    if s in ("-0", "0"):
        return "0."
    if "." not in s:
        s = s.replace("E", ".E", 1) if "E" in s else s + "."
    return s


class Step:
    def __init__(self):
        self.lines, self.n = [], 0
        self.cache = {}

    def add(self, text):
        self.n += 1
        self.lines.append("#%d=%s;" % (self.n, text))
        return "#%d" % self.n

    def point(self, xyz):
        return self.add("CARTESIAN_POINT('',(%s))" % ",".join(r(c) for c in xyz))

    def dirn(self, v):
        return self.add("DIRECTION('',(%s))" % ",".join(r(c) for c in v_unit(v)))

    def axis(self, origin, z, x):
        z = v_unit(z)
        x = v_unit(x)
        # make x exactly perpendicular to z
        d = sum(a * b for a, b in zip(x, z))
        x = v_unit(tuple(a - d * b for a, b in zip(x, z)))
        return self.add("AXIS2_PLACEMENT_3D('',%s,%s,%s)" % (self.point(origin), self.dirn(z), self.dirn(x)))


# ------------------------------------------------------------- conversion

class Unsupported(Exception):
    pass


def any_perp(z):
    z = v_unit(z)
    a = (1.0, 0.0, 0.0) if abs(z[0]) < 0.9 else (0.0, 1.0, 0.0)
    d = sum(p * q for p, q in zip(a, z))
    return v_unit(tuple(p - d * q for p, q in zip(a, z)))


def skip_interval(f, i):
    """ACIS bounds: T = unbounded, F followed by a value = bounded."""
    for _ in range(2):
        if f[i] == ("bool", True):
            i += 1
        else:
            i += 2
    return i


def _num(tok):
    if tok[0] not in ("int", "dbl"):
        raise ValueError("expected number, got %r" % (tok,))
    return float(tok[1])


def _depth0(block):
    depth = 0
    for j, tok in enumerate(block):
        if tok == ("brace", True):
            depth += 1
        elif tok == ("brace", False):
            depth -= 1
        elif depth == 0:
            yield j, tok


def _knots(block, j, count):
    vals, mults = [], []
    for _ in range(count):
        vals.append(_num(block[j])); mults.append(int(block[j + 1][1])); j += 2
    # ACIS stores end multiplicity = degree; STEP wants degree + 1
    mults[0] += 1
    mults[-1] += 1
    return vals, mults, j


def bs3_surface(block):
    """Find the B-spline approximation stored in an ACIS spline-surface block."""
    for j, tok in _depth0(block):
        if tok[0] == "id" and tok[1] in ("nubs", "nurbs"):
            sig = [x[0] for x in block[j + 1:j + 9]]
            if sig != ["int", "int", "enum", "enum", "enum", "enum", "int", "int"]:
                continue
            rational = tok[1] == "nurbs"
            du, dv = block[j + 1][1], block[j + 2][1]
            nku, nkv = block[j + 7][1], block[j + 8][1]
            ku, mu, k = _knots(block, j + 9, nku)
            kv, mv, k = _knots(block, k, nkv)
            nu, nv = sum(mu) - du - 1, sum(mv) - dv - 1
            per = 4 if rational else 3
            pts = [[None] * nv for _ in range(nu)]
            wts = [[1.0] * nv for _ in range(nu)]
            for iv in range(nv):          # ACIS lists u fastest within each v row
                for iu in range(nu):
                    vals = [_num(block[k + c]) for c in range(per)]
                    k += per
                    pts[iu][iv] = tuple(vals[:3])
                    if rational:
                        wts[iu][iv] = vals[3]
            return du, dv, pts, wts, (ku, mu), (kv, mv), rational
    if any(t == ("id", "nullbs") for _, t in _depth0(block)):
        raise Unsupported("spline surface without stored approximation")
    raise Unsupported("spline surface data not found")


def bs3_curve(block):
    for j, tok in _depth0(block):
        if tok[0] == "id" and tok[1] in ("nubs", "nurbs"):
            sig = [x[0] for x in block[j + 1:j + 4]]
            if sig != ["int", "enum", "int"]:
                continue
            rational = tok[1] == "nurbs"
            deg, nk = block[j + 1][1], block[j + 3][1]
            kn, mu, k = _knots(block, j + 4, nk)
            n = sum(mu) - deg - 1
            per = 4 if rational else 3
            pts, wts = [], []
            for _ in range(n):
                vals = [_num(block[k + c]) for c in range(per)]
                k += per
                pts.append(tuple(vals[:3]))
                wts.append(vals[3] if rational else 1.0)
            return deg, pts, wts, (kn, mu), rational
    raise Unsupported("intcurve data not found")


def _full_knots(vals, mults):
    return [v for v, m in zip(vals, mults) for _ in range(m)]


def _basis(knots, deg, n, t):
    """Cox-de Boor: values of all n basis functions at t."""
    t = min(max(t, knots[deg]), knots[n])
    N = [1.0 if knots[i] <= t < knots[i + 1] else 0.0 for i in range(len(knots) - 1)]
    if t >= knots[n]:  # end of range belongs to the last non-empty span
        last = max(i for i in range(len(knots) - 1) if knots[i] < knots[i + 1])
        N = [1.0 if i == last else 0.0 for i in range(len(knots) - 1)]
    for p in range(1, deg + 1):
        for i in range(len(knots) - 1 - p):
            a = (t - knots[i]) / (knots[i + p] - knots[i]) * N[i] if knots[i + p] > knots[i] else 0.0
            b = ((knots[i + p + 1] - t) / (knots[i + p + 1] - knots[i + 1]) * N[i + 1]
                 if knots[i + p + 1] > knots[i + 1] else 0.0)
            N[i] = a + b
    return N[:n]


def eval_surface(data, u, v):
    du, dv, pts, wts, (ku, mu), (kv, mv), _ = data
    nu, nv = len(pts), len(pts[0])
    Nu = _basis(_full_knots(ku, mu), du, nu, u)
    Nv = _basis(_full_knots(kv, mv), dv, nv, v)
    acc, wsum = [0.0, 0.0, 0.0], 0.0
    for i, a in enumerate(Nu):
        if a:
            for j, b in enumerate(Nv):
                if b:
                    w = a * b * wts[i][j]
                    wsum += w
                    for c in range(3):
                        acc[c] += w * pts[i][j][c]
    return tuple(c / wsum for c in acc)


def surface_normal(data, u, v):
    (ku, _), (kv, _) = data[4], data[5]
    hu, hv = (ku[-1] - ku[0]) * 1e-6, (kv[-1] - kv[0]) * 1e-6
    p = eval_surface(data, u, v)
    pu = [(a - b) / hu for a, b in zip(eval_surface(data, u + hu, v), p)]
    pv = [(a - b) / hv for a, b in zip(eval_surface(data, u, v + hv), p)]
    n = (pu[1] * pv[2] - pu[2] * pv[1], pu[2] * pv[0] - pu[0] * pv[2], pu[0] * pv[1] - pu[1] * pv[0])
    return p, v_unit(n)


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def offset_of(block, table):
    """For an off_spl_sur block return (progenitor block, signed distance in cm, normal_flip),
    worked out by comparing the stored approximation with the progenitor."""
    prog = next((resolve(block_at(block, j), table) for j, t in enumerate(block) if t == ("brace", True)), None)
    if prog is None:
        return None
    approx, base = bs3_surface(block), bs3_surface(prog)
    (ku, _), (kv, _) = approx[4], approx[5]
    dists, flips = [], []
    for fu in (0.2, 0.5, 0.8):
        for fv in (0.25, 0.5, 0.75):
            u = ku[0] + (ku[-1] - ku[0]) * fu
            v = kv[0] + (kv[-1] - kv[0]) * fv
            pb, nb = surface_normal(base, u, v)
            pa, na = surface_normal(approx, u, v)
            d = tuple(a - b for a, b in zip(pa, pb))
            along = _dot(d, nb)
            # the stored surface is only an approximation, so allow some slack
            if v_len(tuple(x - along * y for x, y in zip(d, nb))) > 0.2 * max(abs(along), 1e-9):
                return None  # parameterisations differ: keep the stored approximation
            dists.append(along)
            flips.append(_dot(na, nb) < 0)
    if max(dists) - min(dists) > 0.1 * abs(sum(dists) / len(dists)) or len(set(flips)) != 1:
        return None
    stored = [abs(t[1]) for _, t in _depth0(block) if t[0] == "dbl" and t[1] != 0]
    d = sum(dists) / len(dists)
    exact = next((s for s in stored if abs(s - abs(d)) < 0.05 * abs(d)), abs(d))
    return prog, math.copysign(exact, d), flips[0]


def _reals(xs):
    return "(%s)" % ",".join(r(x) for x in xs)


def _ints(xs):
    return "(%s)" % ",".join(str(x) for x in xs)


def step_bspline_surface(st, data):
    du, dv, pts, wts, (ku, mu), (kv, mv), rational = data
    cp = "(%s)" % ",".join("(%s)" % ",".join(st.point(v_scale(p, CM_TO_MM)) for p in row) for row in pts)
    if not rational:
        return st.add("B_SPLINE_SURFACE_WITH_KNOTS('',%d,%d,%s,.UNSPECIFIED.,.U.,.U.,.F.,%s,%s,%s,%s,.UNSPECIFIED.)"
                      % (du, dv, cp, _ints(mu), _ints(mv), _reals(ku), _reals(kv)))
    w = "(%s)" % ",".join(_reals(row) for row in wts)
    return st.add("( BOUNDED_SURFACE() B_SPLINE_SURFACE(%d,%d,%s,.UNSPECIFIED.,.U.,.U.,.F.) "
                  "B_SPLINE_SURFACE_WITH_KNOTS(%s,%s,%s,%s,.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() "
                  "RATIONAL_B_SPLINE_SURFACE(%s) REPRESENTATION_ITEM('') SURFACE() )"
                  % (du, dv, cp, _ints(mu), _ints(mv), _reals(ku), _reals(kv), w))


def step_bspline_curve(st, data):
    deg, pts, wts, (kn, mu), rational = data
    cp = "(%s)" % ",".join(st.point(v_scale(p, CM_TO_MM)) for p in pts)
    if not rational:
        return st.add("B_SPLINE_CURVE_WITH_KNOTS('',%d,%s,.UNSPECIFIED.,.U.,.F.,%s,%s,.UNSPECIFIED.)"
                      % (deg, cp, _ints(mu), _reals(kn)))
    return st.add("( BOUNDED_CURVE() B_SPLINE_CURVE(%d,%s,.UNSPECIFIED.,.U.,.F.) "
                  "B_SPLINE_CURVE_WITH_KNOTS(%s,%s,.UNSPECIFIED.) CURVE() GEOMETRIC_REPRESENTATION_ITEM() "
                  "RATIONAL_B_SPLINE_CURVE(%s) REPRESENTATION_ITEM('') )" % (deg, cp, _ints(mu), _reals(kn), _reals(wts)))


def spline_ref(st, block):
    """One STEP B-spline per ACIS subtype block, shared by every face that uses it."""
    key = ("bs", id(block))
    if key not in st.cache:
        st.cache[key] = step_bspline_surface(st, bs3_surface(block))
    return st.cache[key]


def convert_surface(st, rec, table):
    """Return (step_ref, normal_flipped)."""
    f, t = rec.f, rec.type
    if t == "spline-surface":
        block = resolve(block_at(f, 4), table)
        flipped = not fwd(f[3])
        if block and block[0] == ("id", "off_spl_sur"):
            off = offset_of(block, table)
            if off:
                prog, dist, nflip = off
                base = spline_ref(st, prog)
                ref_ = st.add("OFFSET_SURFACE('',%s,%s,.F.)" % (base, r(dist * CM_TO_MM)))
                return ref_, flipped != nflip
            print("  note: offset surface kept as stored approximation", file=sys.stderr)
        return spline_ref(st, block), flipped
    if t == "plane-surface":
        root, normal, udir = f[3][1], f[4][1], f[5][1]
        return st.add("PLANE('',%s)" % st.axis(v_scale(root, CM_TO_MM), normal, udir)), False
    if t == "cone-surface":
        center, axis, major, ratio = f[3][1], f[4][1], f[5][1], f[6][1]
        i = skip_interval(f, 7)
        sin_a, cos_a, scale = f[i][1], f[i + 1][1], f[i + 2][1]
        if abs(ratio - 1.0) > 1e-9:
            raise Unsupported("elliptical cylinder/cone (ratio %g)" % ratio)
        radius = v_len(major) * CM_TO_MM
        flipped = cos_a < 0
        if abs(sin_a) < 1e-12:
            pos = st.axis(v_scale(center, CM_TO_MM), axis, major)
            return st.add("CYLINDRICAL_SURFACE('',%s,%s)" % (pos, r(radius))), flipped
        # Unverified against a real file: cone convention.
        semi = math.atan(abs(sin_a / cos_a))
        ax = axis if (sin_a > 0) == (cos_a > 0) else v_neg(axis)
        pos = st.axis(v_scale(center, CM_TO_MM), ax, major)
        return st.add("CONICAL_SURFACE('',%s,%s,%s)" % (pos, r(radius), r(semi))), flipped
    if t == "sphere-surface":  # unverified against a real file
        center, radius, udir, pole = f[3][1], f[4][1], f[5][1], f[6][1]
        pos = st.axis(v_scale(center, CM_TO_MM), pole, udir)
        return st.add("SPHERICAL_SURFACE('',%s,%s)" % (pos, r(abs(radius) * CM_TO_MM))), radius < 0
    if t == "torus-surface":  # unverified against a real file
        center, normal, major_r, minor_r, udir = f[3][1], f[4][1], f[5][1], f[6][1], f[7][1]
        pos = st.axis(v_scale(center, CM_TO_MM), normal, udir)
        return st.add("TOROIDAL_SURFACE('',%s,%s,%s)" % (pos, r(major_r * CM_TO_MM), r(abs(minor_r) * CM_TO_MM))), minor_r < 0
    raise Unsupported(t)


def convert_curve(st, rec, table):
    """Return (step_ref, direction_reversed)."""
    f, t = rec.f, rec.type
    if t == "intcurve-curve":
        block = resolve(block_at(f, 4), table)
        return step_bspline_curve(st, bs3_curve(block)), not fwd(f[3])
    return _analytic_curve(st, rec), False


def _analytic_curve(st, rec):
    f, t = rec.f, rec.type
    if t == "straight-curve":
        root, d = f[3][1], f[4][1]
        vec = st.add("VECTOR('',%s,%s)" % (st.dirn(d), r(CM_TO_MM)))
        return st.add("LINE('',%s,%s)" % (st.point(v_scale(root, CM_TO_MM)), vec))
    if t == "ellipse-curve":
        center, normal, major, ratio = f[3][1], f[4][1], f[5][1], f[6][1]
        pos = st.axis(v_scale(center, CM_TO_MM), normal, major)
        a = v_len(major) * CM_TO_MM
        if abs(ratio - 1.0) < 1e-12:
            return st.add("CIRCLE('',%s,%s)" % (pos, r(a)))
        return st.add("ELLIPSE('',%s,%s,%s)" % (pos, r(a), r(a * ratio)))
    raise Unsupported(t)


def fwd(tok):
    # In these ASM binaries the 0x0B logical encodes "forward" / "single".
    return tok == ("bool", True)


def convert_body(st, recs, body, name):
    shells_out = []
    open_body = False
    cache = {}
    table = recs["subtypes"]

    def once(key, fn):
        if key not in cache:
            cache[key] = fn()
        return cache[key]

    def vertex(vi):
        v = recs[vi]
        pt = recs[[x for x in v.f if x[0] == "ptr"][-1][1]]
        return once(("v", vi), lambda: st.add("VERTEX_POINT('',%s)" % st.point(v_scale(pt.f[3][1], CM_TO_MM))))

    def edge(ei):
        e = recs[ei]
        def mk():
            v0, v1 = vertex(e.ptr(3)), vertex(e.ptr(5))
            cv, rev = once(("c", e.ptr(8)), lambda: convert_curve(st, recs[e.ptr(8)], table))
            same = fwd(e.f[9]) != rev
            return st.add("EDGE_CURVE('',%s,%s,%s,%s)" % (v0, v1, cv, ".T." if same else ".F."))
        return once(("e", ei), mk)

    lump = body.ptr(3)
    while lump != -1:
        L = recs[lump]
        shell = L.ptr(4)
        while shell != -1:
            S = recs[shell]
            faces_out = []
            face = S.ptr(5)
            while face != -1:
                F = recs[face]
                surf, flipped = once(("s", F.ptr(7)), lambda: convert_surface(st, recs[F.ptr(7)], table))
                same = fwd(F.f[8]) != flipped
                bounds = []
                loop = F.ptr(4)
                while loop != -1:
                    Lp = recs[loop]
                    oes, start, ce = [], Lp.ptr(4), Lp.ptr(4)
                    while True:
                        C = recs[ce]
                        if C.ptr(5) in (-1, ce):
                            open_body = True
                        oes.append(st.add("ORIENTED_EDGE('',*,*,%s,%s)" % (edge(C.ptr(6)), ".T." if fwd(C.f[7]) else ".F.")))
                        ce = C.ptr(3)
                        if ce in (start, -1):
                            break
                    el = st.add("EDGE_LOOP('',(%s))" % ",".join(oes))
                    bounds.append(el)
                    loop = Lp.ptr(3)
                kind = "FACE_OUTER_BOUND" if len(bounds) == 1 else "FACE_BOUND"
                fb = [st.add("%s('',%s,.T.)" % (kind, b)) for b in bounds]
                faces_out.append(st.add("ADVANCED_FACE('',(%s),%s,%s)" % (",".join(fb), surf, ".T." if same else ".F.")))
                face = F.ptr(3)
            if faces_out:
                shells_out.append(faces_out)
            shell = S.ptr(3)
        lump = L.ptr(3)

    if body.ptr(5) != -1:
        tf = recs[body.ptr(5)].f
        rows = [x[1] for x in tf if x[0] == "vec"]
        if rows != [(1, 0, 0), (0, 1, 0), (0, 0, 1), (0, 0, 0)]:
            print("  warning: %s has a non-identity transform that is not applied" % name, file=sys.stderr)
    items = []
    for faces in shells_out:
        if open_body:
            sh = st.add("OPEN_SHELL('',(%s))" % ",".join(faces))
            items.append(("surface", sh))
        else:
            sh = st.add("CLOSED_SHELL('',(%s))" % ",".join(faces))
            items.append(("solid", st.add("MANIFOLD_SOLID_BREP('%s',%s)" % (name, sh))))
    return items


def convert(f3d_path, step_path):
    st = Step()
    solids, sheets, problems = [], [], []
    with zipfile.ZipFile(f3d_path) as z:
        # only the design itself; CAM/simulation/aggregation assets keep their own copies
        breps = sorted(n for n in z.namelist()
                       if n.endswith(".smbh") and n.startswith("FusionAssetName[Active]/"))
        if not breps:
            raise SystemExit("no .smbh body files found in %s" % f3d_path)
        for bi, member in enumerate(breps):
            recs = read_sab(z.read(member))
            bodies = [r_ for k_, r_ in recs.items() if isinstance(k_, int) and r_.type == "body"]
            for k, body in enumerate(bodies):
                name = "Body%d" % (len(solids) + len(sheets) + 1)
                try:
                    for kind, ref in convert_body(st, recs, body, name):
                        (solids if kind == "solid" else sheets).append(ref)
                except Unsupported as e:
                    problems.append("%s: unsupported geometry %s" % (member.rsplit("/", 1)[-1], e))

    # units, context, product structure
    mm = st.add("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )")
    rad = st.add("( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) )")
    sr = st.add("( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() )")
    unc = st.add("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(0.01),%s,'DISTANCE_ACCURACY_VALUE','')" % mm)
    ctx = st.add("( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((%s)) "
                 "GLOBAL_UNIT_ASSIGNED_CONTEXT((%s,%s,%s)) REPRESENTATION_CONTEXT('','3D') )" % (unc, mm, rad, sr))
    origin = st.axis((0, 0, 0), (0, 0, 1), (1, 0, 0))
    items = list(solids)
    if sheets:
        items.append(st.add("SHELL_BASED_SURFACE_MODEL('',(%s))" % ",".join(sheets)))
    if not items:
        raise SystemExit("nothing converted: " + "; ".join(problems))
    rep = st.add("ADVANCED_BREP_SHAPE_REPRESENTATION('',(%s,%s),%s)" % (origin, ",".join(items), ctx))
    name = Path(f3d_path).stem
    app = st.add("APPLICATION_CONTEXT('core data for automotive mechanical design processes')")
    st.add("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,%s)" % app)
    pctx = st.add("PRODUCT_CONTEXT('',%s,'mechanical')" % app)
    prod = st.add("PRODUCT('%s','%s','',(%s))" % (name, name, pctx))
    form = st.add("PRODUCT_DEFINITION_FORMATION('','',%s)" % prod)
    dctx = st.add("PRODUCT_DEFINITION_CONTEXT('part definition',%s,'design')" % app)
    pdef = st.add("PRODUCT_DEFINITION('design','',%s,%s)" % (form, dctx))
    pds = st.add("PRODUCT_DEFINITION_SHAPE('','',%s)" % pdef)
    st.add("SHAPE_DEFINITION_REPRESENTATION(%s,%s)" % (pds, rep))

    stamp = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
    header = ("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\n"
              "FILE_NAME('%s','%s',(''),(''),'f3d2step prototype','f3d2step','');\n"
              "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\nENDSEC;\nDATA;\n"
              % (Path(step_path).name, stamp))
    Path(step_path).write_text(header + "\n".join(st.lines) + "\nENDSEC;\nEND-ISO-10303-21;\n")
    return len(solids), len(sheets), problems


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    src = argv[1]
    dst = argv[2] if len(argv) > 2 else str(Path(src).with_suffix(".step"))
    nsolid, nsheet, problems = convert(src, dst)
    print("%s -> %s: %d solid(s), %d sheet(s)" % (src, dst, nsolid, nsheet))
    for p in problems:
        print("  skipped " + p)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
