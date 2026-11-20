"""Convert each fixture and compare the result with the STEP file Fusion exported for it.

Run from the repository root:  python -m unittest discover tests
"""
import sys
import tempfile
import unittest
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

import f3d2step  # noqa: E402
from stepcheck import parse, summary  # noqa: E402

FIXTURES = ROOT / "tests" / "fixtures"
ANALYTIC = ("PLANE", "CYLINDRICAL_SURFACE", "CONICAL_SURFACE", "SPHERICAL_SURFACE", "TOROIDAL_SURFACE")


def convert(name, tmp):
    out = Path(tmp) / (name + ".step")
    f3d2step.convert(str(FIXTURES / (name + ".f3d")), str(out))
    return summary(str(out)), summary(str(FIXTURES / (name + ".step"))), out


class FixtureTests(unittest.TestCase):
    def check(self, name):
        with tempfile.TemporaryDirectory() as tmp:
            (mine, my_verts, my_bad), (fus, fus_verts, _), out = convert(name, tmp)
            self.assertEqual(len(mine), len(fus), "face count")
            self.assertEqual(my_bad, [], "every edge must be shared by two faces in opposite directions")
            # Fusion adds seam vertices on round faces; ours must all appear in its export
            self.assertTrue(my_verts <= fus_verts, "vertex positions")
            pick = lambda faces: Counter((s, o) for s, o, _ in faces if s[0] in ANALYTIC)
            self.assertEqual(pick(mine), pick(fus), "analytic faces: type, size and orientation")
            return mine, Counter(t for t, _ in parse(str(out)).values())

    def test_cube20(self):
        self.check("cube20")

    def test_cube30(self):
        self.check("cube30")

    def test_cube20_fillet(self):
        self.check("cube20_fillet")

    def test_cylinder(self):
        self.check("cylinder")

    def test_lamp_cone(self):
        mine, entity_types = self.check("lamp_cone")
        kinds = Counter(s[0] for s, _, _ in mine)
        self.assertEqual(kinds["B_SPLINE_SURFACE_WITH_KNOTS"], 1, "lofted outer wall")
        self.assertEqual(kinds["OFFSET_SURFACE"], 1, "inner wall written as an exact offset")
        offsets = [s for s, _, _ in mine if s[0] == "OFFSET_SURFACE"]
        self.assertAlmostEqual(abs(offsets[0][1]), 1.0, places=6, msg="1 mm wall")
        # both walls share one B-spline definition
        self.assertEqual(entity_types["B_SPLINE_SURFACE_WITH_KNOTS"], 1)


class ReaderTests(unittest.TestCase):
    def test_rejects_non_asm_data(self):
        with self.assertRaises(ValueError):
            f3d2step.read_sab(b"not a brep")


if __name__ == "__main__":
    unittest.main()
