# f3d2step

Convert Fusion 360 `.f3d` design archives to STEP (AP214) without Fusion installed.

It reads the B-rep geometry stored inside the `.f3d` file and writes it out as a
STEP solid you can open in FreeCAD, other CAD programs or a slicer. Pure Python 3,
standard library only, one file.

> **Status: prototype.** It handles the parts it has been tested on (below). Expect
> unsupported-geometry errors on other designs; bug reports with sample files are welcome.

## Usage

```
python f3d2step.py input.f3d [output.step]
```

If the output path is left out, it writes `input.step` next to the input.

## What works

| Geometry | Written as | Status |
|---|---|---|
| Planes | `PLANE` | exact, verified |
| Cylinders | `CYLINDRICAL_SURFACE` | exact, verified |
| Fillets (rolling-ball on straight edges) | `TOROIDAL_SURFACE` / `CYLINDRICAL_SURFACE` | exact, verified |
| Lofts and other freeform surfaces | `B_SPLINE_SURFACE_WITH_KNOTS` | stored NURBS, verified |
| Shell / thicken walls of a freeform face | `OFFSET_SURFACE` of the base B-spline | exact, verified |
| Lines, circles, ellipses | `LINE`, `CIRCLE`, `ELLIPSE` | exact |
| Intersection curves on freeform faces | `B_SPLINE_CURVE_WITH_KNOTS` | stored approximation |
| Spheres, cones | `SPHERICAL_SURFACE`, `CONICAL_SURFACE` | written, not yet verified |

Every test file is checked against the STEP that Fusion itself exported:
same face count, same surface types and sizes, same vertices, same face
orientation, and every edge shared by exactly two faces.

### Accuracy

- Analytic faces (planes, cylinders, tori) are exact.
- Lofted surfaces come from the B-spline Fusion stores; on the lamp test part
  they are within 0.00035 mm of Fusion's own export.
- Offset walls are written as true offsets, so they are exact.
- Edges where a freeform face meets another face use the curve Fusion stores, which
  is an approximation: on the lamp part, up to 0.02 mm from the exact surface.
  Fine for printing; tighter tolerance work may need a heal step in your CAD tool.

### Known limits

- Only the active design's final bodies are converted (no sketches, history,
  components with transforms, appearances or names beyond the body).
- Spline surface types that don't carry a stored approximation are rejected.
- Not yet tested in a wide range of CAD programs and slicers.

## How it works

An `.f3d` file is a ZIP archive. The final solid bodies live in
`FusionAssetName[Active]/Breps.BlobParts/*.smbh`. Those are binary B-rep files
in the ShapeManager format (an ACIS SAB descendant): a stream of tagged records
(body → lump → shell → face → loop → coedge → edge → vertex, plus surface and
curve records). Units are centimetres.

`f3d2step.py` tokenises that stream, walks the topology, maps each surface and
curve to its STEP equivalent, and writes a `MANIFOLD_SOLID_BREP` (or an open shell
for sheet bodies). Freeform surfaces carry a B-spline in the file, which is
written directly. Offset surfaces are recognised and written as `OFFSET_SURFACE`
of their base.

## Tests

```
python -m unittest discover tests
```

`tests/fixtures/` holds small test designs, each as `.f3d` plus the `.step`
Fusion exported from it: 20 mm and 30 mm cubes, a filleted cube, a cylinder and a
lofted lamp shade with a 1 mm wall.

## Disclaimer

This project is not affiliated with, endorsed by or sponsored by Autodesk, Inc.
Autodesk and Fusion 360 are trademarks of Autodesk, Inc., used here only to
describe the file format this tool reads. The converter was written by examining
files created by the author; it contains no Autodesk code.

## License

[GNU Affero General Public License v3.0](LICENSE)
