#!/usr/bin/env python3
"""
check_separation.py - quick check of an SU2 RANS run for flow separation
and wall resolution, straight from surface_flow.vtu (no ParaView needed).

usage:   python check_separation.py  CASE_DIR_OR_VTU  [more ...]
         e.g. python check_separation.py supersonic_drone/Scratch/aflr3
              python check_separation.py supersonic_drone/Scratch/aflr3/sweep_aoa_*

For each case it prints
  * y+ (area-weighted mean, 95th percentile, max) - want ~1 for SA
  * % of the upper wing / whole aircraft with reversed streamwise skin
    friction (Cf_x < 0) - the footprint of separated flow
and saves separation_check.png next to the .vtu: a top view of the upper
surface coloured by Cf_x (red = reversed/separated) and a y+ map.

Assumes SU2 axes (x downstream, y spanwise, z up) and needs the regular
python with vtk + matplotlib (the ESP python has no vtk).
"""
import sys, os, glob
import numpy as np
import vtk
from vtk.util.numpy_support import vtk_to_numpy as v2n

FUSELAGE_RADIUS = 0.42      # fus:radius in the .csm - wing = |y| beyond this


def load(vtu):
    r = vtk.vtkXMLUnstructuredGridReader(); r.SetFileName(vtu); r.Update()
    g = r.GetOutput()
    # point data -> cell data, so every value comes with a face area
    p2c = vtk.vtkPointDataToCellData(); p2c.SetInputData(g); p2c.Update()
    c = p2c.GetOutput()
    xyz = v2n(g.GetPoints().GetData())
    tris = []
    for i in range(g.GetNumberOfCells()):
        ids = g.GetCell(i).GetPointIds()
        tris.append([ids.GetId(k) for k in range(ids.GetNumberOfIds())][:3])
    tris = np.array(tris)
    cd = c.GetCellData()
    get = lambda n: v2n(cd.GetArray(n)) if cd.HasArray(n) else None
    return xyz, tris, get("Skin_Friction_Coefficient"), get("Y_Plus")


def check(vtu):
    xyz, tris, cf, yp = load(vtu)
    if cf is None:
        print(f"{vtu}: no Skin_Friction_Coefficient - not a viscous (RANS/NS) result")
        return
    a, b, c = xyz[tris[:, 0]], xyz[tris[:, 1]], xyz[tris[:, 2]]
    n = np.cross(b - a, c - a)
    area = 0.5 * np.linalg.norm(n, axis=1)
    cen = (a + b + c) / 3
    # SU2 boundary faces are consistently oriented; make the topmost face point up
    if n[np.argmax(cen[:, 2]), 2] < 0:
        n = -n
    upper_wing = (n[:, 2] > 0) & (np.abs(cen[:, 1]) > FUSELAGE_RADIUS)
    rev = cf[:, 0] < 0

    pct = lambda m: 100 * area[m & rev].sum() / area[m].sum()
    w = area / area.sum()
    order = np.argsort(yp); cum = np.cumsum(w[order])
    yp95 = yp[order][np.searchsorted(cum, 0.95)]
    print(f"{vtu}\n"
          f"   y+         : mean {np.sum(w*yp):6.2f}   95% {yp95:6.2f}   max {yp.max():6.2f}"
          f"   {'OK' if yp95 <= 2 else 'TOO HIGH for SA (want ~1)'}\n"
          f"   Cf_x < 0   : upper wing {pct(upper_wing):5.1f}%   whole aircraft {pct(np.ones_like(rev)):5.1f}%"
          f"   {'-> separated flow present' if pct(upper_wing) > 1 else '-> attached'}")

    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    top = n[:, 2] > 0
    fig, ax = plt.subplots(2, 1, figsize=(11, 7))
    lim = np.percentile(np.abs(cf[top, 0]), 98)
    for k, (val, title, cmap, vmin, vmax) in enumerate([
            (cf[:, 0], "Cf_x on upper surface (red = reversed / separated)", "RdBu", -lim, lim),
            (yp, "y+ on upper surface", "viridis", 0, max(2.0, yp95))]):
        pc = ax[k].tripcolor(xyz[:, 0], xyz[:, 1], tris[top], facecolors=val[top],
                             cmap=cmap, vmin=vmin, vmax=vmax)
        fig.colorbar(pc, ax=ax[k], shrink=0.9)
        ax[k].set_aspect("equal"); ax[k].set_title(title); ax[k].set_xlabel("x"); ax[k].set_ylabel("y")
    fig.tight_layout()
    out = os.path.join(os.path.dirname(os.path.abspath(vtu)), "separation_check.png")
    fig.savefig(out, dpi=130); plt.close(fig)
    print(f"   plot       : {out}")


def main():
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    for arg in sys.argv[1:]:
        for path in sorted(glob.glob(arg)) or [arg]:
            vtu = os.path.join(path, "surface_flow.vtu") if os.path.isdir(path) else path
            if not vtu.endswith(".vtu"):
                continue        # e.g. sweep_aoa_results.csv caught by a sweep_aoa_* glob
            if os.path.isfile(vtu):
                check(vtu)
            else:
                print(f"{path}: no surface_flow.vtu")


if __name__ == "__main__":
    main()
