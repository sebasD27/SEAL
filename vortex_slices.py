#!/usr/bin/env python3
"""
vortex_slices.py - leading-edge-vortex visuals from an SU2 RANS result
(flow.vtu + surface_flow.vtu), no ParaView needed. For one case it saves:

  vortex_3d[_FIELD].png      aircraft surface coloured by Cp, with cross-flow
                             planes at several x/c_root stations coloured by
                             FIELD, plus contour lines (full aircraft, mirrored)
  vortex_planes[_FIELD].png  one 2-D panel per station (looking downstream):
                             FIELD colour + in-plane velocity arrows
  cp_span.png / .csv         surface Cp across the local span at each station,
                             upper (solid) and lower (dashed) wing surface -
                             the vortex shows up as a suction peak

FIELD (--field) is one of
  omega_x  streamwise vorticity - the vortex core and the feeding shear layer
  Cp       pressure coefficient - low pressure under/inside the vortex
  p0_loss  total-pressure loss 1 - p0/p0_inf - vortex core, separated flow
  Mach     local Mach number - shocks and the flow accelerating round the vortex
The omega_x files keep the plain names (vortex_3d.png, vortex_planes.png).

usage:
  python vortex_slices.py CASE_DIR                         (omega_x + cp_span)
  python vortex_slices.py CASE_DIR --field Cp
  python vortex_slices.py CASE_DIR --field p0_loss --range 0,0.6
  python vortex_slices.py CASE_DIR --wx-lim 10 --cp-range=-1,0.85   (fixed scales
                                                       to compare sweep angles)
  options: --stations 0.3,0.5,0.7,0.9  --zmax 0.75  --no-cp-span
           --exp-station 0.8 [--exp-croot-mm 490 --exp-uinf 136]  -> one station
             drawn like the VFE-2 PIV figures (mm, m/s, omega_x in 1/s)

  other geometry (e.g. the VFE-2 65 deg delta, half model on +y, pure delta,
  sting from x/c_r = 0.66):
  python vortex_slices.py CASE_DIR --croot 25.734 --semispan 12 --taper 0
         --fus-radius 1.65 --fus-x0 16.98 --stations 0.2,0.4,0.6,0.8,0.95 --zmax 4

Assumes SU2 axes (x downstream, y spanwise, z up), the wing apex / root
leading edge at x = 0 and a straight leading edge; which side of y = 0
the half model is on is detected automatically. Defaults are the
superDrone .csm wing: root chord 3.0, taper 0.1, semi-span 2.0,
fuselage radius 0.42 (override with --croot / --taper / --semispan /
--fus-radius / --fus-x0). Values are
non-dimensional (SU2 FREESTREAM_VEL_EQ_ONE), so omega_x is in freestream
velocity per mesh unit. Needs the regular python with vtk, numpy, scipy
and matplotlib (not the ESP python).
"""
import argparse, os, csv
import numpy as np
import vtk
from vtk.util.numpy_support import vtk_to_numpy as v2n, numpy_to_vtk as n2v

vtk.vtkLogger.SetStderrVerbosity(vtk.vtkLogger.VERBOSITY_WARNING)   # hide cutter INFO lines

FIELDS = {
    "omega_x": dict(label="omega_x (streamwise vorticity, non-dim.)", cmap="RdBu_r"),
    "Cp":      dict(label="Cp", cmap="jet", array="Pressure_Coefficient"),
    "p0_loss": dict(label="total-pressure loss  1 - p0/p0_inf", cmap="magma_r"),
    "Mach":    dict(label="Mach", cmap="jet", array="Mach"),
}


def read_vtu(path):
    r = vtk.vtkXMLUnstructuredGridReader(); r.SetFileName(path); r.Update()
    return r.GetOutput()


def add_array(data, name, values):
    a = n2v(np.ascontiguousarray(values), deep=True); a.SetName(name)
    data.GetPointData().AddArray(a)


def total_pressure(data):
    pd = data.GetPointData()
    return v2n(pd.GetArray("Pressure")) * (1 + 0.2 * v2n(pd.GetArray("Mach")) ** 2) ** 3.5


def near_wing(flow, bounds, p0_inf):
    """Cut the volume down to a box around the wing, then compute vorticity
    and total-pressure loss there (cheaper than on the whole domain)."""
    box = vtk.vtkBox(); box.SetBounds(*bounds)
    ex = vtk.vtkExtractGeometry(); ex.SetInputData(flow); ex.SetImplicitFunction(box)
    ex.ExtractInsideOn(); ex.ExtractBoundaryCellsOn(); ex.Update()
    gr = vtk.vtkGradientFilter(); gr.SetInputConnection(ex.GetOutputPort())
    gr.SetInputArrayToProcess(0, 0, 0, vtk.vtkDataObject.FIELD_ASSOCIATION_POINTS, "Velocity")
    gr.ComputeGradientOff(); gr.ComputeVorticityOn(); gr.SetVorticityArrayName("Vorticity")
    gr.Update()
    out = gr.GetOutput()
    add_array(out, "omega_x", v2n(out.GetPointData().GetArray("Vorticity"))[:, 0])
    add_array(out, "p0_loss", 1 - total_pressure(out) / p0_inf)
    return out


def plane_cut(data, x, ylim, zlim):
    """Cross-flow plane at x, clipped to the y/z window (one station)."""
    pl = vtk.vtkPlane(); pl.SetOrigin(x, 0, 0); pl.SetNormal(1, 0, 0)
    cut = vtk.vtkCutter(); cut.SetInputData(data); cut.SetCutFunction(pl); cut.Update()
    box = vtk.vtkBox(); box.SetBounds(x - 1, x + 1, *ylim, *zlim)
    clip = vtk.vtkClipPolyData(); clip.SetInputConnection(cut.GetOutputPort())
    clip.SetClipFunction(box); clip.InsideOutOn(); clip.Update()
    return clip.GetOutput()


def mirrored(data, flip=("omega_x",)):
    """Reflect a half-model piece about y = 0 for a full-aircraft view.
    omega_x is a scalar copy of a pseudovector component, so its sign must
    be flipped by hand on the mirror image (the vortex spins the other way)."""
    rf = vtk.vtkReflectionFilter(); rf.SetInputData(data)
    rf.SetCenter(0.0); rf.SetPlane(vtk.vtkReflectionFilter.USE_Y); rf.CopyInputOff(); rf.Update()
    out = rf.GetOutput()
    for name in flip:
        arr = out.GetPointData().GetArray(name)
        if arr is not None:
            v = -v2n(arr); out.GetPointData().RemoveArray(name); add_array(out, name, v)
    return out


def off_wall_range(planes, surf, min_dist):
    """+/- colour limit for omega_x from points at least min_dist from the
    aircraft surface - the boundary-layer vorticity at the wall is orders of
    magnitude larger than the vortex's and would wash the planes out."""
    gf = vtk.vtkGeometryFilter(); gf.SetInputData(surf); gf.Update()
    dist = vtk.vtkImplicitPolyDataDistance(); dist.SetInput(gf.GetOutput())
    vals = []
    for p in planes:
        pts = v2n(p.GetPoints().GetData()); wx = v2n(p.GetPointData().GetArray("omega_x"))
        keep = np.array([abs(dist.EvaluateFunction(*q)) > min_dist for q in pts])
        vals.append(np.abs(wx[keep]))
    return float(np.percentile(np.concatenate(vals), 99))


def field_array(field):
    return FIELDS[field].get("array", field)


def lut(name, lo, hi):
    t = vtk.vtkLookupTable(); t.SetNumberOfTableValues(256); t.SetRange(lo, hi)
    import matplotlib; cm = matplotlib.colormaps[name]
    for i in range(256):
        t.SetTableValue(i, *cm(i / 255.0))
    t.Build()
    return t


def render_3d(surf, planes, field, rng, cp_rng, out, wing_ctr, scale):
    ren = vtk.vtkRenderer(); ren.SetBackground(1, 1, 1)
    cp_lut = lut("jet", *cp_rng)
    # aircraft surface coloured by Cp, both halves
    for piece in (surf, mirrored(surf, flip=())):
        m = vtk.vtkDataSetMapper(); m.SetInputData(piece)
        m.SetScalarModeToUsePointFieldData(); m.SelectColorArray("Pressure_Coefficient")
        m.SetLookupTable(cp_lut); m.SetScalarRange(*cp_rng)
        a = vtk.vtkActor(); a.SetMapper(m); ren.AddActor(a)
    name = field_array(field)
    f_lut = cp_lut if field == "Cp" else lut(FIELDS[field]["cmap"], *rng)
    if field == "Cp":
        rng = cp_rng
    full = []
    for p in planes:
        gf = vtk.vtkGeometryFilter(); gf.SetInputData(mirrored(p)); gf.Update()
        full += [p, gf.GetOutput()]
    for p in full:
        pm = vtk.vtkPolyDataMapper(); pm.SetInputData(p)
        pm.SetScalarModeToUsePointFieldData(); pm.SelectColorArray(name)
        pm.SetLookupTable(f_lut); pm.SetScalarRange(*rng)
        pa = vtk.vtkActor(); pa.SetMapper(pm)
        pa.GetProperty().LightingOff()     # true colours: no shading greying out the planes
        ren.AddActor(pa)
        p.GetPointData().SetActiveScalars(name)
        ct = vtk.vtkContourFilter(); ct.SetInputData(p); ct.GenerateValues(10, *rng)
        cm_ = vtk.vtkPolyDataMapper(); cm_.SetInputConnection(ct.GetOutputPort()); cm_.ScalarVisibilityOff()
        ca = vtk.vtkActor(); ca.SetMapper(cm_); ca.GetProperty().SetColor(0.15, 0.15, 0.15)
        ca.GetProperty().SetLineWidth(1.0); ren.AddActor(ca)
    bars = [("Cp", cp_lut, 0.02)] + ([] if field == "Cp" else [(field, f_lut, 0.90)])
    for title, t, xpos in bars:
        sb = vtk.vtkScalarBarActor(); sb.SetLookupTable(t); sb.SetTitle(title)
        sb.SetNumberOfLabels(5); sb.SetPosition(xpos, 0.1); sb.SetWidth(0.07); sb.SetHeight(0.8)
        for tp in (sb.GetTitleTextProperty(), sb.GetLabelTextProperty()):
            tp.SetColor(0, 0, 0); tp.ItalicOff(); tp.ShadowOff()
        ren.AddActor2D(sb)
    cam = ren.GetActiveCamera()
    cx, cy, cz = wing_ctr
    # above and behind the aircraft, looking forward/down: apex at the top of the image
    cam.SetFocalPoint(cx, 0.0, cz)
    cam.SetPosition(cx + 1.33 * scale, -0.4 * scale, cz + 2.0 * scale); cam.SetViewUp(-1, 0, 0)
    cam.OrthogonalizeViewUp(); ren.ResetCameraClippingRange(); cam.Zoom(1.15)
    win = vtk.vtkRenderWindow(); win.SetOffScreenRendering(1); win.AddRenderer(ren); win.SetSize(1800, 1150)
    win.Render()
    w2i = vtk.vtkWindowToImageFilter(); w2i.SetInput(win); w2i.ReadFrontBufferOff(); w2i.Update()
    wr = vtk.vtkPNGWriter(); wr.SetFileName(out); wr.SetInputConnection(w2i.GetOutputPort()); wr.Write()


def render_planes(planes, xs, stations, field, rng, out, ylim, zlim):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from scipy.interpolate import griddata
    n = len(planes); ncol = min(n, 2); nrow = -(-n // ncol)
    fig, axs = plt.subplots(nrow, ncol, figsize=(7.5 * ncol, 2.9 * nrow), squeeze=False,
                            constrained_layout=True)
    for ax, p, x, s in zip(axs.flat, planes, xs, stations):
        pts = v2n(p.GetPoints().GetData()); pd = p.GetPointData()
        val = v2n(pd.GetArray(field_array(field))); vel = v2n(pd.GetArray("Velocity"))
        tris = []
        for i in range(p.GetNumberOfCells()):
            c = p.GetCell(i)
            if c.GetNumberOfPoints() == 3:
                tris.append([c.GetPointId(k) for k in range(3)])
        tc = ax.tripcolor(pts[:, 1], pts[:, 2], np.array(tris), val, cmap=FIELDS[field]["cmap"],
                          vmin=rng[0], vmax=rng[1], shading="gouraud")
        gy, gz = np.meshgrid(np.linspace(*ylim, 34), np.linspace(*zlim, 13))
        v = griddata(pts[:, 1:], vel[:, 1], (gy, gz)); w = griddata(pts[:, 1:], vel[:, 2], (gy, gz))
        ax.quiver(gy, gz, v, w, color="k", scale=6, width=0.003, alpha=0.7)
        ax.set_xlim(*ylim); ax.set_ylim(*zlim); ax.set_aspect("equal")
        ax.set_title(f"x/c_r = {s:g}  (x = {x:.2f})"); ax.set_xlabel("y (span)"); ax.set_ylabel("z")
    for ax in list(axs.flat)[n:]:
        ax.set_visible(False)
    fig.colorbar(tc, ax=axs, shrink=0.8, label=FIELDS[field]["label"])
    fig.suptitle(f"Cross-flow planes looking downstream: colour = {field}, arrows = in-plane velocity")
    fig.savefig(out, dpi=130); plt.close(fig)


def cp_span(surf, xs, stations, yle, fus_radius, fus_x0, out_png, out_csv):
    """Surface Cp across the local span at each station: cut the aircraft
    surface with the plane x = const, split the cut into upper/lower wing
    surface by the face normal, and plot -Cp against eta = y / y_LE(x)."""
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    gf = vtk.vtkGeometryFilter(); gf.SetInputData(surf); gf.Update()
    nf = vtk.vtkPolyDataNormals(); nf.SetInputConnection(gf.GetOutputPort())
    nf.ComputeCellNormalsOn(); nf.ComputePointNormalsOff(); nf.SplittingOff()
    nf.ConsistencyOff(); nf.AutoOrientNormalsOff(); nf.Update()
    poly = nf.GetOutput()
    # SU2 faces are consistently oriented; make the topmost face's normal point up
    nrm = v2n(poly.GetCellData().GetArray("Normals")).copy()
    cc = vtk.vtkCellCenters(); cc.SetInputData(poly); cc.Update()
    if nrm[np.argmax(v2n(cc.GetOutput().GetPoints().GetData())[:, 2]), 2] < 0:
        a = n2v(-nrm, deep=True); a.SetName("Normals")
        poly.GetCellData().RemoveArray("Normals"); poly.GetCellData().AddArray(a)
    rows = []
    n = len(xs); ncol = min(n, 2); nrow = -(-n // ncol)
    fig, axs = plt.subplots(nrow, ncol, figsize=(7.0 * ncol, 3.0 * nrow), squeeze=False,
                            constrained_layout=True)
    for ax, x, s in zip(axs.flat, xs, stations):
        pl = vtk.vtkPlane(); pl.SetOrigin(x, 0, 0); pl.SetNormal(1, 0, 0)
        cut = vtk.vtkCutter(); cut.SetInputData(poly); cut.SetCutFunction(pl); cut.Update()
        c = cut.GetOutput(); pts = v2n(c.GetPoints().GetData()) if c.GetNumberOfPoints() else None
        if pts is None:
            ax.set_visible(False); continue
        cp = v2n(c.GetPointData().GetArray("Pressure_Coefficient"))
        nz = v2n(c.GetCellData().GetArray("Normals"))[:, 2]
        y_le = yle(x)
        data = {"upper": [], "lower": []}
        for i in range(c.GetNumberOfCells()):
            ids = [c.GetCell(i).GetPointId(k) for k in range(c.GetCell(i).GetNumberOfPoints())]
            ym, zm, cpm = pts[ids, 1].mean(), pts[ids, 2].mean(), cp[ids].mean()
            if (x >= fus_x0 and abs(ym) < fus_radius) or abs(nz[i]) < 0.3:   # body, tip/edge faces
                continue
            side = "upper" if nz[i] > 0 else "lower"
            data[side].append((ym / y_le, ym, zm, cpm))
        for side, style in (("upper", "-"), ("lower", "--")):
            d = sorted(data[side])
            if d:
                ax.plot([r[0] for r in d], [-r[3] for r in d], style, lw=1.6, label=side)
                rows += [(s, x, side) + r for r in d]
        ax.set_xlim(0, 1.02); ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
        ax.set_title(f"x/c_r = {s:g}  (x = {x:.2f})")
        ax.set_xlabel("eta = y / y_LE  (0 = centreline, 1 = leading edge)"); ax.set_ylabel("-Cp")
    for ax in list(axs.flat)[n:]:
        ax.set_visible(False)
    fig.suptitle("Spanwise surface pressure: vortex = suction peak on the upper surface")
    fig.savefig(out_png, dpi=130); plt.close(fig)
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f); w.writerow(["x_over_cr", "x", "surface", "eta", "y", "z", "Cp"])
        w.writerows(rows)


def render_exp_style(near, surf, s_xc, croot, yle_x, side, exp_croot_mm, exp_uinf,
                     levels, out):
    """One cross-flow plane drawn like the VFE-2 PIV figures: the CFD model
    scaled to the experiment's root chord (mm), velocities in m/s and
    omega_x in 1/s using the experiment's freestream speed, a fixed
    rainbow level set, dense in-plane arrows and the wing section outline.
    SU2 is non-dimensional (V_inf = 1, lengths in mesh units), so
    omega_phys = omega_su2 * U_inf / (mesh unit in m)."""
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.tri as mtri
    x = s_xc * croot
    mm = exp_croot_mm / croot                      # mm per mesh unit at experiment scale
    ys = abs(yle_x)
    raw = plane_cut(near, x, tuple(sorted((side * 0.42 * ys, side * 1.02 * ys))),
                    (-0.07 * ys, 0.34 * ys))
    tf = vtk.vtkTriangleFilter(); tf.SetInputData(raw)
    cl = vtk.vtkCleanPolyData(); cl.SetInputConnection(tf.GetOutputPort()); cl.Update()
    plane = cl.GetOutput()                         # merged duplicate points from the cutter
    pts = v2n(plane.GetPoints().GetData()); pd = plane.GetPointData()
    tris = np.array([[plane.GetCell(i).GetPointId(k) for k in range(3)]
                     for i in range(plane.GetNumberOfCells()) if plane.GetCell(i).GetNumberOfPoints() == 3])
    y_mm, z_mm = side * pts[:, 1] * mm, pts[:, 2] * mm            # outboard positive, like the paper
    wx = side * v2n(pd.GetArray("omega_x")) * exp_uinf / (mm * 1e-3)   # mirror flips the sign on -y
    vel = v2n(pd.GetArray("Velocity")) * exp_uinf
    tri = mtri.Triangulation(y_mm, z_mm, tris)

    fig, ax = plt.subplots(figsize=(8.2, 5.2))
    cf = ax.tricontourf(tri, wx, levels=levels, cmap="jet", extend="both")
    gy, gz = np.meshgrid(np.linspace(y_mm.min(), y_mm.max(), 48), np.linspace(z_mm.min(), z_mm.max(), 30))
    from scipy.interpolate import griddata
    from scipy.spatial import cKDTree
    v = griddata((y_mm, z_mm), side * vel[:, 1], (gy, gz))
    w = griddata((y_mm, z_mm), vel[:, 2], (gy, gz))
    # no arrows inside the wing section: grid points far from any flow point
    far = cKDTree(np.column_stack([y_mm, z_mm])).query(np.column_stack([gy.ravel(), gz.ravel()]))[0]
    hole = (far > 0.012 * ys * mm).reshape(gy.shape)
    v[hole] = np.nan; w[hole] = np.nan
    q = ax.quiver(gy, gz, v, w, color="k", scale_units="xy", angles="xy",
                  scale=100 / (0.06 * (y_mm.max() - y_mm.min())), width=0.0018)
    ax.quiverkey(q, 0.88, 1.04, 100, "100 m/s", labelpos="E", coordinates="axes")
    # wing section outline from the aircraft surface
    gf = vtk.vtkGeometryFilter(); gf.SetInputData(surf); gf.Update()
    pl = vtk.vtkPlane(); pl.SetOrigin(x, 0, 0); pl.SetNormal(1, 0, 0)
    cut = vtk.vtkCutter(); cut.SetInputConnection(gf.GetOutputPort()); cut.SetCutFunction(pl); cut.Update()
    c = cut.GetOutput(); cp_ = v2n(c.GetPoints().GetData())
    for i in range(c.GetNumberOfCells()):
        ids = [c.GetCell(i).GetPointId(k) for k in range(c.GetCell(i).GetNumberOfPoints())]
        ax.plot(side * cp_[ids, 1] * mm, cp_[ids, 2] * mm, "k-", lw=1.2)
    ax.set_xlim(0.44 * ys * mm, 1.01 * ys * mm); ax.set_ylim(-0.055 * ys * mm, 0.33 * ys * mm)
    ax.set_aspect("equal"); ax.set_xlabel("y, mm"); ax.set_ylabel("z, mm")
    ax.text(0.62 * ys * mm, -0.02 * ys * mm, f"x/c_r = {s_xc:g}", fontsize=11)
    fig.colorbar(cf, ax=ax, shrink=0.85, label="omega_x  1/s")
    ax.set_title(f"SU2, scaled to c_r = {exp_croot_mm:g} mm, U_inf = {exp_uinf:g} m/s")
    fig.tight_layout(); fig.savefig(out, dpi=150); plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("case_dir")
    ap.add_argument("--field", default="omega_x", choices=sorted(FIELDS),
                    help="what to colour the cross-flow planes by")
    ap.add_argument("--range", help="fixed colour range for --field, 'lo,hi'")
    ap.add_argument("--wx-lim", type=float, help="shorthand for --range=-L,L with --field omega_x")
    ap.add_argument("--cp-range", help="fixed Cp colour range for the aircraft surface, 'lo,hi'")
    ap.add_argument("--stations", default="0.3,0.4,0.5,0.6,0.7,0.8,0.9",
                    help="x / c_root stations for the planes")
    ap.add_argument("--croot", type=float, default=3.0, help="wing:croot (root LE at x = 0)")
    ap.add_argument("--semispan", type=float, default=2.0, help="wing:span")
    ap.add_argument("--taper", type=float, default=0.1, help="wing:taper (sets the LE line)")
    ap.add_argument("--fus-radius", type=float, default=0.42, help="fus:radius (0 = no body)")
    ap.add_argument("--fus-x0", type=float, default=-1e9,
                    help="x where the body/sting starts (cp_span ignores |y| < fus-radius aft of it)")
    ap.add_argument("--zmin", type=float, default=0.0, help="bottom of the planes (mesh units)")
    ap.add_argument("--zmax", type=float, default=0.75, help="top of the planes (mesh units)")
    ap.add_argument("--no-cp-span", action="store_true", help="skip cp_span.png/.csv")
    ap.add_argument("--exp-station", type=float,
                    help="also draw one station (x/c_r) in the VFE-2 PIV figure style -> "
                         "vortex_exp_style_xc<station>.png")
    ap.add_argument("--exp-croot-mm", type=float, default=490.0,
                    help="experiment model root chord in mm (VFE-2 PIV figure: LE at 183 mm "
                         "at x/c_r = 0.8 -> about 490 mm)")
    ap.add_argument("--exp-uinf", type=float, default=136.0,
                    help="experiment freestream speed, m/s (sets the 1/s and m/s scales)")
    ap.add_argument("--exp-levels", default="-18000,26000,4000",
                    help="omega_x contour levels 'lo,hi,step' in 1/s (paper: -18000..26000)")
    a = ap.parse_args()

    stations = [float(s) for s in a.stations.split(",")]
    xs = [s * a.croot for s in stations]
    print("reading", a.case_dir)
    flow = read_vtu(os.path.join(a.case_dir, "flow.vtu"))
    surf = read_vtu(os.path.join(a.case_dir, "surface_flow.vtu"))
    sy = v2n(surf.GetPoints().GetData())[:, 1]
    side = 1.0 if sy.max() > -sy.min() else -1.0          # half model on +y or -y
    pad, root = 0.075 * a.croot / 1.5, 0.7 * a.fus_radius    # plane margin, inboard edge
    yle = lambda x: side * min(a.semispan, x * a.semispan / (a.croot * (1 - a.taper)))
    win = lambda lo_hi: tuple(sorted(lo_hi))
    ylim = win((side * (a.semispan + pad), side * root)); zlim = (a.zmin, a.zmax)
    print(f"half model on {'+' if side > 0 else '-'}y, semi-span {a.semispan}, root chord {a.croot}")
    X = v2n(flow.GetPoints().GetData())
    far = np.linalg.norm(X - X.mean(0), axis=1) > 0.8 * np.linalg.norm(X - X.mean(0), axis=1).max()
    p0_inf = float(np.median(total_pressure(flow)[far]))
    print("computing vorticity and total-pressure loss near the wing")
    yb = win((side * (a.semispan + 2 * pad), -side * 0.05))
    near = near_wing(flow, (min(xs) - 2 * pad, max(xs) + 2 * pad, *yb,
                            zlim[0] - 2 * pad, zlim[1] + 2 * pad), p0_inf)
    # each plane spans from just outboard of the local leading edge to the fuselage side
    planes = [plane_cut(near, x, win((yle(x) + side * pad, side * root)), zlim) for x in xs]

    cp = v2n(surf.GetPointData().GetArray("Pressure_Coefficient"))
    cp_rng = (tuple(float(v) for v in a.cp_range.split(",")) if a.cp_range
              else (float(np.percentile(cp, 1)), float(np.percentile(cp, 99))))
    if a.range:
        rng = tuple(float(v) for v in a.range.split(","))
    elif a.field == "omega_x":
        lim = a.wx_lim or off_wall_range(planes, surf, 0.05 * a.croot / 3); rng = (-lim, lim)
    else:
        vals = np.concatenate([v2n(p.GetPointData().GetArray(field_array(a.field))) for p in planes])
        rng = (float(np.percentile(vals, 1)), float(np.percentile(vals, 99)))
        if a.field == "p0_loss":
            rng = (0.0, rng[1])
    print(f"{a.field} colour range {rng[0]:.3g}..{rng[1]:.3g}, surface Cp {cp_rng[0]:.2f}..{cp_rng[1]:.2f}")

    tag = "" if a.field == "omega_x" else f"_{a.field}"
    out3d = os.path.join(a.case_dir, f"vortex_3d{tag}.png")
    render_3d(surf, planes, a.field, rng, cp_rng, out3d, (a.croot * 0.55, 0.0, 0.2 * a.croot / 3),
              a.croot)
    print("saved", out3d)
    out2d = os.path.join(a.case_dir, f"vortex_planes{tag}.png")
    render_planes(planes, xs, stations, a.field, rng, out2d, ylim, zlim)
    print("saved", out2d)
    if a.exp_station is not None:
        lo, hi, st = (float(v) for v in a.exp_levels.split(","))
        out = os.path.join(a.case_dir, f"vortex_exp_style_xc{a.exp_station:g}.png")
        render_exp_style(near, surf, a.exp_station, a.croot, yle(a.exp_station * a.croot), side,
                         a.exp_croot_mm, a.exp_uinf, np.arange(lo, hi + st / 2, st), out)
        print("saved", out)
    if not a.no_cp_span:
        png, csv_path = (os.path.join(a.case_dir, f"cp_span.{e}") for e in ("png", "csv"))
        cp_span(surf, xs, stations, yle, a.fus_radius, a.fus_x0, png, csv_path)
        print("saved", png, "and", csv_path)


if __name__ == "__main__":
    main()
