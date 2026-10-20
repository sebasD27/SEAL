###################################################################
#                                                                 #
# aflr_mesh_drone.py -- surface + volume mesh for supersonic_drone.csm
#                        using AFLR4 (surface) then AFLR3 (volume),
#                        against the C-style farfield domain built
#                        by build:cfdMesh in the .csm file.
#
#              Adapted from AFLRtest.py. capsGroup names below
#              ("Farfield", "Aircraft") match exactly what
#              supersonic_drone.csm tags when build:cfdMesh=1.
#
#              All settings are exposed as command-line arguments
#              (with the same defaults this script had hardcoded
#              before) so mesh_gui.py - or you, by hand - can drive
#              it without editing this file each time.
#                                                                 #
###################################################################

import argparse
import csv
import glob
import os
import re
import shutil
import subprocess
import sys

import pyCAPS


def _find_su2_file(analysis_obj, proj_name):
    """AFLR3 writes into its own analysisDir; the exact attribute name
    has varied across pyCAPS versions, so try a couple of likely spots
    before falling back to a search from the current working
    directory."""
    candidates = []
    for attr in ("analysisDir", "analysisDirectory", "workDir"):
        d = getattr(analysis_obj, attr, None)
        if d:
            candidates.append(os.path.join(d, proj_name + ".su2"))
            candidates += glob.glob(os.path.join(d, "*.su2"))

    candidates.append(proj_name + ".su2")
    candidates += glob.glob(os.path.join(os.getcwd(), "**", proj_name + ".su2"), recursive=True)

    for c in candidates:
        if os.path.isfile(c):
            return os.path.abspath(c)

    raise FileNotFoundError(
        f"could not locate {proj_name}.su2 - check the AFLR3 analysis "
        f"directory by hand and rename markers there manually if needed")


def read_mapbc_names(su2_path):
    """
    AFLR3 writes <proj>.mapbc next to the .su2 file: a count line, then
    one "index bcType capsGroup" line per group, in the exact numbering
    CAPS used for BC_N. Returns {index: name}, or {} if there is no
    readable .mapbc.
    """
    mapbc_path = os.path.splitext(su2_path)[0] + ".mapbc"
    names = {}
    try:
        with open(mapbc_path) as f:
            for line in f.readlines()[1:]:
                parts = line.split()
                if len(parts) >= 3 and parts[0].isdigit():
                    names[int(parts[0])] = parts[2]
    except OSError:
        pass
    return names


def rename_su2_markers(su2_path, name_order):
    """
    AFLR3's SU2 writer names markers generically ("BC_1", "BC_2", ...)
    rather than using the real capsGroup names. Rename BC_N using the
    .mapbc file AFLR3 writes alongside the mesh (authoritative, covers
    every capsGroup); if there is none, fall back to name_order[N-1]
    (1-indexed, matching CAPS's own numbering - check your build log's
    "Mapping capsGroup attributes" section).

    AFLR3 also writes a group's triangles and quads (e.g. boundary-layer
    quads on the symmetry plane) as two separate BC_N markers with the
    same N. Markers that end up with the same name are merged into one,
    so SU2 sees each boundary exactly once.

    Any BC_N with no name (e.g. an internal SEND_RECV marker used for
    MPI partitioning) is left unchanged rather than raising.
    """
    names = read_mapbc_names(su2_path)
    source = "mapbc"
    if not names:
        names = {i+1: n for i, n in enumerate(name_order)}
        source = "--su2-marker-order"

    with open(su2_path) as f:
        lines = f.readlines()

    i_nmark = next(i for i, l in enumerate(lines) if l.strip().startswith("NMARK"))
    nmark = int(lines[i_nmark].split("=")[1])

    markers = {}   # name -> element lines, in first-seen order
    renamed = []
    j = i_nmark + 1
    for _ in range(nmark):
        tag = lines[j].split("=", 1)[1].strip()
        n = int(lines[j+1].split("=", 1)[1])
        m = re.match(r"^BC_(\d+)$", tag)
        if m and int(m.group(1)) in names:
            name = names[int(m.group(1))]
            if (int(m.group(1)), name) not in renamed:
                renamed.append((int(m.group(1)), name))
        else:
            name = tag
        markers.setdefault(name, []).extend(lines[j+2:j+2+n])
        j += 2 + n

    out = lines[:i_nmark]
    out.append(f"NMARK= {len(markers)}\n")
    for name, elems in markers.items():
        out.append(f"MARKER_TAG= {name}\n")
        out.append(f"MARKER_ELEMS= {len(elems)}\n")
        out.extend(elems)
    out.extend(lines[j:])

    with open(su2_path, "w") as f:
        f.writelines(out)

    print(f"==> marker names taken from {source}; "
          f"{nmark} marker(s) in, {len(markers)} out: {list(markers)}")
    return renamed


def get_outpmtr(capsProblem, name, fallback):
    """Best-effort fetch of an OUTPMTR from the .csm geometry (e.g.
    wing:MAC, wing:refArea) - falls back to a hardcoded value if the
    name doesn't exist or pyCAPS's accessor pattern differs, rather
    than crashing a whole run over a reference-value lookup."""
    try:
        return capsProblem.geometry.outpmtr[name].value
    except Exception as e:
        print(f"==> could not fetch OUTPMTR '{name}' ({e}); using fallback {fallback}")
        return fallback


def write_su2_config(template_path, out_cfg_path, mesh_filename,
                      mach, aoa, ref_length, ref_area, max_iter,
                      reynolds_number=5e6, moment_x=0.25):
    with open(template_path) as f:
        text = f.read()

    subs = {
        "__MACH_NUMBER__":   str(mach),
        "__AOA__":           str(aoa),
        "__MESH_FILENAME__": mesh_filename,
        "__REF_LENGTH__":    str(ref_length),
        "__REF_AREA__":      str(ref_area),
        "__MAX_ITER__":      str(max_iter),
        # only viscous (RANS) templates like su2_template_wing_viscous.cfg
        # have a __REYNOLDS_NUMBER__ placeholder - harmless no-op replace
        # on inviscid templates that don't
        "__REYNOLDS_NUMBER__": str(reynolds_number),
        # pitching-moment reference x (MAC quarter chord) - only the drone
        # templates have this placeholder; no-op on the others
        "__MOMENT_X__":      str(moment_x),
    }
    for key, val in subs.items():
        text = text.replace(key, val)

    leftover = [line for line in text.splitlines()
                if "__" in line and not line.strip().startswith("%")]
    if leftover:
        raise ValueError("su2_template.cfg has un-substituted placeholder(s):\n  "
                          + "\n  ".join(leftover))

    with open(out_cfg_path, "w") as f:
        f.write(text)


def run_su2(su2_exe, cfg_path, work_dir, mpi_exe=None, mpi_procs=1):
    """Run SU2_CFD, optionally through mpiexec for parallelism, e.g.:
        mpiexec -n 4 SU2_CFD.exe droneCFD.cfg
    Streams output live rather than buffering it all until the end -
    a run can take a long time and you want to see progress (and be
    able to Ctrl-C) as it happens. cfg_path is passed as a full path
    (not just a filename) so this works the same whether or not
    work_dir happens to match where the caller is standing."""
    if mpi_exe:
        cmd = [mpi_exe, "-n", str(mpi_procs), su2_exe, cfg_path]
    else:
        cmd = [su2_exe, cfg_path]

    print(f"==> running: {' '.join(cmd)}")
    print(f"==> working directory: {work_dir}")
    proc = subprocess.Popen(
        cmd,
        cwd=work_dir,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    for line in proc.stdout:
        print(line, end="")
    proc.wait()
    return proc.returncode


def get_last_row_coeffs(history_csv_path):
    """Read the LAST row of a history.csv and pull out CL/CD -
    the converged (or best-so-far, if the run didn't fully converge)
    values for that case. Note: HISTORY_OUTPUT/SCREEN_OUTPUT use the
    keywords LIFT/DRAG to REQUEST these fields, but the actual column
    header SU2 writes to the CSV is "CL"/"CD" (the coefficient
    abbreviations), not "LIFT"/"DRAG" - matching on the wrong name
    here silently returned None for every case regardless of whether
    the run actually succeeded. Returns None for any column that isn't
    present rather than raising, so one bad/short case doesn't kill a
    whole sweep. CM is SU2's "CMy" - the pitching moment in SU2 axes
    (y spanwise), about REF_ORIGIN_MOMENT_*."""
    with open(history_csv_path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        return None, None, None

    last = rows[-1]
    cl = cd = cm = None
    for k, v in last.items():
        name = k.strip().strip('"')
        try:
            val = float(v.strip().strip('"'))
        except (ValueError, AttributeError):
            continue
        if name.upper() == "CL":
            cl = val
        elif name.upper() == "CD":
            cd = val
        elif name.upper() == "CMY":
            cm = val
    return cl, cd, cm


def sanitize_for_dirname(value):
    return str(value).replace(".", "p").replace("-", "neg")


def _get_plt():
    """Import matplotlib lazily (Agg backend - no display needed, this
    runs headless from the SU2 subprocess loop) so a Python without
    matplotlib installed can still mesh/run SU2, just without plots."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        return plt
    except ImportError:
        print("==> matplotlib not installed in this Python - skipping plot")
        return None


def save_convergence_plot(history_csv_path, out_png_path):
    """Plot every rms[...] residual column in history.csv vs iteration
    and save as a PNG. Same column-discovery logic as mesh_gui.py's
    live convergence tab (not hardcoded to a fixed variable list), so
    this works for both Euler and RANS histories."""
    plt = _get_plt()
    if plt is None:
        return

    if not os.path.isfile(history_csv_path):
        print(f"==> {history_csv_path} not found - skipping convergence plot")
        return

    with open(history_csv_path, newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = [fn.strip().strip('"') for fn in (reader.fieldnames or [])]
        rows = list(reader)

    if not rows:
        print(f"==> {history_csv_path} has no data rows - skipping convergence plot")
        return

    residual_cols = [fn for fn in fieldnames if fn.lower().startswith("rms[")]
    if not residual_cols:
        print(f"==> no rms[...] columns in {history_csv_path} - skipping convergence plot")
        return

    iters = list(range(len(rows)))
    fig, ax = plt.subplots(figsize=(9, 6))
    for col in residual_cols:
        raw_key = next((k for k in rows[0].keys() if k.strip().strip('"') == col), None)
        vals = []
        for row in rows:
            try:
                vals.append(float(row[raw_key].strip().strip('"')))
            except (ValueError, TypeError, KeyError):
                vals.append(None)
        ax.plot(iters, vals, label=col)

    ax.set_xlabel("iteration")
    ax.set_ylabel("residual")
    ax.set_title("Convergence history")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_png_path, dpi=150)
    plt.close(fig)
    print(f"==> saved convergence plot: {out_png_path}")


def save_sweep_plots(summary_csv_path, sweep_var, out_dir):
    """Plot CL, CD and CM vs sweep_var from the sweep summary CSV
    written at the end of a sweep, one PNG each."""
    plt = _get_plt()
    if plt is None:
        return

    if not os.path.isfile(summary_csv_path):
        print(f"==> {summary_csv_path} not found - skipping sweep plots")
        return

    with open(summary_csv_path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    xs, cls, cds, cms = [], [], [], []
    for row in rows:
        try:
            xs.append(float(row[sweep_var]))
        except (ValueError, KeyError):
            continue
        cls.append(float(row["CL"]) if row.get("CL") not in (None, "") else None)
        cds.append(float(row["CD"]) if row.get("CD") not in (None, "") else None)
        cms.append(float(row["CM"]) if row.get("CM") not in (None, "") else None)

    for label, ys, fname in (("CL", cls, "sweep_CL.png"), ("CD", cds, "sweep_CD.png"),
                             ("CM", cms, "sweep_CM.png")):
        pts = [(x, y) for x, y in zip(xs, ys) if y is not None]
        if not pts:
            print(f"==> no {label} data in {summary_csv_path} - skipping {fname}")
            continue
        pxs, pys = zip(*sorted(pts))
        fig, ax = plt.subplots(figsize=(7, 5))
        ax.plot(pxs, pys, marker="o")
        ax.set_xlabel(sweep_var)
        ax.set_ylabel(label)
        ax.set_title(f"{label} vs {sweep_var}")
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        out_path = os.path.join(out_dir, fname)
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
        print(f"==> saved sweep plot: {out_path}")

    save_performance_plots(xs, cls, cds, cms, sweep_var, out_dir, plt)


def _linfit(xs, ys):
    """Least-squares line y = a + b*x (plain Python - the ESP python
    isn't guaranteed to have numpy). Returns (a, b) or None."""
    n = len(xs)
    if n < 2:
        return None
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return None
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return my - b * mx, b


def save_performance_plots(xs, cls, cds, cms, sweep_var, out_dir, plt,
                           linear_max_aoa=10.0):
    """Aircraft-performance plots from the sweep results, on top of the
    plain CL/CD/CM-vs-sweep_var ones:
      sweep_LD.png         L/D vs sweep_var (max L/D marked)
      sweep_polar.png      drag polar CL vs CD, points labelled   (AoA sweeps)
      sweep_CD_vs_CL2.png  CD vs CL^2 with a line fit -> CD0, K   (AoA sweeps)
      sweep_summary.png    every panel on one page
      sweep_performance.txt  max L/D, CL_alpha, CD0, K
    CL_alpha and the CD0/K fit use only |AoA| <= linear_max_aoa (the
    attached-flow range) - at higher AoA the leading-edge vortex makes
    both curves nonlinear, which would bias a fit through every point."""
    pts = sorted((x, cl, cd, cm) for x, cl, cd, cm in zip(xs, cls, cds, cms)
                 if cl is not None and cd not in (None, 0))
    if len(pts) < 2:
        print("==> fewer than 2 complete sweep cases - skipping performance plots")
        return
    x = [p[0] for p in pts]; cl = [p[1] for p in pts]; cd = [p[2] for p in pts]
    cm = [p[3] for p in pts]
    ld = [l / d for l, d in zip(cl, cd)]
    is_aoa = sweep_var == "aoa"

    i_best = max(range(len(ld)), key=lambda i: ld[i])
    lines = [f"max L/D = {ld[i_best]:.3f} at {sweep_var} = {x[i_best]:g}"
             f" (CL = {cl[i_best]:.4f}, CD = {cd[i_best]:.4f})"]

    lin = [i for i in range(len(x)) if abs(x[i]) <= linear_max_aoa] if is_aoa else []
    if len(lin) < 3:
        lin = list(range(len(x)))       # not enough low-AoA points - fit them all
    cla_fit = _linfit([x[i] for i in lin], [cl[i] for i in lin]) if is_aoa else None
    polar_fit = (_linfit([cl[i] ** 2 for i in lin], [cd[i] for i in lin])
                 if is_aoa else None)
    if cla_fit:
        lines.append(f"CL_alpha = {cla_fit[1]:.5f} /deg = {cla_fit[1] * 57.29578:.4f} /rad"
                     f"  (fit over {sweep_var} {x[lin[0]]:g}..{x[lin[-1]]:g})")
    if polar_fit:
        lines.append(f"CD0 = {polar_fit[0]:.5f}   K = {polar_fit[1]:.5f}"
                     f"   (CD = CD0 + K*CL^2, same fit range)")

    def lab(ax, xl, yl, title):
        ax.set_xlabel(xl); ax.set_ylabel(yl); ax.set_title(title); ax.grid(True, alpha=0.3)

    def draw_ld(ax):
        ax.plot(x, ld, marker="o")
        ax.plot([x[i_best]], [ld[i_best]], "r*", ms=14,
                label=f"max {ld[i_best]:.2f} at {x[i_best]:g}")
        ax.legend(); lab(ax, sweep_var, "L/D", f"L/D vs {sweep_var}")

    def draw_polar(ax):
        ax.plot(cd, cl, marker="o")
        for xi, cdi, cli in zip(x, cd, cl):
            ax.annotate(f"{xi:g}", (cdi, cli), textcoords="offset points", xytext=(5, -10),
                        fontsize=8)
        lab(ax, "CD", "CL", "Drag polar (labels = AoA, deg)")

    def draw_cl2(ax):
        cl2 = [c ** 2 for c in cl]
        ax.plot(cl2, cd, "o", label="SU2")
        if polar_fit:
            top = max(cl2)
            ax.plot([0, top], [polar_fit[0], polar_fit[0] + polar_fit[1] * top], "--",
                    label=f"fit: CD0={polar_fit[0]:.4f}, K={polar_fit[1]:.4f}")
        ax.legend(); lab(ax, "CL^2", "CD", "CD vs CL^2")

    def draw_vs(ax, ys, name, fit=None):
        ax.plot(x, ys, marker="o", label="SU2")
        if fit:
            ax.plot([x[0], x[-1]], [fit[0] + fit[1] * x[0], fit[0] + fit[1] * x[-1]], "--",
                    label=f"linear fit, slope {fit[1]:.4f}/deg")
            ax.legend()
        lab(ax, sweep_var, name, f"{name} vs {sweep_var}")

    singles = [("sweep_LD.png", draw_ld)]
    if is_aoa:
        singles += [("sweep_polar.png", draw_polar), ("sweep_CD_vs_CL2.png", draw_cl2)]
    for fname, draw in singles:
        fig, ax = plt.subplots(figsize=(7, 5))
        draw(ax)
        fig.tight_layout()
        out_path = os.path.join(out_dir, fname)
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
        print(f"==> saved sweep plot: {out_path}")

    panels = [lambda ax: draw_vs(ax, cl, "CL", cla_fit), lambda ax: draw_vs(ax, cd, "CD"),
              draw_ld]
    if any(c is not None for c in cm):
        panels.append(lambda ax: draw_vs(ax, [c if c is not None else float("nan") for c in cm],
                                         "CM"))
    if is_aoa:
        panels += [draw_polar, draw_cl2]
    ncol = 3 if len(panels) > 4 else 2
    nrow = -(-len(panels) // ncol)
    fig, axs = plt.subplots(nrow, ncol, figsize=(5.2 * ncol, 4.2 * nrow))
    axs = list(axs.flat) if hasattr(axs, "flat") else [axs]
    for ax, draw in zip(axs, panels):
        draw(ax)
    for ax in axs[len(panels):]:
        ax.set_visible(False)
    fig.suptitle("Sweep performance summary\n" + "   |   ".join(lines), fontsize=9)
    fig.tight_layout()
    out_path = os.path.join(out_dir, "sweep_summary.png")
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    print(f"==> saved sweep plot: {out_path}")

    txt_path = os.path.join(out_dir, "sweep_performance.txt")
    with open(txt_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    for line in lines:
        print(f"==> {line}")
    print(f"==> performance numbers written to: {txt_path}")


def add_su2_sweep_args(p):
    """SU2 + sweep CLI args - identical regardless of which airframe is
    being meshed, so aflr_mesh_wing.py shares this instead of
    redefining it."""
    p.add_argument("--run-su2", action="store_true",
                    help="run SU2_CFD automatically after the mesh is written")
    p.add_argument("--su2-exe", default=r"C:\Users\duran\Downloads\SU2\win64-mpi\bin\SU2_CFD.exe")
    p.add_argument("--mpi-exe", default=r"C:\Program Files\Microsoft MPI\Bin\mpiexec.exe",
                    help="path to mpiexec.exe - runs SU2 as: mpiexec -n <mpi-procs> SU2_CFD.exe cfg. "
                         "Pass an empty string to run SU2_CFD.exe directly without MPI.")
    p.add_argument("--mpi-procs", type=int, default=4)
    p.add_argument("--su2-template", default="su2_template.cfg",
                    help="path to the SU2 .cfg template with __PLACEHOLDER__ substitutions")
    p.add_argument("--su2-config-out", default="droneCFD.cfg",
                    help="filename for the generated (substituted) SU2 config")
    p.add_argument("--mach", type=float, default=1.05)
    p.add_argument("--aoa", type=float, default=0.0)
    p.add_argument("--reynolds-number", type=float, default=5e6,
                    help="only used by viscous/RANS templates with a __REYNOLDS_NUMBER__ "
                         "placeholder (e.g. su2_template_wing_viscous.cfg) - ignored by "
                         "inviscid templates that don't have one")
    p.add_argument("--max-iter", type=int, default=9999)
    p.add_argument("--ref-length-outpmtr", default="wing:MAC",
                    help="OUTPMTR name in the .csm to use for SU2's REF_LENGTH")
    p.add_argument("--ref-area-outpmtr", default="wing:refArea",
                    help="OUTPMTR name in the .csm to use for SU2's REF_AREA")
    p.add_argument("--ref-length-fallback", type=float, default=1.0)
    p.add_argument("--ref-area-fallback", type=float, default=1.0)
    p.add_argument("--moment-ref-outpmtr", default="wing:xRef",
                    help="OUTPMTR name in the .csm for SU2's REF_ORIGIN_MOMENT_X "
                         "(pitching-moment reference, e.g. the MAC quarter chord)")
    p.add_argument("--moment-ref-fallback", type=float, default=0.25)

    # sweep - optional, loops the SU2 step only (mesh built once)
    p.add_argument("--sweep-var", choices=["none", "aoa", "mach"], default="none",
                    help="which variable to sweep - the mesh is built ONCE, only the "
                         "SU2 config/run is repeated per value (geometry doesn't "
                         "change for an AoA or Mach sweep, so remeshing would be wasted work)")
    p.add_argument("--sweep-values", default="",
                    help="comma-separated values for --sweep-var, e.g. '0,2,4,6,8' for AoA "
                         "or '0.8,1.05,1.2,1.5' for Mach. Each runs in its own subfolder.")
    return p


def run_su2_stage(args, capsProblem, su2_path):
    """Optional SU2 stage (single case or a sweep) run right after the
    mesh is written. Airframe-agnostic - only needs the parsed args,
    the pyCAPS Problem (for OUTPMTR lookups), and the .su2 mesh path -
    so aflr_mesh_wing.py calls this exact same function rather than
    duplicating the run/sweep/plotting logic."""
    if not args.run_su2:
        return
    if su2_path is None:
        print("==> --run-su2 requires --volume-format SU2 - skipping SU2 run")
        return

    ref_length = get_outpmtr(capsProblem, args.ref_length_outpmtr, args.ref_length_fallback)
    ref_area   = get_outpmtr(capsProblem, args.ref_area_outpmtr, args.ref_area_fallback)
    moment_x   = get_outpmtr(capsProblem, args.moment_ref_outpmtr, args.moment_ref_fallback)
    print(f"==> REF_LENGTH={ref_length} (from {args.ref_length_outpmtr}), "
          f"REF_AREA={ref_area} (from {args.ref_area_outpmtr}), "
          f"REF_ORIGIN_MOMENT_X={moment_x} (from {args.moment_ref_outpmtr})")

    mesh_dir = os.path.dirname(su2_path) or "."

    sweep_values = [s.strip() for s in args.sweep_values.split(",") if s.strip()]
    if args.sweep_var == "none" or not sweep_values:
        # ------------------- single case, as before -------------
        work_dir = mesh_dir
        cfg_path = os.path.join(work_dir, args.su2_config_out)
        write_su2_config(
            args.su2_template, cfg_path,
            mesh_filename=os.path.basename(su2_path),  # bare filename - cwd=mesh_dir
                                                         # below, and the mesh is
                                                         # already right there; a full
                                                         # path risks SU2's config
                                                         # parser choking on spaces
                                                         # (e.g. "...\New folder\...")
            mach=args.mach, aoa=args.aoa,
            ref_length=ref_length, ref_area=ref_area,
            max_iter=args.max_iter,
            reynolds_number=args.reynolds_number,
            moment_x=moment_x,
        )
        print(f"==> wrote SU2 config: {cfg_path}")

        rc = run_su2(args.su2_exe, cfg_path, work_dir,
                     mpi_exe=args.mpi_exe or None, mpi_procs=args.mpi_procs)
        if rc == 0:
            print("==> SU2 finished successfully")
        else:
            print(f"==> SU2 exited with code {rc}")

        save_convergence_plot(os.path.join(work_dir, "history.csv"),
                               os.path.join(work_dir, "convergence.png"))

    else:
        # ------------------- sweep: mesh once, loop SU2 ---------
        print(f"==> SWEEP starting: {args.sweep_var} = {sweep_values} "
              f"({len(sweep_values)} case(s)), mesh built once")

        results = []  # (value, CL, CD, CM)
        for raw_val in sweep_values:
            val = float(raw_val)
            mach = val if args.sweep_var == "mach" else args.mach
            aoa  = val if args.sweep_var == "aoa"  else args.aoa

            case_dir = os.path.join(mesh_dir, f"sweep_{args.sweep_var}_{sanitize_for_dirname(val)}")
            os.makedirs(case_dir, exist_ok=True)
            cfg_path = os.path.join(case_dir, args.su2_config_out)

            # copy the mesh into this case's folder and reference it by
            # a bare filename - a full path risks SU2's config parser
            # choking on spaces in the real path (e.g. "...\New folder\..."),
            # and a bare filename wouldn't otherwise resolve since SU2
            # runs with cwd=case_dir (a different directory than
            # mesh_dir, kept separate deliberately so each case's
            # history.csv/restart_flow/etc. don't overwrite each other)
            mesh_copy_path = os.path.join(case_dir, os.path.basename(su2_path))
            if not os.path.isfile(mesh_copy_path):
                shutil.copy2(su2_path, mesh_copy_path)

            print(f"\n==> sweep case: {args.sweep_var}={val}  (mach={mach}, aoa={aoa})")
            write_su2_config(
                args.su2_template, cfg_path,
                mesh_filename=os.path.basename(su2_path),
                mach=mach, aoa=aoa,
                ref_length=ref_length, ref_area=ref_area,
                max_iter=args.max_iter,
                reynolds_number=args.reynolds_number,
                moment_x=moment_x,
            )
            print(f"==> wrote SU2 config: {cfg_path}")

            rc = run_su2(args.su2_exe, cfg_path, case_dir,
                         mpi_exe=args.mpi_exe or None, mpi_procs=args.mpi_procs)

            history_path = os.path.join(case_dir, "history.csv")
            cl = cd = cm = None
            if rc == 0 and os.path.isfile(history_path):
                try:
                    cl, cd, cm = get_last_row_coeffs(history_path)
                except Exception as e:
                    print(f"==> could not read {history_path}: {e}")
            else:
                print(f"==> SU2 exited with code {rc} for {args.sweep_var}={val} "
                      f"- CL/CD unavailable for this case")

            save_convergence_plot(history_path, os.path.join(case_dir, "convergence.png"))

            results.append((val, cl, cd, cm))
            # printed in a fixed, parseable format so the GUI can
            # track sweep progress live from the log, same idea
            # as the "wrote SU2 config:" line used for history_path
            # (CM goes last so the GUI's "CL=... CD=..." regex still matches)
            print(f"==> sweep result: {args.sweep_var}={val} CL={cl} CD={cd} CM={cm}")

        summary_path = os.path.join(mesh_dir, f"sweep_{args.sweep_var}_results.csv")
        with open(summary_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow([args.sweep_var, "CL", "CD", "L_over_D", "CM"])
            for val, cl, cd, cm in results:
                lod = (cl / cd) if (cl is not None and cd not in (None, 0)) else ""
                w.writerow([val, cl if cl is not None else "", cd if cd is not None else "", lod,
                            cm if cm is not None else ""])

        n_ok = sum(1 for _, cl, cd, _cm in results if cl is not None and cd is not None)
        print(f"\n==> SWEEP complete: {n_ok}/{len(results)} case(s) produced CL/CD")
        print(f"==> sweep results written to: {summary_path}")

        save_sweep_plots(summary_path, args.sweep_var, mesh_dir)


def parse_args():
    p = argparse.ArgumentParser(description="AFLR4/AFLR3 mesh for supersonic_drone.csm")

    p.add_argument("--csm", required=True,
                    help=r'Path to the .csm file, e.g. "C:\Users\duran\Downloads\supersonic_drone (11).csm"')
    p.add_argument("--proj-name", default="droneMesh",
                    help="Proj_Name passed to both AFLR4 and AFLR3")
    p.add_argument("--surface-format", default="VTK",
                    help="AFLR4 Mesh_Format (surface mesh)")
    p.add_argument("--volume-format", default="VTK",
                    help="AFLR3 Mesh_Format (volume mesh)")

    # AFLR4 surface sizing
    p.add_argument("--curv-factor", type=float, default=0.18)
    p.add_argument("--ff-cdfr", type=float, default=1.1)
    p.add_argument("--mesh-length-factor", type=float, default=0.25,
                    help="ref_len = capsMeshLength * Mesh_Length_Factor")
    p.add_argument("--max-scale", type=float, default=3,
                    help="max_spacing = max_scale * ref_len")
    p.add_argument("--min-scale", type=float, default=0.05,
                    help="min_spacing = min_scale * ref_len")
    p.add_argument("--abs-min-scale", type=float, default=0.0001,
                    help="abs_min_spacing = abs_min_scale * ref_len")
    p.add_argument("--aircraft-edge-weight", type=float, default=1.2,
                    help="edgeWeight for the Aircraft capsGroup")

    # AFLR3 boundary layer
    p.add_argument("--bl", dest="bl_enabled", action="store_true", default=True,
                    help="enable AFLR3 boundary-layer prisms (-blc)")
    p.add_argument("--no-bl", dest="bl_enabled", action="store_false",
                    help="disable boundary-layer prisms - useful for isolating whether a "
                         "mesh failure is BL-specific (a sharp-feature issue) or a base "
                         "surface/volume-mesh issue")
    p.add_argument("--bl-max-layers", type=int, default=100)
    p.add_argument("--bl-initial-spacing", type=float, default=0.0001)
    p.add_argument("--bl-thickness", type=float, default=0.0005)

    p.add_argument("--out-level", type=int, default=0,
                    help="pyCAPS.Problem outLevel (0=quiet, higher=more verbose)")
    p.add_argument("--su2-marker-order", default="Farfield,symmetry,Aircraft",
                    help="comma-separated, index N -> renames SU2 marker BC_N to that name. "
                         "MUST match your build log's 'Mapping capsGroup attributes' section "
                         "(Name = X, index = N) - only applied when --volume-format is SU2. "
                         "Pass an empty string to skip renaming.")

    add_su2_sweep_args(p)

    return p.parse_args()


def main():
    args = parse_args()

    # ---------------------------------------------------------------
    # Load the CSM file
    # ---------------------------------------------------------------
    print("==> loading "+args.csm)
    capsProblem = pyCAPS.Problem(problemName="supersonic_drone",
                                  capsFile=args.csm,
                                  outLevel=args.out_level)

    # NOTE: build:finalUnion and build:cfdMesh must both be 1 in the
    # .csm's own CFGPMTR defaults - pyCAPS's geometry.cfgpmtr accessor
    # isn't reliably exposed in every install, so this script no
    # longer tries to override them here.

    # ---------------------------------------------------------------
    # AFLR4 - surface mesh
    # ---------------------------------------------------------------
    aflr4 = capsProblem.analysis.create(aim="aflr4AIM", name="aflr4")

    aflr4.input.Proj_Name    = args.proj_name
    aflr4.input.Mesh_Format  = args.surface_format

    aflr4.input.curv_factor  = args.curv_factor
    aflr4.input.ff_cdfr      = args.ff_cdfr

    aflr4.input.Mesh_Length_Factor = args.mesh_length_factor
    aflr4.input.max_scale          = args.max_scale
    aflr4.input.min_scale          = args.min_scale
    aflr4.input.abs_min_scale      = args.abs_min_scale

    # capsGroup names here MUST match the ATTRIBUTE capsGroup values
    # tagged in supersonic_drone.csm's build:cfdMesh block:
    #   "Farfield" - the 6 outer box faces (bcType Farfield)
    #   "Aircraft" - every aircraft surface face (default body-level
    #                group, no special bcType -> treated as a wall)
    aflr4.input.Mesh_Sizing = {
        "Farfield": {"bcType": "Farfield"},
        "Aircraft": {"edgeWeight": args.aircraft_edge_weight},
        "symmetry": {"bcType":"Symmetry"}
    }

    print("==> running AFLR4 (surface mesh)")
    aflr4.runAnalysis()

    # ---------------------------------------------------------------
    # AFLR3 - volume mesh, linked to the AFLR4 surface
    # ---------------------------------------------------------------
    aflr3 = capsProblem.analysis.create(aim="aflr3AIM", name="aflr3")
    aflr3.input["Surface_Mesh"].link(aflr4.output["Surface_Mesh"])

    aflr3.input.Proj_Name   = args.proj_name
    aflr3.input.Mesh_Format = args.volume_format

    if args.bl_enabled:
        print("==> boundary layer ENABLED: "
              f"max_layers={args.bl_max_layers} "
              f"initial_spacing={args.bl_initial_spacing} "
              f"thickness={args.bl_thickness}")
        aflr3.input.BL_Max_Layers         = args.bl_max_layers
        aflr3.input.BL_Initial_Spacing    = args.bl_initial_spacing
        aflr3.input.BL_Thickness          = args.bl_thickness
        aflr3.input.Mesh_Gen_Input_String = "-blc"
    else:
        print("==> boundary layer DISABLED (base volume mesh only)")

    print("==> running AFLR3 (volume mesh)")
    aflr3.runAnalysis()

    su2_path = None
    if args.volume_format.upper() == "SU2":
        try:
            su2_path = _find_su2_file(aflr3, args.proj_name)
        except FileNotFoundError as e:
            print(f"==> could not locate the SU2 mesh file: {e}")

        if su2_path:
            name_order = [s.strip() for s in args.su2_marker_order.split(",") if s.strip()]
            renamed = rename_su2_markers(su2_path, name_order)
            print(f"==> renamed SU2 markers in {su2_path}: {renamed} (used order: {name_order})")
            print("==> VERIFY this against your build log's 'Mapping capsGroup "
                  "attributes' section - if wrong, rerun with a corrected "
                  "--su2-marker-order, don't trust this blindly")

    print(f"==> done - {args.surface_format} surface mesh and "
          f"{args.volume_format} volume mesh written for {args.proj_name}")

    # ---------------------------------------------------------------
    # SU2 - optional, right after the mesh
    # ---------------------------------------------------------------
    run_su2_stage(args, capsProblem, su2_path)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("==> FAILED: "+str(exc))
        sys.exit(1)