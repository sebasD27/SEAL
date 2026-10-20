#!/usr/bin/env python3
"""
polhamus.py - Polhamus leading-edge-suction-analogy estimate of lift and
drag for the drone's delta wing, plotted against SU2 results.

    CL = Kp sin(a) cos^2(a)  +  Kv cos(a) sin^2(a)
         potential lift          vortex lift
    Kv = (Kp - Kp^2 Ki) / cos(LE sweep)        Ki = 1/(pi AR)
    CD (no LE suction, vortex flow) = CD0 + CL tan(a)
    CD (full LE suction, attached)  = CD0 + Ki CL^2

Kp comes from supersonic linear theory for a delta wing with a subsonic
leading edge (Kp = 2 pi cot(LE) / E(k), k = sqrt(1 - (beta cot LE)^2)),
which is valid while beta*cot(LE) < 1. Pass --kp to override it, e.g.
with the CL slope measured from the CFD sweep. Polhamus assumes a sharp
leading edge and no vortex breakdown, so it is an upper bound on vortex
lift for a rounded-edge, non-slender (~53 deg) wing like this one.

usage:
  python polhamus.py --sweep-csv supersonic_drone/Scratch/aflr3/sweep_aoa_results.csv
  python polhamus.py --point 20,1.0346,0.4279 --out polhamus.png
"""
import argparse, csv, math, os


def coefficients(mach, croot, taper, semispan, kp=None):
    ctip = croot * taper
    le = math.atan((croot - ctip) / semispan)              # straight TE: xtip = croot - ctip
    area = semispan * (croot + ctip)                       # full wing, both panels
    ar = (2 * semispan) ** 2 / area
    ki = 1 / (math.pi * ar)
    if kp is None:
        m = math.sqrt(mach * mach - 1) / math.tan(le) if mach > 1 else 0.0
        if m >= 1:
            raise SystemExit(f"beta*cot(LE) = {m:.2f} >= 1: supersonic leading edge, "
                             "the subsonic-LE formula does not apply - pass --kp")
        from scipy.special import ellipe
        kp = 2 * math.pi / math.tan(le) / ellipe(1 - m * m)
    kv = (kp - kp * kp * ki) / math.cos(le)
    return dict(le=math.degrees(le), ar=ar, ki=ki, kp=kp, kv=kv)


def read_points(paths, points):
    """(label, [(aoa, CL, CD), ...]) series from sweep CSVs and --point args."""
    series = []
    for p in paths or []:
        rows = []
        with open(p, newline="") as f:
            for r in csv.DictReader(f):
                try:
                    rows.append((float(r["aoa"]), float(r["CL"]), float(r["CD"])))
                except (KeyError, ValueError):
                    continue
        series.append((os.path.basename(os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(p))))) or p, sorted(rows)))
    if points:
        series.append(("SU2 point(s)", [tuple(float(v) for v in s.split(",")) for s in points]))
    return series


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mach", type=float, default=1.1)
    ap.add_argument("--croot", type=float, default=3.0, help="wing:croot in the .csm")
    ap.add_argument("--taper", type=float, default=0.1, help="wing:taper")
    ap.add_argument("--semispan", type=float, default=2.0, help="wing:span (semi-span)")
    ap.add_argument("--kp", type=float, help="override the potential lift slope Kp (per rad)")
    ap.add_argument("--cd0", type=float, default=0.03, help="zero-lift drag for the drag bounds")
    ap.add_argument("--sweep-csv", nargs="*", help="sweep_aoa_results.csv file(s) to overlay")
    ap.add_argument("--point", action="append", help="extra SU2 result 'aoa,CL,CD' (repeatable)")
    ap.add_argument("--amax", type=float, default=40.0)
    ap.add_argument("--out", default="polhamus.png")
    a = ap.parse_args()

    c = coefficients(a.mach, a.croot, a.taper, a.semispan, a.kp)
    print(f"LE sweep {c['le']:.1f} deg  AR {c['ar']:.3f}  Ki {c['ki']:.4f}  "
          f"Kp {c['kp']:.3f}/rad  Kv {c['kv']:.3f}")

    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    al = [i * a.amax / 200 for i in range(201)]
    r = [math.radians(x) for x in al]
    pot = [c["kp"] * math.sin(x) * math.cos(x) ** 2 for x in r]
    vor = [c["kv"] * math.cos(x) * math.sin(x) ** 2 for x in r]
    tot = [p + v for p, v in zip(pot, vor)]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.2))
    ax1.plot(al, tot, "k-", lw=2, label=f"Polhamus total (Kp={c['kp']:.2f}, Kv={c['kv']:.2f})")
    ax1.plot(al, pot, "k--", label="potential lift only (no vortex lift)")
    ax1.fill_between(al, pot, tot, color="0.85", label="vortex-lift increment")
    ax2.plot([a.cd0 + cl * math.tan(x) for cl, x in zip(tot, r)], tot, "k-",
             label="no LE suction: CD0 + CL tan(a), along Polhamus CL(a)")
    ax2.plot([a.cd0 + c["ki"] * cl * cl for cl in tot], tot, "k--",
             label="full LE suction: CD0 + Ki CL^2  (attached)")
    series = read_points(a.sweep_csv, a.point)
    for (label, pts), mk in zip(series, "osD^v"):
        if not pts:
            continue
        x, cl, cd = zip(*pts)
        ax1.plot(x, cl, mk, ms=8, label=label)
        ax2.plot(cd, cl, mk, ms=8, label=label)
        # no-suction drag at each point's OWN aoa and CL - the like-for-like check
        ax2.plot([a.cd0 + cli * math.tan(math.radians(xi)) for xi, cli, _ in pts], cl, "rx",
                 ms=9, label=f"{label}: no-suction CD at its own aoa")
        for xi, cli, cdi in pts:
            tc = math.radians(xi)
            pol = c["kp"] * math.sin(tc) * math.cos(tc) ** 2 + c["kv"] * math.cos(tc) * math.sin(tc) ** 2
            print(f"  {label}: aoa {xi:5.1f}  CFD CL {cli:7.4f}  Polhamus {pol:7.4f}  "
                  f"ratio {cli / pol if pol else float('nan'):5.2f}   CFD CD {cdi:.4f}  "
                  f"no-suction CD {a.cd0 + cli * math.tan(tc):.4f}  full-suction CD {a.cd0 + c['ki'] * cli * cli:.4f}")
    ax1.set(xlabel="angle of attack (deg)", ylabel="CL", title=f"Lift: Polhamus vs SU2, M = {a.mach}")
    ax2.set(xlabel="CD", ylabel="CL", title="Drag polar: leading-edge suction bounds")
    top = max([max(p[1] for p in pts) for _, pts in series if pts] or [1.0])
    right = max([max(p[2] for p in pts) for _, pts in series if pts] or [0.9])
    ax2.set_ylim(0, 1.4 * max(top, 1.0)); ax2.set_xlim(0, 1.1 * max(right, 0.9))
    for ax in (ax1, ax2):
        ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(a.out, dpi=140)
    print(f"plot: {os.path.abspath(a.out)}")


if __name__ == "__main__":
    main()
