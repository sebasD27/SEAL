###################################################################
#                                                                 #
# aflr_mesh_wing.py -- surface + volume mesh for testWing.csm
#                       using AFLR4 (surface) then AFLR3 (volume).
#
#              Adapted from testWing (1).py, generalized the same
#              way aflr_mesh_drone.py generalized AFLRtest.py: the
#              Kulfan airfoil file and wing despmtrs (sweep/dihedral/
#              taper) are CLI args instead of hardcoded, so mesh_gui.py
#              - or you, by hand - can drive it without editing this
#              file each time.
#
#              Everything SU2/sweep-related (write_su2_config,
#              run_su2, run_su2_stage, save_convergence_plot,
#              save_sweep_plots, ...) is shared with
#              aflr_mesh_drone.py by import - none of that logic is
#              airframe-specific, only the geometry/Mesh_Sizing setup
#              below is.
#                                                                 #
###################################################################

import argparse
import sys

import numpy as np
import pyCAPS
from kulfan import Kulfan

from aflr_mesh_drone import (
    _find_su2_file,
    rename_su2_markers,
    add_su2_sweep_args,
    run_su2_stage,
)


def load_kulfan_coefficients(airfoil_file):
    afl = Kulfan()
    afl.readfile(airfoil_file)
    upper = np.asarray(afl.upperCoefficients.magnitude, dtype=float)
    lower = np.asarray(afl.lowerCoefficients.magnitude, dtype=float)
    return upper, lower


def parse_args():
    p = argparse.ArgumentParser(description="AFLR4/AFLR3 mesh for testWing.csm")

    p.add_argument("--csm", required=True,
                    help=r'Path to the .csm file, e.g. "C:\Users\duran\Downloads\testWing.csm"')
    p.add_argument("--proj-name", default="TestWing",
                    help="Proj_Name passed to both AFLR4 and AFLR3")
    p.add_argument("--surface-format", default="VTK",
                    help="AFLR4 Mesh_Format (surface mesh)")
    p.add_argument("--volume-format", default="VTK",
                    help="AFLR3 Mesh_Format (volume mesh)")

    # wing shape - Kulfan-parameterized airfoil + planform despmtrs
    p.add_argument("--airfoil-file", required=True,
                    help=r'Path to the .dat airfoil coordinate file read by Kulfan, '
                         r'e.g. "C:\Users\duran\Downloads\ONERA_D_selig.dat"')
    p.add_argument("--sweep", type=float, default=26.7, help="wing:sweep (deg)")
    p.add_argument("--dihedral", type=float, default=0.0, help="wing:dihedral (deg)")
    p.add_argument("--taper", type=float, default=0.56, help="wing:taper")

    # AFLR4 surface sizing
    p.add_argument("--curv-factor", type=float, default=0.18)
    p.add_argument("--ff-cdfr", type=float, default=1.1)
    p.add_argument("--mesh-length-factor", type=float, default=1.0,
                    help="ref_len = capsMeshLength * Mesh_Length_Factor")
    p.add_argument("--max-scale", type=float, default=3,
                    help="max_spacing = max_scale * ref_len")
    p.add_argument("--min-scale", type=float, default=0.05,
                    help="min_spacing = min_scale * ref_len")
    p.add_argument("--abs-min-scale", type=float, default=0.001,
                    help="abs_min_spacing = abs_min_scale * ref_len")
    p.add_argument("--upper-edge-weight", type=float, default=1.2,
                    help="edgeWeight for the upperFace capsGroup")
    p.add_argument("--lower-edge-weight", type=float, default=1.2,
                    help="edgeWeight for the lowerFace capsGroup")

    # AFLR3 boundary layer
    p.add_argument("--bl", dest="bl_enabled", action="store_true", default=True,
                    help="enable AFLR3 boundary-layer prisms (-blc)")
    p.add_argument("--no-bl", dest="bl_enabled", action="store_false",
                    help="disable boundary-layer prisms - useful for isolating whether a "
                         "mesh failure is BL-specific (a sharp-feature issue) or a base "
                         "surface/volume-mesh issue")
    p.add_argument("--bl-max-layers", type=int, default=100)
    p.add_argument("--bl-initial-spacing", type=float, default=1.45e-6)
    p.add_argument("--bl-thickness", type=float, default=0.0147235456)

    p.add_argument("--out-level", type=int, default=0,
                    help="pyCAPS.Problem outLevel (0=quiet, higher=more verbose)")
    p.add_argument("--su2-marker-order", default="Farfield,symmetry,upperFace,lowerFace,tail,tip",
                    help="comma-separated, index N -> renames SU2 marker BC_N to that name. "
                         "MUST match your build log's 'Mapping capsGroup attributes' section "
                         "(Name = X, index = N) - only applied when --volume-format is SU2. "
                         "Pass an empty string to skip renaming.")

    add_su2_sweep_args(p)

    return p.parse_args()


def main():
    args = parse_args()

    # ---------------------------------------------------------------
    # Load the CSM file and set the wing shape despmtrs
    # ---------------------------------------------------------------
    print("==> loading "+args.csm)
    capsProblem = pyCAPS.Problem(problemName="test_wing",
                                  capsFile=args.csm,
                                  outLevel=args.out_level)

    print("==> reading airfoil "+args.airfoil_file+" (Kulfan)")
    upper, lower = load_kulfan_coefficients(args.airfoil_file)
    geometry = capsProblem.geometry
    geometry.despmtr["aupper"].value = upper
    geometry.despmtr["alower"].value = lower

    geometry.despmtr["wing:sweep"].value    = args.sweep
    geometry.despmtr["wing:dihedral"].value = args.dihedral
    geometry.despmtr["wing:taper"].value    = args.taper

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
    # tagged in testWing.csm's SELECT FACE blocks: "upperFace",
    # "lowerFace", "symmetry", "Farfield" (plus "tail"/"tip", left at
    # AFLR4's default sizing since testWing (1).py never sized them
    # explicitly either)
    aflr4.input.Mesh_Sizing = {
        "upperFace": {"edgeWeight": args.upper_edge_weight},
        "lowerFace": {"edgeWeight": args.lower_edge_weight},
        "symmetry":  {"bcType": "Symmetry"},
        "Farfield":  {"bcType": "Farfield"},
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
            print(f"==> renamed SU2 markers in {su2_path}: {renamed}")

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
