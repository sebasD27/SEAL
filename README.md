# SEAL

A program that links SU2, Engineering Sketch Pad (ESP), and AFLR. A Tkinter GUI sets up the ESP environment, generates an AFLR mesh from a `.csm` geometry with pyCAPS, and optionally runs SU2 (including angle-of-attack / Mach sweeps) with a live convergence plot.

## Requirements

- Python 3 with Tkinter, to launch the GUI
- [Engineering Sketch Pad](https://acdl.mit.edu/ESP/) with its bundled Python and pyCAPS, plus AFLR
- [SU2](https://su2code.github.io/), and MS-MPI for parallel runs (optional)
- For the post-processing scripts: `pip install -r requirements.txt`

## Running

```
run_mesh_gui.bat
```

or `python mesh_gui.py`. In the GUI, set the paths to your ESP install, ESP's `python.exe`, the `.csm` file and SU2. Settings are saved to `mesh_gui_config.json`, which is git-ignored because it holds machine-specific paths.

## Files

| File | Purpose |
|---|---|
| `mesh_gui.py` | GUI front end |
| `aflr_mesh_drone.py` | Drone meshing (AFLR4 surface + AFLR3 volume) and SU2 run |
| `aflr_mesh_wing.py` | Wing meshing using Kulfan airfoil parameterization |
| `kulfan.py` | Kulfan (CST) airfoil parameterization |
| `su2_template*.cfg` | SU2 configuration templates |
| `su2_inspect.py` | Diagnose SU2 "Can't find the edge" mesh errors |
| `check_separation.py`, `vortex_slices.py` | Post-processing of SU2 `.vtu` output |
| `polhamus.py` | Polhamus leading-edge suction analogy estimates |
| `ONERA_D_selig.dat` | Sample airfoil coordinates |
