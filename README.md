<p align="center">
  <img src="assets/seal_logo.png" alt="SEAL logo" width="300">
</p>

<h1 align="center">SEAL</h1>

<p align="center"><b>S</b>U2 · <b>E</b>SP · <b>A</b>FLR · <b>L</b>inkage</p>

SEAL links SU2, Engineering Sketch Pad (ESP), and AFLR. A Tkinter GUI sets up the ESP environment, generates an AFLR mesh from a `.csm` geometry with pyCAPS, and optionally runs SU2 (including angle-of-attack / Mach sweeps) with a live convergence plot.

## Requirements

- Windows 10/11 (x64)
- Python 3 with Tkinter, to launch the GUI
- [Engineering Sketch Pad (ESP)](https://acdl.mit.edu/ESP/), which includes its own Python, pyCAPS and AFLR
- [SU2](https://su2code.github.io/), plus Microsoft MPI for parallel runs (optional)

Tested with ESP 1.27 and SU2 8.3.0.

## Installation

### 1. Engineering Sketch Pad (ESP)

The Windows prebuilt includes everything SEAL needs from ESP: OpenCASCADE, a bundled Python with pyCAPS, and the AFLR mesh generators (`aflr3AIM` and `aflr4AIM` in `EngSketchPad\lib`).

1. Download the latest Windows prebuilt (`ESP1xx-win-x64.exe` or `.zip`) from <https://acdl.mit.edu/ESP/PreBuilts/>. If Windows blocks the browser download, download it from a Command Prompt instead:
   ```
   curl -O https://acdl.mit.edu/ESP/PreBuilts/ESP129-win-x64.zip
   ```
2. Extract it to a folder **with no spaces in the path**, for example `C:\ESP`.
3. Double-click `setup.cmd` in the extracted folder. It creates `EngSketchPad\ESPenv.bat` and two desktop shortcuts (`ESP1xx` and `runESP1xx`). You can ignore pip warnings about `scheme.headers` or `PYTHONHOME`.
4. If ESP complains that `VCRUNTIME*.dll` is missing, run `VC_redist.x64.exe` from the same folder (requires admin rights).
5. Check the install by double-clicking the `ESP1xx` desktop shortcut and running:
   ```
   python -c "import pyCAPS; print('pyCAPS OK')"
   ```
6. In that same ESP command prompt, install the extra Python packages SEAL uses. Run it from this repo's folder:
   ```
   pip install -r requirements.txt
   ```

### 2. SU2

1. Download the Windows binaries (`win64-mpi`) from <https://su2code.github.io/download.html> and extract them, for example to `C:\SU2`. `SU2_CFD.exe` is in the `bin` folder.
2. For parallel runs, install [Microsoft MPI](https://learn.microsoft.com/en-us/message-passing-interface/microsoft-mpi) (`msmpisetup.exe` only; the SDK is not needed). This installs `C:\Program Files\Microsoft MPI\Bin\mpiexec.exe`.
3. Optional: to run SU2 from any terminal, add the `bin` folder to your `Path` environment variable and create a `SU2_RUN` variable pointing to it. The GUI does not need this because you give it the full path to `SU2_CFD.exe`.
4. Check the install:
   ```
   C:\SU2\win64-mpi\bin\SU2_CFD.exe --help
   ```

### 3. SEAL

```
git clone https://github.com/sebasD27/SEAL.git
```

Then fill in these fields in the GUI (see Running below):

| GUI field | Set it to |
|---|---|
| ESP environment directory | `<ESP folder>\EngSketchPad` (the folder containing `ESPenv.bat`) |
| Python executable | `<ESP folder>\Python-3.x.x\python.exe` |
| Mesh script location | `aflr_mesh_drone.py` or `aflr_mesh_wing.py` in this repo |
| SU2_CFD executable | `<SU2 folder>\win64-mpi\bin\SU2_CFD.exe` |
| mpiexec.exe | `C:\Program Files\Microsoft MPI\Bin\mpiexec.exe`, or blank to run SU2 without MPI |

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

## AI disclosure

Parts of this project were developed with the help of AI tools, including Claude (Anthropic) through Claude Code. AI assistance was used to write and edit code and documentation, including this README, `.gitignore` and `requirements.txt`. All AI-assisted work was reviewed by the author, who is responsible for the project's content. Mesh and CFD results should be checked independently before they are relied on.
