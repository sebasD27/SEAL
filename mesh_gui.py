###################################################################
#                                                                 #
# mesh_gui.py -- GUI front-end for running aflr_mesh_drone.py
#
#   Replicates the manual workflow of:
#     1. open a cmd prompt
#     2. set up the ESP environment (ESPenv.bat in your ESP dir)
#     3. call your ESP-bundled python.exe
#     4. run aflr_mesh_drone.py with the right arguments
#
#   ...as one GUI: fill in the paths once, click Run, watch the
#   log. Settings are remembered between runs in mesh_gui_config.json
#   (saved next to this script).
#
#   Requires only the Python standard library (tkinter, subprocess,
#   threading, json) - no extra installs needed to run the GUI
#   itself. The ESP python (selected in the GUI) is what needs
#   pyCAPS installed, not whatever python launches this GUI.
#                                                                 #
###################################################################

import csv
import json
import os
import queue
import re
import subprocess
import sys
import tempfile
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mesh_gui_config.json")


_GUI_DIR = os.path.dirname(os.path.abspath(__file__))

DEFAULTS = {
    "espenv_dir":        r"C:\Users\duran\OneDrive\Desktop\ESP\ESP127",
    "espenv_script":      "ESPenv.bat",
    "python_exe":         r"C:\Users\duran\OneDrive\Desktop\ESP\ESP127\Python-3.11.9\python.exe",
    "case":               "drone",
    "mesh_script":        os.path.join(_GUI_DIR, "aflr_mesh_drone.py"),
    "csm_file":           r"C:\Users\duran\Downloads\supersonic_drone (11).csm",
    "proj_name":          "droneMesh",
    "surface_format":     "VTK",
    "volume_format":      "VTK",
    "mesh_length_factor": "0.25",
    "max_scale":          "3",
    "min_scale":          "0.05",
    "abs_min_scale":      "0.0001",
    "aircraft_edge_weight": "1.2",
    "airfoil_file":       r"C:\Users\duran\Downloads\ONERA_D_selig.dat",
    "wing_sweep":         "26.7",
    "wing_dihedral":      "0",
    "wing_taper":         "0.56",
    "upper_edge_weight":  "1.2",
    "lower_edge_weight":  "1.2",
    "su2_marker_order":   "Farfield,symmetry,Aircraft",
    "bl_enabled":         True,
    "bl_max_layers":      "100",
    "bl_initial_spacing": "0.0001",
    "bl_thickness":       "0.0005",
    "run_su2":            False,
    "su2_exe":            r"C:\Users\duran\Downloads\SU2\win64-mpi\bin\SU2_CFD.exe",
    "mpi_exe":            r"C:\Program Files\Microsoft MPI\Bin\mpiexec.exe",
    "mpi_procs":          "4",
    "su2_template":       os.path.join(_GUI_DIR, "su2_template.cfg"),
    "su2_config_out":     "droneCFD.cfg",
    "mach":               "1.05",
    "aoa":                "0.0",
    "reynolds_number":    "5000000",
    "max_iter":           "9999",
    "ref_length_outpmtr": "wing:MAC",
    "ref_area_outpmtr":   "wing:refArea",
    "sweep_var":          "none",
    "sweep_values":       "0, 2, 4, 6, 8",
}

# Values that switch when the Case selector changes - convenience
# starting points, not enforced (the user can still edit any field
# after switching). Keyed by the internal case id ("drone"/"wing"),
# not the combobox display label.
CASE_LABELS = {"drone": "Supersonic Drone", "wing": "Test Wing"}
CASE_PRESETS = {
    "drone": {
        "mesh_script":        os.path.join(_GUI_DIR, "aflr_mesh_drone.py"),
        "csm_file":           r"C:\Users\duran\Downloads\supersonic_drone (11).csm",
        "proj_name":          "droneMesh",
        "su2_marker_order":   "Farfield,symmetry,Aircraft",
        "mesh_length_factor": "0.25",
        "max_scale":          "3",
        "min_scale":          "0.05",
        "abs_min_scale":      "0.0001",
        "bl_initial_spacing": "0.0001",
        "bl_thickness":       "0.0005",
        # inviscid Euler - MARKER_EULER= ( Aircraft )
        "su2_template":       os.path.join(_GUI_DIR, "su2_template.cfg"),
    },
    "wing": {
        "mesh_script":        os.path.join(_GUI_DIR, "aflr_mesh_wing.py"),
        "csm_file":           r"C:\Users\duran\Downloads\testWing.csm",
        "proj_name":          "TestWing",
        "su2_marker_order":   "Farfield,symmetry,upperFace,lowerFace,tail,tip",
        "mesh_length_factor": "1",
        "max_scale":          "3",
        "min_scale":          "0.05",
        "abs_min_scale":      "0.001",
        "bl_initial_spacing": "0.00000145",
        "bl_thickness":       "0.0147235456",
        # viscous RANS (SA) - needs the BL-prism mesh above; uses
        # MARKER_HEATFLUX= ( upperFace, lowerFace, tail, tip ), matching
        # this case's capsGroups (the drone's inviscid template doesn't)
        "su2_template":       os.path.join(_GUI_DIR, "su2_template_wing_viscous.cfg"),
    },
}


def load_config():
    cfg = dict(DEFAULTS)
    if os.path.exists(CONFIG_PATH):
        try:
            with open(CONFIG_PATH, "r") as f:
                cfg.update(json.load(f))
        except Exception:
            pass
    return cfg


def save_config(cfg):
    try:
        with open(CONFIG_PATH, "w") as f:
            json.dump(cfg, f, indent=2)
    except Exception:
        pass


class MeshGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("AFLR Mesh Runner")
        self.geometry("900x920")

        self.cfg = load_config()
        self.proc = None
        self.log_queue = queue.Queue()

        self._build_widgets()
        self.title(f"{CASE_LABELS.get(self.cfg['case'], self.cfg['case'])} - AFLR Mesh Runner")
        self.after(100, self._poll_log_queue)

    # -----------------------------------------------------------
    def _build_widgets(self):
        pad = {"padx": 6, "pady": 4}

        # --- scrollable settings area -------------------------------
        # everything above the buttons/log gets long with the SU2
        # section added, so it scrolls independently rather than
        # forcing the window (or the always-visible log below) to
        # keep growing
        scroll_outer = ttk.Frame(self)
        scroll_outer.pack(fill="both", expand=False, padx=0, pady=0)
        scroll_outer.configure(height=560)
        scroll_outer.pack_propagate(False)

        settings_canvas = tk.Canvas(scroll_outer, highlightthickness=0)
        settings_scroll = ttk.Scrollbar(scroll_outer, orient="vertical", command=settings_canvas.yview)
        settings_canvas.configure(yscrollcommand=settings_scroll.set)
        settings_canvas.pack(side="left", fill="both", expand=True)
        settings_scroll.pack(side="right", fill="y")

        settings_frame = ttk.Frame(settings_canvas)
        settings_window = settings_canvas.create_window((0, 0), window=settings_frame, anchor="nw")

        def _on_frame_configure(event):
            settings_canvas.configure(scrollregion=settings_canvas.bbox("all"))
        settings_frame.bind("<Configure>", _on_frame_configure)

        def _on_canvas_configure(event):
            # keep the inner frame's width matched to the canvas so
            # widgets don't get clipped/left with dead space on resize
            settings_canvas.itemconfigure(settings_window, width=event.width)
        settings_canvas.bind("<Configure>", _on_canvas_configure)

        def _on_mousewheel(event):
            settings_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        # bind_all so scrolling works with the cursor anywhere over the
        # settings area, not just directly over the canvas widget
        settings_canvas.bind("<Enter>", lambda e: settings_canvas.bind_all("<MouseWheel>", _on_mousewheel))
        settings_canvas.bind("<Leave>", lambda e: settings_canvas.unbind_all("<MouseWheel>"))

        parent = settings_frame  # everything below attaches here instead of self

        case_frame = ttk.LabelFrame(parent, text="Case")
        case_frame.pack(fill="x", **pad)
        ttk.Label(case_frame, text="Which airframe to mesh").grid(row=0, column=0, sticky="w", padx=6, pady=3)
        self.case_var = tk.StringVar(value=CASE_LABELS.get(self.cfg["case"], CASE_LABELS["drone"]))
        case_combo = ttk.Combobox(case_frame, textvariable=self.case_var,
                                   values=list(CASE_LABELS.values()), state="readonly", width=20)
        case_combo.grid(row=0, column=1, sticky="w", padx=6, pady=3)
        case_combo.bind("<<ComboboxSelected>>", self._on_case_change)
        ttk.Label(case_frame, text="Switching this fills in that case's script/CSM/mesh defaults below - "
                                    "review before running.",
                  font=("TkDefaultFont", 8)).grid(row=1, column=0, columnspan=2, sticky="w", padx=6, pady=(0, 4))

        paths = ttk.LabelFrame(parent, text="Environment & files")
        paths.pack(fill="x", **pad)

        self.espenv_dir   = self._path_row(paths, 0, "ESP environment directory",
                                            self.cfg["espenv_dir"], is_dir=True)
        self.espenv_script = self._entry_row(paths, 1, "ESP env script name (inside that directory)",
                                              self.cfg["espenv_script"])
        self.python_exe   = self._path_row(paths, 2, "Python executable (the ESP-bundled one)",
                                            self.cfg["python_exe"], is_dir=False)
        self.mesh_script  = self._path_row(paths, 3, "Mesh script location (aflr_mesh_drone.py / aflr_mesh_wing.py)",
                                            self.cfg["mesh_script"], is_dir=False)
        self.csm_file     = self._path_row(paths, 4, ".csm file to mesh",
                                            self.cfg["csm_file"], is_dir=False)

        mesh = ttk.LabelFrame(parent, text="Mesh settings")
        mesh.pack(fill="x", **pad)

        self.proj_name      = self._entry_row(mesh, 0, "Proj_Name", self.cfg["proj_name"])
        self.surface_format = self._entry_row(mesh, 1, "AFLR4 surface Mesh_Format", self.cfg["surface_format"])
        self.volume_format  = self._entry_row(mesh, 2, "AFLR3 volume Mesh_Format", self.cfg["volume_format"])
        self.mesh_length_factor = self._entry_row(mesh, 3, "Mesh_Length_Factor", self.cfg["mesh_length_factor"])
        self.max_scale       = self._entry_row(mesh, 4, "max_scale", self.cfg["max_scale"])
        self.min_scale        = self._entry_row(mesh, 5, "min_scale", self.cfg["min_scale"])
        self.abs_min_scale    = self._entry_row(mesh, 6, "abs_min_scale", self.cfg["abs_min_scale"])
        self.su2_marker_order = self._entry_row(mesh, 7, "SU2 marker order (index 1,2,3... - verify against build log)",
                                                  self.cfg["su2_marker_order"])

        # --- case-specific settings ----------------------------------
        # only one of these two frames is shown at a time (toggled by
        # the Case selector above) - kept in a fixed-position container
        # so showing/hiding doesn't reorder the rest of the settings
        case_specific = ttk.Frame(parent)
        case_specific.pack(fill="x")

        self.drone_case_frame = ttk.LabelFrame(case_specific, text="Aircraft (Supersonic Drone) settings")
        self.aircraft_edge_weight = self._entry_row(self.drone_case_frame, 0, "Aircraft edgeWeight",
                                                      self.cfg["aircraft_edge_weight"])

        self.wing_case_frame = ttk.LabelFrame(case_specific, text="Wing (Test Wing) settings")
        self.airfoil_file  = self._path_row(self.wing_case_frame, 0, "Airfoil .dat file (read by Kulfan)",
                                             self.cfg["airfoil_file"], is_dir=False)
        self.wing_sweep    = self._entry_row(self.wing_case_frame, 1, "wing:sweep (deg)", self.cfg["wing_sweep"])
        self.wing_dihedral = self._entry_row(self.wing_case_frame, 2, "wing:dihedral (deg)", self.cfg["wing_dihedral"])
        self.wing_taper    = self._entry_row(self.wing_case_frame, 3, "wing:taper", self.cfg["wing_taper"])
        self.upper_edge_weight = self._entry_row(self.wing_case_frame, 4, "upperFace edgeWeight",
                                                   self.cfg["upper_edge_weight"])
        self.lower_edge_weight = self._entry_row(self.wing_case_frame, 5, "lowerFace edgeWeight",
                                                   self.cfg["lower_edge_weight"])

        self._show_case_frame(self.cfg["case"])

        bl = ttk.LabelFrame(parent, text="AFLR3 boundary layer")
        bl.pack(fill="x", **pad)

        self.bl_enabled_var = tk.BooleanVar(value=self.cfg["bl_enabled"])
        ttk.Checkbutton(bl, text="Enable boundary layer (-blc)  "
                                  "(uncheck to isolate BL-specific mesh failures)",
                         variable=self.bl_enabled_var).grid(row=0, column=0, columnspan=3, sticky="w", **pad)

        self.bl_max_layers      = self._entry_row(bl, 1, "BL_Max_Layers", self.cfg["bl_max_layers"])
        self.bl_initial_spacing = self._entry_row(bl, 2, "BL_Initial_Spacing", self.cfg["bl_initial_spacing"])
        self.bl_thickness       = self._entry_row(bl, 3, "BL_Thickness", self.cfg["bl_thickness"])

        su2 = ttk.LabelFrame(parent, text="SU2 (optional, runs after the mesh)")
        su2.pack(fill="x", **pad)

        self.run_su2_var = tk.BooleanVar(value=self.cfg["run_su2"])
        ttk.Checkbutton(su2, text="Run SU2 automatically after the mesh finishes",
                         variable=self.run_su2_var).grid(row=0, column=0, columnspan=3, sticky="w", padx=6, pady=4)

        self.su2_exe      = self._path_row(su2, 1, "SU2_CFD executable", self.cfg["su2_exe"], is_dir=False)
        self.mpi_exe      = self._path_row(su2, 2, "mpiexec.exe (blank to run SU2 directly, no MPI)",
                                            self.cfg["mpi_exe"], is_dir=False)
        self.mpi_procs    = self._entry_row(su2, 3, "MPI processes (-n)", self.cfg["mpi_procs"])
        self.su2_template = self._path_row(su2, 4, "SU2 config template (su2_template.cfg)",
                                            self.cfg["su2_template"], is_dir=False)
        self.su2_config_out = self._entry_row(su2, 5, "Generated config filename", self.cfg["su2_config_out"])
        self.mach          = self._entry_row(su2, 6, "Mach number", self.cfg["mach"])
        self.aoa           = self._entry_row(su2, 7, "Angle of attack (deg)", self.cfg["aoa"])
        self.reynolds_number = self._entry_row(su2, 8,
            "Reynolds number (only used by viscous/RANS templates, e.g. su2_template_wing_viscous.cfg)",
            self.cfg["reynolds_number"])
        self.max_iter       = self._entry_row(su2, 9, "Max SU2 iterations", self.cfg["max_iter"])
        self.ref_length_outpmtr = self._entry_row(su2, 10, "REF_LENGTH source (OUTPMTR name in .csm)",
                                                    self.cfg["ref_length_outpmtr"])
        self.ref_area_outpmtr = self._entry_row(su2, 11, "REF_AREA source (OUTPMTR name in .csm)",
                                                  self.cfg["ref_area_outpmtr"])

        sweep = ttk.LabelFrame(parent, text="Sweep (optional - meshes once, loops SU2 only)")
        sweep.pack(fill="x", **pad)

        ttk.Label(sweep, text="Sweep variable").grid(row=0, column=0, sticky="w", padx=6, pady=3)
        self.sweep_var = tk.StringVar(value=self.cfg["sweep_var"])
        ttk.Combobox(sweep, textvariable=self.sweep_var, values=["none", "aoa", "mach"],
                     state="readonly", width=10).grid(row=0, column=1, sticky="w", padx=6, pady=3)
        self.sweep_values = self._entry_row(sweep, 1,
            "Sweep values (comma-separated - overrides Mach/AoA above for the swept variable)",
            self.cfg["sweep_values"], width=60)
        ttk.Label(sweep, text="Fixed Mach/AoA above still apply to whichever variable ISN'T being swept.",
                  font=("TkDefaultFont", 8)).grid(row=2, column=0, columnspan=3, sticky="w", padx=6, pady=(0, 4))

        # --- fixed area below the scrollable settings ---------------
        btns = ttk.Frame(self)
        btns.pack(fill="x", **pad)
        self.run_btn = ttk.Button(btns, text="Run", command=self.on_run)
        self.run_btn.pack(side="left", padx=4)
        ttk.Button(btns, text="Save settings", command=self.on_save).pack(side="left", padx=4)
        ttk.Button(btns, text="Clear log", command=self.on_clear_log).pack(side="left", padx=4)

        output_notebook = ttk.Notebook(self)
        output_notebook.pack(fill="both", expand=True, **pad)

        log_frame = ttk.Frame(output_notebook)
        conv_frame = ttk.Frame(output_notebook)
        sweep_frame = ttk.Frame(output_notebook)
        output_notebook.add(log_frame, text="Output log")
        output_notebook.add(conv_frame, text="Convergence plot")
        output_notebook.add(sweep_frame, text="Sweep results")

        self.log_text = tk.Text(log_frame, wrap="word", state="disabled", bg="black", fg="#d0d0d0")
        self.log_text.pack(fill="both", expand=True, side="left")
        scroll = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        scroll.pack(side="right", fill="y")
        self.log_text.configure(yscrollcommand=scroll.set)

        self.conv_status = ttk.Label(conv_frame, text="Waiting for history.csv...")
        self.conv_status.pack(fill="x", padx=6, pady=(4, 0))
        self.conv_canvas = tk.Canvas(conv_frame, bg="white", highlightthickness=1,
                                      highlightbackground="#999999")
        self.conv_canvas.pack(fill="both", expand=True, padx=6, pady=6)
        self.conv_canvas.bind("<Configure>", lambda e: self._draw_convergence())

        self.sweep_status = ttk.Label(sweep_frame, text="No sweep running")
        self.sweep_status.pack(fill="x", padx=6, pady=(4, 0))
        self.sweep_cl_canvas = tk.Canvas(sweep_frame, bg="white", highlightthickness=1,
                                          highlightbackground="#999999", height=1)
        self.sweep_cl_canvas.pack(fill="both", expand=True, padx=6, pady=(6, 3))
        self.sweep_cd_canvas = tk.Canvas(sweep_frame, bg="white", highlightthickness=1,
                                          highlightbackground="#999999", height=1)
        self.sweep_cd_canvas.pack(fill="both", expand=True, padx=6, pady=(3, 6))
        self.sweep_cl_canvas.bind("<Configure>", lambda e: self._draw_sweep_plots())
        self.sweep_cd_canvas.bind("<Configure>", lambda e: self._draw_sweep_plots())

        self.history_path = None
        self.sweep_var_name = None
        self.sweep_results = []  # list of (value, CL, CD)
        self.after(1000, self._poll_convergence)
        self.after(1000, self._poll_sweep_plot)

    # -----------------------------------------------------------
    def _case_id(self):
        label = self.case_var.get()
        for cid, lbl in CASE_LABELS.items():
            if lbl == label:
                return cid
        return "drone"

    def _show_case_frame(self, case):
        self.drone_case_frame.pack_forget()
        self.wing_case_frame.pack_forget()
        if case == "wing":
            self.wing_case_frame.pack(fill="x", padx=6, pady=4)
        else:
            self.drone_case_frame.pack(fill="x", padx=6, pady=4)

    def _on_case_change(self, event=None):
        case = self._case_id()
        preset = CASE_PRESETS.get(case, {})
        for key, value in preset.items():
            getattr(self, key).set(value)
        self._show_case_frame(case)
        self.title(f"{CASE_LABELS.get(case, case)} - AFLR Mesh Runner")

    def _poll_convergence(self):
        self._draw_convergence()
        self.after(2000, self._poll_convergence)

    def _poll_sweep_plot(self):
        self._draw_sweep_plots()
        self.after(2000, self._poll_sweep_plot)

    def _draw_xy_plot(self, canvas, x_vals, y_vals, x_label, y_label, color="#1f77b4"):
        canvas.delete("all")
        if len(x_vals) < 1:
            return

        w = canvas.winfo_width() or 400
        h = canvas.winfo_height() or 150
        margin_l, margin_r, margin_t, margin_b = 55, 15, 12, 28
        plot_w = max(10, w - margin_l - margin_r)
        plot_h = max(10, h - margin_t - margin_b)

        x_min, x_max = min(x_vals), max(x_vals)
        y_min, y_max = min(y_vals), max(y_vals)
        if x_min == x_max:
            x_min -= 1
            x_max += 1
        if y_min == y_max:
            y_min -= abs(y_min) * 0.1 + 0.01
            y_max += abs(y_max) * 0.1 + 0.01
        x_pad = (x_max - x_min) * 0.08
        y_pad = (y_max - y_min) * 0.1

        def to_px(xv, yv):
            x = margin_l + ((xv - (x_min - x_pad)) / ((x_max + x_pad) - (x_min - x_pad))) * plot_w
            y = margin_t + (1 - (yv - (y_min - y_pad)) / ((y_max + y_pad) - (y_min - y_pad))) * plot_h
            return x, y

        canvas.create_rectangle(margin_l, margin_t, margin_l + plot_w, margin_t + plot_h, outline="#888888")
        for frac in (0, 0.5, 1.0):
            gy = margin_t + frac * plot_h
            val = (y_max + y_pad) - frac * ((y_max + y_pad) - (y_min - y_pad))
            canvas.create_line(margin_l, gy, margin_l + plot_w, gy, fill="#eeeeee")
            canvas.create_text(margin_l - 6, gy, text=f"{val:.3g}", anchor="e", font=("TkDefaultFont", 8))
        canvas.create_text(margin_l, margin_t + plot_h + 12, text=f"{x_min:.3g}", anchor="n", font=("TkDefaultFont", 8))
        canvas.create_text(margin_l + plot_w, margin_t + plot_h + 12, text=f"{x_max:.3g}",
                            anchor="n", font=("TkDefaultFont", 8))
        canvas.create_text(margin_l + plot_w / 2, margin_t + plot_h + 12, text=x_label,
                            anchor="n", font=("TkDefaultFont", 8))
        canvas.create_text(12, margin_t, text=y_label, anchor="nw", font=("TkDefaultFont", 8, "bold"))

        points = sorted(zip(x_vals, y_vals))
        pixel_pts = [to_px(xv, yv) for xv, yv in points]
        if len(pixel_pts) >= 2:
            canvas.create_line(*[c for pt in pixel_pts for c in pt], fill=color, width=2)
        for (xv, yv), (px, py) in zip(points, pixel_pts):
            canvas.create_oval(px - 3, py - 3, px + 3, py + 3, fill=color, outline="")
            canvas.create_text(px, py - 10, text=f"{yv:.4g}", font=("TkDefaultFont", 7))

    def _draw_sweep_plots(self):
        if not self.sweep_results:
            self.sweep_status.configure(text="No sweep running")
            self.sweep_cl_canvas.delete("all")
            self.sweep_cd_canvas.delete("all")
            return

        var_name = self.sweep_var_name or "value"
        complete = [(v, cl, cd) for v, cl, cd in self.sweep_results if cl is not None and cd is not None]
        self.sweep_status.configure(
            text=f"{len(complete)}/{len(self.sweep_results)} case(s) with CL/CD  (sweeping {var_name})")

        if not complete:
            self.sweep_cl_canvas.delete("all")
            self.sweep_cd_canvas.delete("all")
            return

        xs = [v for v, _, _ in complete]
        cls = [cl for _, cl, _ in complete]
        cds = [cd for _, _, cd in complete]
        self._draw_xy_plot(self.sweep_cl_canvas, xs, cls, var_name, "CL", color="#1f77b4")
        self._draw_xy_plot(self.sweep_cd_canvas, xs, cds, var_name, "CD", color="#d62728")

    def _draw_convergence(self):
        canvas = self.conv_canvas
        canvas.delete("all")

        if not self.history_path or not os.path.isfile(self.history_path):
            self.conv_status.configure(text="Waiting for history.csv..."
                                        + (f"  ({self.history_path})" if self.history_path else ""))
            return

        try:
            with open(self.history_path, newline="") as f:
                reader = csv.DictReader(f)
                fieldnames = [fn.strip().strip('"') for fn in (reader.fieldnames or [])]
                rows = list(reader)
        except Exception as e:
            self.conv_status.configure(text=f"Could not read history.csv yet ({e})")
            return

        if not rows:
            self.conv_status.configure(text="history.csv has no data rows yet")
            return

        # residual columns: anything named like rms[...] (works for
        # Euler's density/momentum/energy residuals AND RANS's extra
        # turbulence residual, e.g. rms[nu]) - not hardcoded to a
        # fixed variable list
        residual_cols = [fn for fn in fieldnames if fn.lower().startswith("rms[")]
        if not residual_cols:
            self.conv_status.configure(text="No rms[...] residual columns found in history.csv")
            return

        series = {}
        for col in residual_cols:
            raw_key = None
            for k in rows[0].keys():
                if k.strip().strip('"') == col:
                    raw_key = k
                    break
            vals = []
            for row in rows:
                try:
                    vals.append(float(row[raw_key].strip().strip('"')))
                except (ValueError, TypeError, KeyError):
                    vals.append(None)
            series[col] = vals

        n = len(rows)
        all_vals = [v for vals in series.values() for v in vals if v is not None]
        if not all_vals:
            self.conv_status.configure(text="No numeric residual data yet")
            return

        y_min, y_max = min(all_vals), max(all_vals)
        if y_min == y_max:
            y_min -= 1
            y_max += 1
        y_pad = (y_max - y_min) * 0.05
        y_min -= y_pad
        y_max += y_pad

        w = canvas.winfo_width() or 600
        h = canvas.winfo_height() or 300
        margin_l, margin_r, margin_t, margin_b = 55, 140, 15, 30
        plot_w = max(10, w - margin_l - margin_r)
        plot_h = max(10, h - margin_t - margin_b)

        def to_px(i, v):
            x = margin_l + (i / max(1, n - 1)) * plot_w
            y = margin_t + (1 - (v - y_min) / (y_max - y_min)) * plot_h
            return x, y

        # axes + gridlines
        canvas.create_rectangle(margin_l, margin_t, margin_l + plot_w, margin_t + plot_h,
                                 outline="#888888")
        for frac in (0, 0.25, 0.5, 0.75, 1.0):
            gy = margin_t + frac * plot_h
            val = y_max - frac * (y_max - y_min)
            canvas.create_line(margin_l, gy, margin_l + plot_w, gy, fill="#dddddd")
            canvas.create_text(margin_l - 6, gy, text=f"{val:.1f}", anchor="e", font=("TkDefaultFont", 8))
        canvas.create_text(margin_l, margin_t + plot_h + 14, text="0", anchor="n", font=("TkDefaultFont", 8))
        canvas.create_text(margin_l + plot_w, margin_t + plot_h + 14, text=str(n - 1),
                            anchor="n", font=("TkDefaultFont", 8))
        canvas.create_text(margin_l + plot_w / 2, margin_t + plot_h + 14, text="iteration",
                            anchor="n", font=("TkDefaultFont", 8))

        colors = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd", "#8c564b", "#17becf"]
        for idx, (name, vals) in enumerate(series.items()):
            color = colors[idx % len(colors)]
            points = []
            for i, v in enumerate(vals):
                if v is None:
                    continue
                points.append(to_px(i, v))
            if len(points) >= 2:
                canvas.create_line(*[c for pt in points for c in pt], fill=color, width=2)

            # legend entry, current (last) value shown alongside
            ly = margin_t + idx * 16
            lx = margin_l + plot_w + 12
            canvas.create_line(lx, ly + 6, lx + 16, ly + 6, fill=color, width=2)
            last_val = next((v for v in reversed(vals) if v is not None), None)
            label = name if last_val is None else f"{name} = {last_val:.2f}"
            canvas.create_text(lx + 20, ly, text=label, anchor="nw", font=("TkDefaultFont", 8))

        self.conv_status.configure(text=f"{n} iteration(s) - {self.history_path}")

    def _path_row(self, parent, row, label, default, is_dir):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=6, pady=3)
        var = tk.StringVar(value=default)
        entry = ttk.Entry(parent, textvariable=var, width=70)
        entry.grid(row=row, column=1, sticky="we", padx=6, pady=3)

        def browse():
            if is_dir:
                chosen = filedialog.askdirectory(initialdir=var.get() or ".")
            else:
                chosen = filedialog.askopenfilename(initialdir=os.path.dirname(var.get()) or ".")
            if chosen:
                var.set(chosen)

        ttk.Button(parent, text="Browse...", command=browse).grid(row=row, column=2, padx=6, pady=3)
        parent.columnconfigure(1, weight=1)
        return var

    def _entry_row(self, parent, row, label, default, width=30):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=6, pady=3)
        var = tk.StringVar(value=default)
        ttk.Entry(parent, textvariable=var, width=width).grid(row=row, column=1, sticky="w", padx=6, pady=3)
        return var

    # -----------------------------------------------------------
    def _collect_config(self):
        return {
            "espenv_dir":        self.espenv_dir.get(),
            "espenv_script":     self.espenv_script.get(),
            "python_exe":        self.python_exe.get(),
            "case":              self._case_id(),
            "mesh_script":       self.mesh_script.get(),
            "csm_file":          self.csm_file.get(),
            "proj_name":         self.proj_name.get(),
            "surface_format":    self.surface_format.get(),
            "volume_format":     self.volume_format.get(),
            "mesh_length_factor": self.mesh_length_factor.get(),
            "max_scale":         self.max_scale.get(),
            "min_scale":         self.min_scale.get(),
            "abs_min_scale":     self.abs_min_scale.get(),
            "aircraft_edge_weight": self.aircraft_edge_weight.get(),
            "airfoil_file":      self.airfoil_file.get(),
            "wing_sweep":        self.wing_sweep.get(),
            "wing_dihedral":     self.wing_dihedral.get(),
            "wing_taper":        self.wing_taper.get(),
            "upper_edge_weight": self.upper_edge_weight.get(),
            "lower_edge_weight": self.lower_edge_weight.get(),
            "su2_marker_order":  self.su2_marker_order.get(),
            "bl_enabled":        self.bl_enabled_var.get(),
            "bl_max_layers":     self.bl_max_layers.get(),
            "bl_initial_spacing": self.bl_initial_spacing.get(),
            "bl_thickness":      self.bl_thickness.get(),
            "run_su2":           self.run_su2_var.get(),
            "su2_exe":           self.su2_exe.get(),
            "mpi_exe":           self.mpi_exe.get(),
            "mpi_procs":         self.mpi_procs.get(),
            "su2_template":      self.su2_template.get(),
            "su2_config_out":    self.su2_config_out.get(),
            "mach":              self.mach.get(),
            "aoa":               self.aoa.get(),
            "reynolds_number":   self.reynolds_number.get(),
            "max_iter":          self.max_iter.get(),
            "ref_length_outpmtr": self.ref_length_outpmtr.get(),
            "ref_area_outpmtr":  self.ref_area_outpmtr.get(),
            "sweep_var":         self.sweep_var.get(),
            "sweep_values":      self.sweep_values.get(),
        }

    def on_save(self):
        save_config(self._collect_config())
        messagebox.showinfo("Saved", "Settings saved to "+CONFIG_PATH)

    def on_clear_log(self):
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    # -----------------------------------------------------------
    def on_run(self):
        if self.proc is not None and self.proc.poll() is None:
            messagebox.showwarning("Already running", "A mesh run is already in progress.")
            return

        cfg = self._collect_config()
        save_config(cfg)

        # basic sanity checks before we launch anything
        espenv_full = os.path.join(cfg["espenv_dir"], cfg["espenv_script"])
        problems = []
        if not os.path.isdir(cfg["espenv_dir"]):
            problems.append("ESP environment directory does not exist:\n  "+cfg["espenv_dir"])
        elif not os.path.isfile(espenv_full):
            problems.append("ESP env script not found inside that directory:\n  "+espenv_full)
        if not os.path.isfile(cfg["python_exe"]):
            problems.append("Python executable not found:\n  "+cfg["python_exe"])
        if not os.path.isfile(cfg["mesh_script"]):
            problems.append("Mesh script not found:\n  "+cfg["mesh_script"])
        if not os.path.isfile(cfg["csm_file"]):
            problems.append(".csm file not found:\n  "+cfg["csm_file"])
        if cfg["case"] == "wing" and not os.path.isfile(cfg["airfoil_file"]):
            problems.append("Airfoil .dat file not found:\n  "+cfg["airfoil_file"])

        if problems:
            messagebox.showerror("Cannot run", "\n\n".join(problems))
            return

        if cfg["run_su2"]:
            if not os.path.isfile(cfg["su2_exe"]):
                messagebox.showerror("Cannot run", "SU2_CFD executable not found:\n  "+cfg["su2_exe"])
                return
            if cfg["mpi_exe"].strip() and not os.path.isfile(cfg["mpi_exe"]):
                messagebox.showerror("Cannot run", "mpiexec.exe not found:\n  "+cfg["mpi_exe"]
                                      + "\n\n(leave this field blank to run SU2 directly without MPI)")
                return
            if not os.path.isfile(cfg["su2_template"]):
                messagebox.showerror("Cannot run", "SU2 config template not found:\n  "+cfg["su2_template"])
                return

        cmd_args = [
            f'"{cfg["python_exe"]}"',
            f'"{cfg["mesh_script"]}"',
            "--csm", f'"{cfg["csm_file"]}"',
            "--proj-name", cfg["proj_name"],
            "--surface-format", cfg["surface_format"],
            "--volume-format", cfg["volume_format"],
            "--mesh-length-factor", cfg["mesh_length_factor"],
            "--max-scale", cfg["max_scale"],
            "--min-scale", cfg["min_scale"],
            "--abs-min-scale", cfg["abs_min_scale"],
            "--su2-marker-order", f'"{cfg["su2_marker_order"]}"',
            "--bl-max-layers", cfg["bl_max_layers"],
            "--bl-initial-spacing", cfg["bl_initial_spacing"],
            "--bl-thickness", cfg["bl_thickness"],
        ]
        cmd_args.append("--bl" if cfg["bl_enabled"] else "--no-bl")

        if cfg["case"] == "wing":
            cmd_args += [
                "--airfoil-file", f'"{cfg["airfoil_file"]}"',
                "--sweep", cfg["wing_sweep"],
                "--dihedral", cfg["wing_dihedral"],
                "--taper", cfg["wing_taper"],
                "--upper-edge-weight", cfg["upper_edge_weight"],
                "--lower-edge-weight", cfg["lower_edge_weight"],
            ]
        else:
            cmd_args += ["--aircraft-edge-weight", cfg["aircraft_edge_weight"]]

        if cfg["run_su2"]:
            cmd_args += [
                "--run-su2",
                "--su2-exe", f'"{cfg["su2_exe"]}"',
                "--mpi-exe", f'"{cfg["mpi_exe"]}"',
                "--mpi-procs", cfg["mpi_procs"],
                "--su2-template", f'"{cfg["su2_template"]}"',
                "--su2-config-out", cfg["su2_config_out"],
                "--mach", cfg["mach"],
                f'--aoa={cfg["aoa"]}',
                "--reynolds-number", cfg["reynolds_number"],
                "--max-iter", cfg["max_iter"],
                "--ref-length-outpmtr", cfg["ref_length_outpmtr"],
                "--ref-area-outpmtr", cfg["ref_area_outpmtr"],
            ]
            if cfg["sweep_var"] != "none" and cfg["sweep_values"].strip():
                cmd_args += [
                    "--sweep-var", cfg["sweep_var"],
                    # "=" form: a list starting with a negative value
                    # ("-2,0,2") would otherwise be parsed as a new flag
                    f'--sweep-values="{cfg["sweep_values"]}"',
                ]
            # SU2 now runs wherever AFLR3 wrote the mesh (not a fixed
            # directory the GUI knows ahead of time), so history.csv's
            # location isn't known until the subprocess actually
            # prints it - _run_subprocess watches the live log for the
            # "wrote SU2 config:" line and sets self.history_path then
        self.history_path = None
        self.sweep_results = []
        self.sweep_var_name = cfg["sweep_var"] if (cfg["run_su2"] and cfg["sweep_var"] != "none"
                                                     and cfg["sweep_values"].strip()) else None

        # write a temp .bat that sources the ESP environment (call, so
        # it stays in THIS shell) and then runs python - this mirrors
        # exactly what typing it by hand at a cmd prompt does
        bat_fd, bat_path = tempfile.mkstemp(suffix=".bat")
        with os.fdopen(bat_fd, "w") as f:
            f.write("@echo off\r\n")
            f.write(f'call "{espenv_full}"\r\n')
            f.write(" ".join(cmd_args) + "\r\n")
        self._bat_path = bat_path

        self.on_clear_log()
        self._log(f"--> ESP env : {espenv_full}\n")
        self._log(f"--> python  : {cfg['python_exe']}\n")
        self._log(f"--> script  : {cfg['mesh_script']}\n")
        self._log(f"--> csm     : {cfg['csm_file']}\n")
        self._log("--> starting...\n\n")

        self.run_btn.configure(state="disabled")
        threading.Thread(target=self._run_subprocess, args=(bat_path,), daemon=True).start()

    def _run_subprocess(self, bat_path):
        try:
            self.proc = subprocess.Popen(
                ["cmd.exe", "/c", bat_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            for line in self.proc.stdout:
                self.log_queue.put(line)
                # discover history.csv's actual location live, since
                # SU2 now runs wherever AFLR3 wrote the mesh rather
                # than a fixed directory the GUI knows in advance
                m = re.search(r"wrote SU2 config:\s*(.+?)\s*$", line)
                if m:
                    cfg_dir = os.path.dirname(m.group(1).strip())
                    self.history_path = os.path.join(cfg_dir, "history.csv")

                # live sweep results, one line per finished case:
                # "==> sweep result: aoa=2.0 CL=0.123 CD=0.045"
                sm = re.search(
                    r"sweep result:\s*\S+=([-\d.eE]+)\s+CL=(\S+)\s+CD=(\S+)", line)
                if sm:
                    def _f(s):
                        try:
                            return float(s)
                        except ValueError:
                            return None
                    val, cl, cd = _f(sm.group(1)), _f(sm.group(2)), _f(sm.group(3))
                    if val is not None:
                        self.sweep_results.append((val, cl, cd))
            self.proc.wait()
            self.log_queue.put(f"\n--> process exited with code {self.proc.returncode}\n")
        except Exception as exc:
            self.log_queue.put(f"\n--> FAILED TO LAUNCH: {exc}\n")
        finally:
            self.log_queue.put("__DONE__")
            try:
                os.remove(bat_path)
            except Exception:
                pass

    def _poll_log_queue(self):
        try:
            while True:
                line = self.log_queue.get_nowait()
                if line == "__DONE__":
                    self.run_btn.configure(state="normal")
                else:
                    self._log(line)
        except queue.Empty:
            pass
        self.after(100, self._poll_log_queue)

    def _log(self, text):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", text)
        self.log_text.see("end")
        self.log_text.configure(state="disabled")


if __name__ == "__main__":
    if os.name != "nt":
        print("mesh_gui.py generates a Windows .bat wrapper to source your ESP "
              "environment, so it's built for Windows. It may still run "
              "elsewhere but the ESP env / cmd.exe steps won't apply.")
    app = MeshGui()
    app.mainloop()