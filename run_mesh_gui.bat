@echo off
REM Launches the AFLR mesh GUI. Uses "python" from your normal system
REM PATH just to start the GUI itself (Tkinter is standard library,
REM so any reasonably modern Python works here) - the GUI then lets
REM you separately pick the ESP-bundled python that actually needs
REM pyCAPS installed for running the mesh.
REM
REM If plain "python" isn't on your PATH, edit the line below to
REM point at a specific python.exe instead.

setlocal
set SCRIPT_DIR=%~dp0
python "%SCRIPT_DIR%mesh_gui.py"
if errorlevel 1 (
    echo.
    echo Something went wrong launching the GUI - see the error above.
    pause
)
