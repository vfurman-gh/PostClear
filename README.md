# Post Process All: Fusion CAM Batch Post & Verification Add-In

### Overview
`Post Process All` is a Fusion 360 Manufacture workspace add-in (v3.0.0) that batch post-processes one or multiple `NC Program` entries with full In-Process Workpiece (IPW) simulation, collision detection, safe rapid (`G0`) restoration, and per-program verification reports and plots.

You can invoke `Post Process All` from:
- The **Manufacture** workspace toolbar (next to native **Post Process**) or **Actions** menu.
- The **Right-Click Context Menu** on any `NC Program`, the `NC Programs` folder, or a `Setup` — placed immediately below Fusion's built-in **Post Process** item, pre-checking the corresponding NC Programs in the dialog.

![Post Process All](https://raw.githubusercontent.com/TimPaterson/Fusion360-Batch-Post/master/resources/Command/32x32.png)

---

### Installation (Windows & macOS)

1. Download **`PostProcessAll-v3.0.0.zip`** from **Releases** and unzip the **`PostProcessAll`** folder anywhere you keep Fusion Add-Ins (for example `Documents/FusionAddIns/PostProcessAll`). Keep the folder named **`PostProcessAll`**.
   - If cloning from GitHub instead, clone directly into a folder named `PostProcessAll`:
     ```bash
     git clone <repo-url> PostProcessAll
     ```
2. In Fusion 360, go to **Utilities → ADD-INS → Scripts and Add-Ins** (`Shift+S`), open the **Add-Ins** tab, click **`+`** next to *My Add-Ins*, and select the **`PostProcessAll`** folder.
3. Ensure **Run on Startup** is checked and click **Run**.
4. **Automatic Dependency Setup**: On first load (and automatically after any future Fusion 360 Python update), `PostProcessAll` uses Fusion's built-in Python to install `numpy`, `shapely`, and `matplotlib` into `PostProcessAll/packages/<abi>/` — no system Python or terminal commands required.

#### Updating
Replace the contents of your `PostProcessAll` folder with the new release (keeping your `PostProcessAll.settings` file if present), or run `git pull` if cloned from GitHub, then restart Fusion 360 or click **Stop** / **Run** in **Scripts and Add-Ins**.

#### Uninstalling
Open **Scripts and Add-Ins** (`Shift+S`) → **Add-Ins**, click **Stop** and remove `PostProcessAll` from *My Add-Ins*, then delete the `PostProcessAll` folder.

---

### Workflow & Features

#### Multi-NC-Program Dialog & Naming
- **NC Program Checklist**: Displays all document `NC Program` entries in a table with a left-aligned **Select All** header row, along with each program's target setup and resolved output filename. If no `NC Program` exists yet, one is created automatically.
- **Right-Click Context Pre-Selection**: Right-clicking an `NC Program` pre-checks that program; right-clicking `NC Programs` pre-checks all programs; right-clicking a `Setup` pre-checks every `NC Program` referencing that setup.
- **Output Naming (`nc_program_filename`)**: Every `NC Program` always outputs using its configured `nc_program_filename` parameter (falling back to the `NC Program` name, then the setup name). Duplicate target paths across selected programs are detected before execution starts.
- **Subfolders (`:`)**: Use a colon (`:`) in `nc_program_filename` to organize output files into nested subfolders inside the NC Program's output folder (e.g., `Cover:Part1` writes `<OutputFolder>/Cover/Part1.nc`).
- **Pre-Flight Prompts & Single Progress Dialog**: Runs empty/invalid toolpath checks and existing-file overwrite confirmations once across the entire batch before posting under a single progress bar, restoring original NC Program parameters on completion.
- **Modeless Verification Toast**: Displays a compact floating notification with per-program verification status (`NC`, `Report`, `Plot`, `Folder` quick-open actions and `Dismiss`). Clean single- or multi-program runs auto-dismiss after 2 seconds unless interacted with; warnings or collision failures remain open until dismissed.

---

### Unified Toolpath Verification, Collision-Free Rapid Promotion & Diagnostics
`Post Process All` includes a complete In-Process Workpiece (IPW) simulation and toolpath verifier for both **Milling** and **Turning** setups using a clean 2-pass architecture:
- **Pass 1 (Direct Context Dump)**: Emits per-operation kinematic events, tool definitions, and section data via `universal_dumper.cps` inside Fusion without modifying or registering in the user's Post Library.
- **Full Stock Mode Support**:
  - **Parametric Stock**: Box, Cylinder, and Tube stock definitions (`FixedBoxStock`, `RelativeBoxStock`, `FixedCylinderStock`, `RelativeCylinderStock`, `FixedTubeStock`, `RelativeTubeStock`).
  - **From Solid (`SolidStock`)**: Extracts B-Rep and MeshBody stock geometry directly from Fusion, transforms vertices into the setup WCS, and builds an exact 3D heightfield or axisymmetric turning envelope.
  - **From Preceding Setup (`PreviousSetupStock`)**: Automatically triggers Fusion CAM's native in-process stock STL export, validates unit scale against `workpieceHint`, and verifies toolpath/stock bounding-box alignment.
- **Unified Verification Simulation**:
  - **Milling (`geometry/heightfield_engine.py`)**: Vectorized 3D Heightfield grid simulation tracking material removal, gouges, shank rubs, holder strikes, and rapids into stock (with `MillClearanceVerifier` fallback when NumPy is unavailable).
  - **Turning (`geometry/lathe_engine.py`)**: Exact 2D axisymmetric IPW polygon simulation using `shapely`, tracking swept inserts and holders, fixture/chuck guard monitoring, and dynamic Constant Surface Speed (`G96` CSS) spindle RPM modeling clamped by `maximumSpindleSpeed`.
  - **Universal Collision Detection**: Detects `RAPID_INTO_STOCK`, `HOLDER_STRIKE`, `FIXTURE_COLLISION`, and `SHANK_RUB`, logging detailed diagnostics while continuing final NC code generation.
- **Collision-Free Rapid Promotion**: When **Restore rapid moves (G0)** is enabled, verified air moves with zero material removal and safe cutter/holder clearances are promoted to `G0` using per-event `(sec_idx, event_idx)` addressing:
  - Enforces a uniform **`1.5 mm` minimum travel length filter** (`DEFAULT_MIN_PROMOTE_LEN_MM = 1.5`) so tiny micro-segments are never promoted to `G0`.
  - Automatically disables promotion when the target post processor uses dog-leg rapid mapping (`highFeedMapping != 0` or `useG0 = false`).
  - Blocks promotion inside canned cycles (`in_cycle`), during active cutter radius compensation (`rc != 0` or compensation transitions), on synchronized moves (`sync`), or on any event with a detected collision.
- **Pass 2 (Single-Pass Vendor Replay)**: Emits final machine-dialect NC code by executing the user's unmodified vendor post processor (`universal_replayer.cps`) once across all merged sections, natively preserving tool changes and modal states.
- **Diagnostics & High-Resolution Reports**: Generates per-setup markdown reports (`<Output>_report.md`) with per-operation MRR spike analysis, stock/toolpath alignment bounds, and collision breakdowns, plus verification plots (`<Output>.png`).

---

### Compatibility
Here is a list of post processors that have been tested with `Post Process All`:

| Post Processor | Review OK? | Tested | Tool change | Numeric Name | File Ext. |
|----           | :----: | :----: |----    | :----: |---- |
| centroid.cps  | Yes | Yes |                        | Yes | .nc  |
| eding.cps     | Yes | No  | N10 M9:G28             | No  | .cnc |
| gbrl.cps      | Yes | Yes | M9:G28 G91 Z0:G90      | No  | .nc  |
| linuxcnc.cps  | Yes | No  | M9 G30                 | No  | .ngc |
| mach3mill.cps | Yes | Yes |                        | No  | .tap |
| tinyg.cps     | Yes | Yes |                        | No  | .gcode |
| tormach.cps   | Yes | Yes | M9 G30                 | No  | .nc  |
