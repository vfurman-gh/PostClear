# PostClear — Fusion 360 CAM Multi-Tool Post & Clearance Verification

![PostClear](resources/Command/32x32.png)

Fusion 360 Personal Edition CAM users run into two annoyances at the post-processing stage:

* **Personal Edition removes rapid moves:** Fusion converts rapid positioning moves (G0) into linear feed moves, slowing air travel across the job. Simple text-replacement scripts that blindly turn high-feed moves back into rapids risk driving a cutter or toolholder straight into uncut stock or a chuck jaw.
* **Personal Edition blocks multi-tool NC files:** Fusion refuses to post operations that use multiple tools into a single NC file, forcing you to export one separate file per tool and run them individually or stitch G-code by hand.

**PostClear combines multi-tool operations into one complete NC file with native tool changes (and batches multiple NC Programs in one run), simulates the evolving stock across every tool to catch collisions, and restores G0 rapids only where the tool and holder clear the workpiece.**

As a bonus, PostClear generates high-resolution material removal rate (MRR) plots so you can spot machine overload or underload at a glance.

---

## Installation (Windows & macOS)

No separate Python installation or command-line setup is needed.

1. **Download and extract:** Download the latest release archive from [GitHub Releases](https://github.com/vfurman-gh/PostClear/releases) and unzip the PostClear folder wherever you keep Fusion add-ins (for example, `Documents/FusionAddIns/PostClear`). Keep the folder named `PostClear`.
   * Git users: Clone directly (creates the `PostClear` folder automatically):
     ```bash
     git clone https://github.com/vfurman-gh/PostClear.git
     ```
2. **Add to Fusion 360:** Open Fusion 360 and go to Utilities → ADD-INS → Scripts and Add-Ins (shortcut: Shift+S). Switch to the Add-Ins tab, click `+` next to My Add-Ins, and select your `PostClear` folder.
3. **Enable and run:** Check Run on Startup and click Run.

On first use, the add-in uses Fusion's built-in Python to download its verification libraries (NumPy, Shapely, and Matplotlib) into `PostClear/packages/`. This takes roughly 5–10 seconds once, and automatically refreshes itself whenever Autodesk updates Fusion's internal Python version.

* **Update:** Replace the folder contents with the new release (keep `PostClear.settings` if present), or run `git pull`, then restart Fusion or Stop/Run the add-in in Scripts and Add-Ins.
* **Uninstall:** Open Scripts and Add-Ins (Shift+S) → Add-Ins, click Stop, remove PostClear from My Add-Ins, and delete the folder.
* **Offline first run:** Milling falls back to basic clearance checks without plots; connect to the internet once so full 3D/2D simulation and plotting libraries can download.

---

## Usage

### 1. Launch Post Clear

Before running Post Clear, create an **NC Program** in Fusion 360 configured with at least a valid post processor and output folder/filename.

Then start the command from either place in the **Manufacture** workspace:

* **Toolbar or Actions menu:** Click **Post Clear** on the Manufacture toolbar (right next to Fusion's built-in Post Process button).
* **Right-click context menu:** Right-click an individual **NC Program**, the **NC Programs** folder, or a **Setup** in the browser tree and choose **Post Clear** right below Post Process. The dialog opens with the matching NC Programs already checked.

### 2. Choose NC Programs & Settings

The dialog lists every NC Program in the active document with its target setup and output filename.

* **Select programs:** Check individual NC Programs or toggle **Select All** in the top-left header cell.
* **Restore rapid moves (G0):** Enabled by default. Promotes verified non-cutting feed moves back to G0 rapids when safe.
* **Produce debug log:** Disabled by default. Check this option to write troubleshooting diagnostics (including Fusion 360, Python, and library versions) to `postclear_debug.log` in your system temp folder.
* **Output naming:** Each NC Program writes to its configured output filename and folder.
* **Batch pre-checks:** Before posting starts, the add-in warns if any operations have empty or invalid toolpaths and asks once before overwriting existing NC files.

### 3. Review Verification Results

When posting finishes, a notification card appears in the corner of the Fusion window with one row per posted program and quick-open buttons for the generated **NC** file, Markdown verification **Report**, high-resolution **Plot**, and output **Folder**.

---

## What Gets Verified

| Capability | Milling (3D Heightfield) | Turning (2D Profile & Chuck) |
| :--- | :--- | :--- |
| **Supported stock** | Box, cylinder, tube, CAD solid/mesh body, and in-process stock from a preceding setup | Round bar/tube, revolved CAD solid, and in-process stock from a preceding setup |
| **Collision checks** | Rapid into uncut stock, toolholder strike, non-cutting shank rub | Rapid into uncut stock, insert/holder gouge, chuck jaw & fixture collision |
| **Safe G0 promotion** | Air moves ≥ 1.5 mm with full cutter and holder clearance above current stock | Air moves ≥ 1.5 mm clearing both the evolving 2D stock profile and chuck |
| **Automatic safety holds** | Never promotes inside canned cycles, active cutter compensation, or dog-leg rapid posts | Never promotes during threading/synchronized moves, canned cycles, or cutter compensation |
| **Load & Spindle checks** | Flags abnormal material removal rate (MRR) spikes per operation | Models constant surface speed (G96) RPM clamping and per-operation MRR spikes |

---

## How It Works

![PostClear pipeline](resources/pipeline.svg)

*High-level verification and posting flow for each setup.*

Internally, the add-in runs a two-pass pipeline around your machine-specific (Tormach, Haas, Centroid, LinuxCNC, GRBL, Mach3, and others) Fusion post processor:

* **Pass 1 — Capture each tool's motion and stock:** Posts operations one tool at a time under the hood to extract exact toolpath moves, tool assemblies, and stock boundaries from Fusion CAM, then merges them in order into one continuous job.
* **Simulation — Verify clearances across tool changes:** Tracks material removal move by move across every tool for both 3D milling and 2D turning, flagging collisions and marking air moves that clear the remaining stock and fixtures.
* **Pass 2 — Replay your unmodified post processor once:** Feeds the merged multi-tool sequence through your vendor post in one run to emit a single NC file with machine-specific tool changes (M6).

---

## Acknowledgments

Inspired by Tim Paterson's original [PostProcessAll](https://github.com/TimPaterson/Fusion360-Batch-Post) Fusion 360 add-in, which pioneered bringing multi-tool posting and rapid restoration to Fusion CAM users.
