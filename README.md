# PostClear — Fusion 360 CAM Batch Post & Clearance Verification

![PostClear](resources/Command/32x32.png)

Fusion 360 Personal Edition CAM users run into two recurring headaches at the post-processing stage:

* **Personal Edition removes rapid moves:** Fusion converts rapid positioning moves (G0) into linear feed moves, slowing air travel across the job. Simple text-replacement scripts that blindly turn high-feed moves back into rapids risk driving a cutter or toolholder straight into uncut stock or a chuck jaw.
* **Multi-setup jobs require repetitive posting:** Parts with multiple setups (Op 1, Op 2, milling plus turning) force you to post each NC Program one at a time and check every output file by hand.

**PostClear batch-posts one or many NC Programs in a single step, simulates the evolving stock to catch collisions, and restores G0 rapids only where the tool and holder are proven to clear the workpiece.**

---

## Installation (Windows & macOS)

No separate Python installation or command-line setup is needed.

1. **Download and extract:** Download `PostClear-v3.0.0.zip` from **Releases** and unzip the `PostClear` folder wherever you keep Fusion add-ins (for example, `Documents/FusionAddIns/PostClear`). Keep the folder named **PostClear**.
   * *Git users:* Clone directly (creates the `PostClear` folder automatically):
     ```bash
     git clone https://github.com/vfurman-gh/PostClear.git
     ```
2. **Add to Fusion 360:** Open Fusion 360 and go to **Utilities → ADD-INS → Scripts and Add-Ins** (shortcut: **Shift+S**). Switch to the **Add-Ins** tab, click **`+`** next to *My Add-Ins*, and select your `PostClear` folder.
3. **Enable and run:** Check **Run on Startup** and click **Run**.

On first use, the add-in uses Fusion's built-in Python to download its verification libraries (NumPy, Shapely, and Matplotlib) into `PostClear/packages/`. This takes roughly 5–10 seconds once, and automatically refreshes itself whenever Autodesk updates Fusion's internal Python version.

| Task | Steps |
| :--- | :--- |
| **Update** | Replace the folder contents with the new release (keep `PostClear.settings` if present), or run `git pull`, then restart Fusion or Stop/Run the add-in in **Shift+S**. |
| **Uninstall** | Open **Scripts and Add-Ins** (**Shift+S**) → **Add-Ins**, click **Stop**, remove **PostClear** from *My Add-Ins*, and delete the folder. |
| **Offline first run** | Milling falls back to basic clearance checks without plots; connect to the internet once so full 3D/2D simulation and plotting libraries can download. |

---

## Usage

### 1. Launch Post Clear

You can start the command from two places in the **Manufacture** workspace:

* **Toolbar or Actions menu:** Click **Post Clear** on the Manufacture toolbar (right next to Fusion's built-in Post Process button).
* **Right-click context menu:** Right-click an individual **NC Program**, the **NC Programs** folder, or a **Setup** in the browser tree and choose **Post Clear** right below Post Process. The dialog opens with the matching NC Programs already checked.

### 2. Choose NC Programs & Settings

The dialog lists every NC Program in the active document with its target setup and output filename. If the document has setups without an NC Program, the add-in creates a default NC Program for you.

* **Select programs:** Check individual NC Programs or toggle **Select All** in the top-left header cell.
* **Restore rapid moves (G0):** Enabled by default. Promotes verified non-cutting feed moves back to G0 rapids when safe.
* **Produce debug log:** Disabled by default. Check this option to write troubleshooting diagnostics (including Fusion 360, Python, and library versions) to `postclear_debug.log` in your system temp folder.
* **Output naming:** Each NC Program writes to its configured output filename and folder.
* **Batch pre-checks:** Before posting starts, the add-in checks for missing toolpaths, regenerates out-of-date operations if needed, and asks once before overwriting existing NC files.

### 3. Review Verification Results

When posting finishes, a compact notification card appears in the corner of the Fusion window with one row per posted program:

* **Quick-open buttons:** Open the generated **NC** file, the Markdown verification **Report**, the high-resolution toolpath/stock **Plot**, or the output **Folder** in one click.
* **Stay out of the way when clean:** If every program posts cleanly with no warnings, the notification closes itself after 2 seconds (clicking anywhere on the card keeps it open).
* **Stay visible on issues:** If a collision or spike warning is detected, the notification stays on screen until you dismiss it.

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

```mermaid
flowchart LR
    A["Selected NC Programs"] --> B["Capture Toolpaths & Stock"]
    B --> C["Simulate Stock & Collisions"]
    C --> D["Replay Your Post (.cps)"]
    D --> E["NC Code + Report & Plot"]

    style A fill:#f6f8fa,stroke:#57606a,color:#24292f
    style B fill:#f6f8fa,stroke:#57606a,color:#24292f
    style C fill:#ddf4ff,stroke:#0969da,color:#0a3069,stroke-width:2px
    style D fill:#f6f8fa,stroke:#57606a,color:#24292f
    style E fill:#f6f8fa,stroke:#57606a,color:#24292f
```
*High-level verification and posting flow for each setup (excludes Fusion UI dialog and file pre-checks).*

Instead of editing G-code with fragile regular expressions, the add-in runs a two-pass workflow around your existing Fusion post processor:

* **Pass 1 — Capture exact motion and stock:** Extracts every toolpath move, tool assembly, and stock boundary (box, round bar, CAD solid, or leftover stock from the previous setup) directly from Fusion CAM.
* **Simulation — Verify clearances against live stock:** Tracks material removal move by move for both 3D milling and 2D turning, flagging collisions and marking air moves that safely clear the remaining stock and fixtures.
* **Pass 2 — Run your unmodified post processor:** Replays your chosen vendor post processor (Tormach, Haas, Centroid, LinuxCNC, GRBL, Mach3, and others) once across the setup so tool changes, modal states, and machine formatting come straight from your post.

---

## Acknowledgments

Inspired by Tim Paterson's original [PostProcessAll](https://github.com/TimPaterson/Fusion360-Batch-Post) Fusion 360 add-in, which pioneered bringing batch setup posting and rapid restoration to Fusion CAM users.
