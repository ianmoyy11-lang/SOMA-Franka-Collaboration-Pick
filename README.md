# Isaac Sim 6.0.1, Franka, and SOMA-X Deployment Guide
This guide explains how to do the following on Windows:

1. Install the ZIP version of Isaac Sim 6.0.1.
2. Run the official Franka manipulator.
3. Download SOMA-X.
4. Import the human asset included with SOMA-X into Isaac Sim.
5. Run a collaborative pick-and-place program with Franka and the human model.

---

## Contents
+ [1. Project Overview](#1-project-overview)
+ [2. System Requirements](#2-system-requirements)
+ [3. Install the Isaac Sim ZIP Package](#3-install-the-isaac-sim-zip-package)
+ [4. Deploy the Franka Manipulator](#4-deploy-the-franka-manipulator)
+ [5. Download SOMA-X](#5-download-soma-x)
+ [6. Use the Human Asset Included with SOMA-X](#6-use-the-human-asset-included-with-soma-x)
+ [7. Configure the Human Asset Environment Variable](#7-configure-the-human-asset-environment-variable)
+ [8. Run the Complete Program](#8-run-the-complete-program)
+ [9. Program Features](#9-program-features)
+ [10. References](#10-references)

---

# 1. Project Overview
This project creates a basic human-robot collaboration scene in Isaac Sim.

The scene contains:

+ A Franka Panda manipulator.
+ A dynamic cube.
+ A SOMA-X human model.
+ A finite-state-machine controller.

The manipulator can:

+ Read the cube position.
+ Read the human position.
+ Estimate the center of the human's feet.
+ Grasp the cube.
+ Place the cube beside the human's feet.
+ Update the target after the human moves.
+ Retry automatically after a failed grasp.
+ Return smoothly to its initial pose after completing the task.

The current system reads simulation states directly from Isaac Sim and does not use a camera.

---

# 2. System Requirements
| Item | Requirement |
| --- | --- |
| Operating system | Windows 10 or Windows 11 |
| Isaac Sim | Version 6.0.1 ZIP package |
| Python | Python included with Isaac Sim |
| Manipulator | Franka Emika Panda |
| Human model | SOMA-X |
| Scene format | OpenUSD |
| Physics engine | NVIDIA PhysX |
| Control method | Inverse kinematics and a finite-state machine |

> [!IMPORTANT]  
> This project targets Isaac Sim 6.0.1. Do not mix it with legacy APIs from Isaac Sim 4.x or 5.x.
>

The project uses:

```python
from isaacsim.robot.experimental.manipulators.examples.franka import Franka
```

---

# 3. Install the Isaac Sim ZIP Package
## 3.1 Download Isaac Sim
Open the official NVIDIA download page:

[Isaac Sim 6.0.1 Download](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/download.html)

Download the Windows ZIP package.

The file name is usually similar to:

```latex
isaac-sim-standalone-6.0.1-windows-x86_64.zip
```

Prepare at least 50 GB of free disk space.

---

## 3.2 Create the Installation Directory
This guide uses:

```latex
D:\IsaacSim
```

Run the following in PowerShell:

```powershell
mkdir D:\IsaacSim
```

---

## 3.3 Extract Isaac Sim
You can use 7-Zip or run:

```powershell
tar -xf "$env:USERPROFILE\Downloads\isaac-sim-standalone-6.0.1-windows-x86_64.zip" -C D:\IsaacSim
```

After extraction, the directory should contain:

```latex
D:\IsaacSim
├── apps
├── exts
├── kit
├── standalone_examples
├── isaac-sim.bat
├── post_install.bat
└── python.bat
```

---

## 3.4 Run the Installation Script
```powershell
cd D:\IsaacSim
.\post_install.bat
```

---

## 3.5 Check Computer Compatibility
```powershell
cd D:\IsaacSim
.\isaac-sim.compatibility_check.bat
```

This tool checks the GPU, driver, VRAM, CPU, system memory, and disk space.

---

## 3.6 Start Isaac Sim
```powershell
cd D:\IsaacSim
.\isaac-sim.bat
```

The first startup may take several minutes because Isaac Sim must initialize extensions, PhysX, and RTX shaders.

---

# 4. Deploy the Franka Manipulator
## 4.1 Add Franka Through the Interface
After starting Isaac Sim, select:

```latex
Create
└── Robots
    └── Franka Emika Panda Arm
```

Click the Play button and verify that the manipulator loads correctly.

---

## 4.2 Run the Official Example
Run the following in PowerShell:

```powershell
cd D:\IsaacSim

.\python.bat standalone_examples\api\isaacsim.robot.experimental.manipulators\franka\pick_place.py
```

The example includes:

+ Franka creation.
+ Gripper control.
+ Cube creation.
+ Pick-and-place motion.

---

## 4.3 Franka API Used by This Project
Enable the extension:

```python
import isaacsim.core.experimental.utils.app as app_utils

app_utils.enable_extension(
    "isaacsim.robot.experimental.manipulators.examples"
)
```

Import Franka:

```python
from isaacsim.robot.experimental.manipulators.examples.franka import Franka
```

---

# 5. Download SOMA-X
## 5.1 Official Resources
+ [SOMA-X GitHub](https://github.com/NVlabs/SOMA-X)
+ [SOMA-X Documentation](https://nvlabs.github.io/SOMA-X/stable/)
+ [SOMA-X Hugging Face](https://huggingface.co/nvidia/SOMA-X)

---

## 5.2 Download the ZIP Package
You can download it directly:

[Download SOMA-X ZIP](https://github.com/NVlabs/SOMA-X/archive/refs/heads/main.zip)

Extract it to:

```latex
C:\Projects\SOMA-X
```

> [!WARNING]  
> SOMA-X uses Git LFS. The GitHub ZIP package may not contain all large model files, so Git LFS is recommended.
>

---

## 5.3 Download with Git LFS
After installing Git, run:

```powershell
git lfs install
```

Create the project directory and clone the repository:

```powershell
mkdir C:\Projects
cd C:\Projects

git clone https://github.com/NVlabs/SOMA-X.git
cd SOMA-X
git lfs pull
```

The resulting directory is:

```latex
C:\Projects\SOMA-X
```

---

## 5.4 Install the SOMA-X Environment
Keep the SOMA-X environment separate from the Isaac Sim environment.

Enter the project directory:

```powershell
cd C:\Projects\SOMA-X
```

Create and activate a virtual environment:

```powershell
pip install uv
uv venv .venv
.\.venv\Scripts\activate
```

Install the project:

```powershell
uv pip install ".[dev]"
uv pip install ".[demo]"
```

You may also install the PyPI package:

```powershell
pip install py-soma-x
```

---

# 6. Use the Human Asset Included with SOMA-X
The SOMA-X project already includes a human USD asset that can be imported directly into Isaac Sim:

```latex
C:\Projects\SOMA-X\assets\SOMA_template_rig.usda
```

Therefore, this guide does not require generating another human USD or running a separate export program.

The Isaac Sim control script only needs to reference this file.

Check that the file exists:

```powershell
Test-Path "C:\Projects\SOMA-X\assets\SOMA_template_rig.usda"
```

The expected result is:

```latex
True
```

You can also inspect the file:

```powershell
Get-Item "C:\Projects\SOMA-X\assets\SOMA_template_rig.usda"
```

The project structure should be similar to:

```latex
C:\Projects\SOMA-X
├── assets
│   └── SOMA_template_rig.usda
├── soma
├── tools
└── outputs
```

> [!NOTE]  
> `SOMA_template_rig.usda` is the human model used in this guide. The program loads it into Isaac Sim through a USD reference.
>

---

# 7. Configure the Human Asset Environment Variable
The program should read the human asset path from the `SOMAX_FOLDER` environment variable.

Replace the path configuration in the program with:

```python
import os
from pathlib import Path

somax_path = os.environ.get("SOMAX_FOLDER")

if not somax_path:
    raise RuntimeError(
        "The SOMAX_FOLDER environment variable is not set.\n"
        "Before running the program, set SOMAX_FOLDER to "
        "the SOMA-X human USD file or the directory containing it."
    )

SOMAX_FOLDER = Path(somax_path).expanduser().resolve()

if not SOMAX_FOLDER.exists():
    raise FileNotFoundError(
        f"The path specified by SOMAX_FOLDER does not exist: "
        f"{SOMAX_FOLDER}"
    )
```

`SOMAX_FOLDER` may point to the USD file directly:

```latex
C:\Projects\SOMA-X\assets\SOMA_template_rig.usda
```

It may also point to a directory containing a `.usd`, `.usda`, or `.usdc` file.

The `find_somax_usd()` function supports both a directory and a specific USD file.

> [!IMPORTANT]  
> You must set `SOMAX_FOLDER` in the current command-line terminal before running the program.  
> Set the environment variable and launch the program from the same terminal window.
>

PowerShell:

```powershell
$env:SOMAX_FOLDER = "C:\Projects\SOMA-X\assets\SOMA_template_rig.usda"
```

CMD:

```cmd
set "SOMAX_FOLDER=C:\Projects\SOMA-X\assets\SOMA_template_rig.usda"
```

Check the environment variable:

PowerShell:

```powershell
$env:SOMAX_FOLDER
```

CMD:

```cmd
echo %SOMAX_FOLDER%
```

---

## 7.1 Human Binding Paths in Isaac Sim
The program creates:

```latex
/World/SOMAX
└── Model
```

The corresponding settings are:

```python
SOMAX_ROOT_PATH = "/World/SOMAX"
SOMAX_MODEL_PATH = "/World/SOMAX/Model"
```

Where:

+ `/World/SOMAX` controls the position of the complete human asset.
+ `/World/SOMAX/Model` references `SOMA_template_rig.usda`.
+ The original USD file is not modified directly.

The program binds the human asset with:

```python
stage_utils.add_reference_to_stage(
    somax_usd_path,
    SOMAX_MODEL_PATH,
)
```

To move the human in Isaac Sim, select:

```latex
/World/SOMAX
```

---

# 8. Run the Complete Program
Save the complete script as:

```latex
C:\Projects\IsaacSim-SOMAX\collaboration_pick.py
```

The complete program is available at:

[SOMAX_FOOT_Eng.py](./SOMAX_FOOT_Eng.py)

Before running the program, set the `SOMAX_FOLDER` environment variable.

> [!IMPORTANT]  
> If `SOMAX_FOLDER` is not set, the program will report that the environment variable is missing during startup.  
> Set the environment variable first, and then run the script with the Python included with Isaac Sim.
>

---

## 8.1 Run with Isaac Sim Python
Do not use the system Python interpreter:

```powershell
python collaboration_pick.py
```

Use the `python.bat` included with Isaac Sim.

### PowerShell

```powershell
$env:SOMAX_FOLDER = "C:\Projects\SOMA-X\assets\SOMA_template_rig.usda"

cd D:\IsaacSim

.\python.bat C:\Projects\IsaacSim-SOMAX\collaboration_pick.py
```

### CMD

```cmd
set "SOMAX_FOLDER=C:\Projects\SOMA-X\assets\SOMA_template_rig.usda"

cd /d D:\IsaacSim

python.bat C:\Projects\IsaacSim-SOMAX\collaboration_pick.py
```

The environment variable is valid only for the current terminal session. After closing the terminal, set it again before the next run.

---

## 8.2 Expected Terminal Output
After the program starts normally, the terminal should show:

```latex
[SOMA-X] Selected asset:
C:/Projects/SOMA-X/assets/SOMA_template_rig.usda

[SOMA-X] Initial feet center: [...]

[TARGET] Initial target: [...]

SOMA-X and Franka collaboration system started.
```

When grasping begins:

```latex
[ACTION] The cube is not at the latest human target.
Priority 1: grasp the cube first.
```

After a successful grasp:

```latex
[GRASP VERIFIED]
Cube lift delta: ...
Active human target committed: ...
```

When the human moves:

```latex
[HUMAN TARGET OBSERVED]
Old: [...]
New: [...]
Shift: ... m
```

After completing the task:

```latex
[SUCCESS] The cube reached the committed foot-side target.
Starting smooth home return.

[HOME] Franka returned smoothly.
```

---

# 9. Program Features
## 9.1 Continuous Monitoring
The program continuously reads:

+ The cube's world position.
+ The SOMA-X world bounding box.
+ The estimated position of the human's feet.
+ The latest foot-side target.

---

## 9.2 Grasping Priority
The manipulator must grasp the cube first.

Before the grasp is verified, the human position is recorded but does not control the manipulator.

```latex
Detect the cube
→ Move to the cube
→ Close the gripper
→ Perform a test lift
→ Verify the grasp
→ Enable the human target
```

---

## 9.3 Dynamic Human Target
The program recalculates the human position every few frames:

```python
TARGET_UPDATE_INTERVAL = 5
```

After moving `/World/SOMAX`, the foot-side target is updated.

---

## 9.4 Grasping Stability
The program uses:

```python
CUBE_MASS = 0.05
CUBE_STATIC_FRICTION = 1.5
CUBE_DYNAMIC_FRICTION = 1.2
GRIP_CLOSE_STEPS = 140
```

It also includes:

+ Continuous gripper closing.
+ A test lift.
+ Grasp verification.
+ Drop detection.
+ Automatic retries.

---

## 9.5 Smooth Return
After completing the task, the manipulator returns to its initial pose through joint interpolation:

```python
HOME_RETURN_STEPS = 180
```

At 60 FPS, this takes approximately three seconds.

---

# 10. References
## NVIDIA Isaac Sim
+ [Isaac Sim 6.0.1 Download](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/download.html)
+ [Isaac Sim Quick Install](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/quick-install.html)
+ [Isaac Sim Workstation Installation](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/install_workstation.html)
+ [Franka Pick-and-Place Example](https://docs.isaacsim.omniverse.nvidia.com/latest/examples/manipulation_franka_pick_place.html)

## SOMA-X
+ [SOMA-X GitHub](https://github.com/NVlabs/SOMA-X)
+ [SOMA-X ZIP](https://github.com/NVlabs/SOMA-X/archive/refs/heads/main.zip)
+ [SOMA-X Documentation](https://nvlabs.github.io/SOMA-X/stable/)
+ [SOMA-X Hugging Face](https://huggingface.co/nvidia/SOMA-X)

---

# License
The SOMA-X code is licensed under the Apache-2.0 License.

SMPL, SMPL-X, and other human-body models may use separate licenses.

Do not upload restricted model files to a public GitHub repository.
