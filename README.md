# isel iMC-S8 Controller

## Overview
This is a cross-platform Python application designed to control the **isel iMC-S8** stepper motor controller via serial communication. It features a rich Graphical User Interface (GUI) for manual and automated control, along with a built-in TCP/IP API server that allows external programs (like MATLAB, LabVIEW, or other external scripts) to orchestrate machine movements.

## Features
- **Serial Communication:** Reliable command queuing, background hardware handshaking, and detailed error handling. It automatically attempts to install missing dependencies (`pyserial`).
- **Control Panel:** Allows for manual jogging, absolute positioning, axis initialization, and homing (reference search) complete with soft-limit safety checks.
- **Grid Automation:** Define multi-dimensional measurement or routing grids (1D to 3D) using zig-zag or typewriter patterns. Includes an interactive 2D top-down trajectory preview.
- **Configuration & Calibration:** Adjustable steps-per-mm calibration and software limits to prevent hardware collisions. Settings are saved persistently.
- **Event Logging:** Real-time logging of movements, commands, and errors alongside current coordinate tracking. Fully exportable to CSV.
- **TCP API Server:** Control the machine programmatically using predefined JSON commands over a TCP socket (port 5000). Includes automated export of API documentation and a ready-to-use MATLAB wrapper.

## Installation and Requirements
- **Python 3.x**
- No manual package installation is strictly necessary—the application will check for `pyserial` on startup and dynamically install it if missing.

To run the application:
```bash
python isel_imc_s8_controller.py
```

---

## User Guide

### 1. Connecting to the Controller
1. Open the app and navigate to the **Control Panel** tab.
2. Select the correct COM port from the drop-down menu (click **Refresh** if your device was just plugged in).
3. Click **Connect**. The system will attempt to connect and automatically initialize the axes.

### 2. Initialization and Homing (Required)
Before making precise movements, the controller must be initialized and homed to establish a physical baseline for `0, 0, 0`.
1. Click **1. Init Axes (XYZ)**: This prepares the controller and forces it into 3D interpolation mode.
2. Click **2. Home (Reference)**: This drives the machine axes to their physical limit switches to zero its internal coordinates. Be sure the **Home Speed** and **Accel** values are set appropriately for your hardware before homing.

### 3. Manual Movement
From the **Control Panel** tab:
- **Manual Jog (Relative):** Input a *Step Size* and *Speed*, then click the X, Y, or Z directional buttons to step the machine relative to its current location.
- **Move to Position (Absolute):** Enter precise absolute coordinates for X, Y, and Z, set the speed, and click **Go To Position**.
- **STOP:** An emergency stop button is always visible. Clicking it immediately interrupts all active queues and stops hardware movement.

### 4. Grid Automation
The **Grid Automation** tab allows you to configure automated multi-point routing or scanning sequences.
1. Enter the **Start Point**, **Spacing**, and **Number of Points** for each axis (X, Y, Z).
2. Configure **Routing & Dynamics**:
   - **Fastest / Middle / Slowest Axis:** Assigns loop priority. The machine will finish the fastest axis loop before stepping the middle axis, and finish the middle loop before stepping the slowest axis.
   - **Pattern:** Select *Zig-Zag* (snaking over and back) or *Typewriter* (always returning to the start coordinate of the fast axis line).
   - **Wait at Point:** Set a dwell time (in seconds) at each coordinate, useful for reading external measurement sensors.
3. Review the **Path Preview** window to verify the XY traversal path.
4. Click **▶ Start Grid** to execute the sequence. You can track progress visually on the progress bar.

### 5. Calibration and Settings
The **Settings** tab lets you match the software to the mechanical properties of your machine.
- **Steps per mm:** Defines how many motor steps equal 1 mm of physical travel. If you don't know this, use the provided **Calibration Tool** to have the app calculate it based on a test movement and a measured ruler distance.
- **Soft Limits:** Define a warning boundary (bounding box). If a manual or API command targets coordinates outside this box, you'll be warned.
- **Apply & Save Settings:** Saves your inputs so they persist to your next session.

### 6. Event Log
The **Event Log** tab tracks API requests, queue entries, position arrivals, and hardware errors. If you need to keep a record of an automated grid run, hit **Export Log to CSV**.

---

## API and MATLAB Integration
To allow easy automation in scientific environments, the app acts as a local server (listening on `0.0.0.0:5000`).

### Using MATLAB
You can dynamically control the machine from MATLAB without worrying about the low-level serial protocol. The repository includes an `isel_cmd.m` wrapper script for immediate use.

1. Ensure the provided `isel_cmd.m` file is in your MATLAB working directory. *(Note: You can also export this script directly from the application's **Help & API** tab).*
2. In MATLAB, run the commands directly:

```matlab
% Wake up and Home
[is_ok, msg] = isel_cmd('init', 'wait_ready', true);
[is_ok, msg] = isel_cmd('home', 'wait_ready', true);

% Move Absolute
[is_ok, msg, pos_data] = isel_cmd('move_abs', 'x', 50, 'y', 20, 'speed', 10, 'wait_ready', true);

% Read current position without moving
[is_ok, msg, pos_data] = isel_cmd('get_pos');
disp(['Current X: ', num2str(pos_data.pos.x)]);
```

### API Protocol Specification
You can send JSON strings followed by a newline (`\n`) to `localhost:5000` via any language (Python, C#, LabVIEW). 

#### Example Request:
```json
{
  "cmd": "move_abs",
  "x": 150.0,
  "speed": 15.0,
  "accel": 500.0,
  "wait_ready": true
}
```

Detailed JSON parameters for `init`, `home`, `grid`, `move_rel`, `move_abs`, and `stop` can be viewed inside the app under the **Help & API** tab. Using **Export API Docs...** will generate a standalone text file with all endpoint parameters.