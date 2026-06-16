# ROV de GO

Unity HDRP based ROV simulator with tether dynamics, underwater visibility presets, sonar simulation, and gamepad/joystick/keyboard control.

## Overview

ROV de GO is a Unity project for testing and visualizing small ROV operation in underwater environments. The current simulation focuses on:

- ROV thrust control with gamepad, joystick, keyboard, or MAVLink `MANUAL_CONTROL` input.
- XPBD tether simulation with collision, tension feedback, current load, and selectable drag model.
- Legacy coefficient drag and Morison-style hydrodynamic drag for tether nodes.
- Imaging sonar and virtual Oculus M750d-style sonar views.
- Environment presets for water visibility, current, time of day, suspended matter, and worksite debris.

## Unity Version

Open the project with:

```text
Unity 6000.3.9f1
```

The project uses HDRP `17.3.0` and the Unity Input System.

## Main Scenes

Primary scenes are under `Assets/Scenes`.

| Scene | Purpose |
| --- | --- |
| `EmptyMenu.unity` | Startup/menu entry scene. |
| `U_Boat.unity` | Main underwater ROV scene. |
| `UnderwaterStructure.unity` | Cave/structure style worksite scene. |
| `mine_s.unity` | Mine/worksite scene. |
| `WaveEvaluation_U_Boat.unity` | Wave and water-surface behavior evaluation scene. |
| `SampleScene.unity` | General sample scene. |

For normal operation, start from `Assets/Scenes/EmptyMenu.unity` and select the target scene in the startup menu.

## Controls

The startup menu can select `Gamepad`, `Joystick`, or `Keyboard`.

### Gamepad

- Left stick: move.
- Right stick: yaw / heave.
- LB / RB: camera tilt.
- LT / RT: gripper close / open.
- D-pad left / right: light intensity.
- D-pad up / down: control gain.
- Y: heading lock.
- X: altitude hold.
- B: auto tether pay.
- A / Start: menu submit.
- Start: open settings.

### Keyboard

- `W` / `S`: forward / back.
- `A` / `D` or arrow left / right: lateral movement.
- `Q` / `E`: yaw.
- `R` or `Space`: up.
- `F`: down.
- Up / PageUp: camera tilt up.
- Down / PageDown: camera tilt down.
- `Z`: gripper close.
- `X`: gripper open.
- `H`: heading lock.
- `J`: altitude hold.
- `K`: auto tether pay.
- `[` / `]`: light darker / brighter.
- `-` / `=`: control gain lower / higher.
- `Esc`: settings.
- `F5` / `F6` / `F7`: visibility Clear / Normal / Murky.

## Tether Simulation

The active tether implementation is in:

```text
Assets/script/xpbd/CableXPBD.cs
Assets/script/xpbd/CableSonarColliders.cs
```

Important inspector settings:

- `hydrodynamicDragModel`
  - `LegacyCoefficients`: existing coefficient-based drag.
  - `Morison`: drag based on water density, cable diameter, normal/tangential coefficients, and node tributary length.
- `targetSegmentLength`
  - Current prefab target is `0.5 m`.
- `maxHydrodynamicAcceleration`
  - Limits one-step drag acceleration to avoid numerical blow-up.
- `maxCableNodeSpeed`
  - Caps node velocity after integration and constraint solving.
- `applyCurrentLoadToBottom`
  - Applies current-induced tether load back to the ROV attach point.

The shared ROV prefab is:

```text
Assets/Prefab/ROV_SYSTEM.prefab
```

Scene instances may still contain prefab overrides. If a scene behaves differently from the prefab, inspect the scene instance overrides first.

## Sonar

Main sonar scripts are under `Assets/script/Sonar`.

- `ImagingSonarSim.cs`: imaging sonar simulation.
- `VirtualOculusM750dTerrainSonar.cs`: virtual Oculus-style terrain sonar.
- `SeaTurbiditySelector.cs`: sonar turbidity preset handling.
- `SonarWaterColumnNoise.cs`: water-column noise.

The startup menu can switch between imaging sonar and virtual Oculus mode.

## MAVLink Input

`Assets/script/xpbd/MavlinkUdpInputReceiver.cs` listens for MAVLink `MANUAL_CONTROL` packets.

Default UDP port:

```text
14550
```

The receiver maps `x`, `y`, `z`, and `r` to surge, sway, heave, and yaw.

## Build / Check

From the repository root, C# compilation can be checked with:

```powershell
dotnet restore Assembly-CSharp.csproj
dotnet build Assembly-CSharp.csproj --no-restore
```

Unity generated folders such as `Library`, `Temp`, `Logs`, `UserSettings`, `Recordings`, and generated project files are ignored by Git.

## Repository Notes

- Keep Unity-generated cache folders out of commits.
- Prefer changing prefab defaults in `Assets/Prefab/ROV_SYSTEM.prefab` when the same ROV behavior should apply to all scenes.
- Check scene instance overrides when one scene behaves differently from another.
- Use focused commits for simulation code, prefab tuning, and scene tuning so changes are easier to compare.
