# REBUILT 2026

## Contributing, Conforming and Terms

Visit [Contributing.md](./CONTRIBUTING.md)

## Design Doc
At this [Google Doc](https://docs.google.com/document/d/1LHVBFvE4I9V1lK-kII-BY99baFRJSkxOhEw3NQdT6d4/edit?usp=sharing).

This document contains all specifications and requirements for the program.

## Robot Initialization State
Robot must be in a predetermined position for positions to correctly initialize.

- **Turret must be within about 21 degrees of the stow angle of 0 degrees (aiming forward towards intake)**
    - Failure to do so leads to turret being off by the closest multiple of 42.3 degrees, requiring turret calibration in dashboard
- Intake must be stowed up against the hardstop
- Hood must be stowed down against the hardstop; gravity can usually achieve this

## Controls
Hardware: Xbox Controller or equivalent

### Drive
- Move - `Left Joystick`
    - field-oriented, operator (alliance) perspective
- Rotate - `Right Joystick` (horizontal direction)
- Reset robot heading to 0 - Press `Right Joystick Button`

### Stow
- Stow robot - Press `Start`
    - Stow state deactivates when any subsystem is activated

### Intake
D-pad

- Stow intake and rollers off - Press `Up`
- Deploy intake and rollers on - Press `Down`
- Deploy intake and rollers off - Press `Right`
- Reverse intake rollers - Hold `Left`
    - Upon finishing, start rollers

### Launcher
- Automatic target aiming and launching - On by default
- Stow launcher - Hold `Left Trigger`
- Fixed launching - Hold `Left Bumper`

#### Feeder
- Start feeding override - Hold `Right Trigger`
- Stop feeding override - Hold `Right Bumper`

### Fuel Unjamming
- Intake fuel unjamming - Hold `A`
    - Moves intake up while running intake in reverse
    - Upon finishing, deploy intake and start rollers
- Hopper/launcher fuel unjamming - Hold `B`
    - Runs both intake and spindexer in reverse

### Spindexer
- Always on unless robot is stowed

## Trajectory Calculations and Math
See [trajectory.pdf](./trajectory/trajectory.pdf).

## Notice: Using Prewritten Code
The prewritten code must be available public, on a public forum post such as Chief Delphi,
before kickoff. From R303 of (2026) Game Manual.

## Xbox Controller Keyboard Mappings
What each control is in SmartDashboard, in Keyboard Settings.

Set Keyboard 0 controls to these when you need each specific control.

Axes (6):
| Axis # | Description | Increase+ | Decrease- | Key Rate | Decay Rate |
| - | - | - | - | - | - |
| 0 | Left stick X axis | `d` | `a` | ~0.050 | ~0.050 |
| 1 | Left stick Y axis | `s` | `w` | ~0.050 | ~0.050 |
| 2 | Left trigger | `q` | (None) | 1.000 | 1.000 |
| 3 | Right trigger | `o` | (None) | 1.000 | 1.000 |
| 4 | Right stick X axis | `l` | `j` | ~0.050 | ~0.050 |
| 5 | Right stick Y axis | `k` | `i` | ~0.050 | ~0.050 |

(All axes have Max Absolute Value as 1)

Buttons (10):
| Button # | Description | Control |
| - | - | - |
| 1 | A button | `g` |
| 2 | B button | `h` |
| 3 | X button | `f` |
| 4 | Y button | `t` |
| 5 | Left Bumper | `e` |
| 6 | Right Bumper | `u` |
| 7 | Back button | `v` |
| 8 | Start button | `b` |
| 9 | Left stick button | `x` |
| 10 | Right stick button | `m` |

POVs (1): \
POV 0:
| Degrees | Description | Control |
| - | - | - |
| 0 | Up | `Up` |
| 45 | Up-Right | (None) |
| 90 | Right | `Right` |
| 135 | Down-Right | (None) |
| 180 | Down | `Down` |
| 225 | Down-Left | (None) |
| 270 | Left | `Left` |
| 315 | Up-Left | (None) |

These only go by angles; avoid pressing more than one button at once.

### Sources
- https://hal7df.github.io/pauls-tutorials/wpi/ch2/user-input.html
- https://docs.wpilib.org/en/stable/docs/software/basic-programming/joystick.html
- https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#joystick-and-controller-coordinate-system

## Gradle Commands
Formatting:
```
./gradlew spotlessApply
```

## Generated, Templated, and Imported Code
- `TunerConstants.java` is entirely auto-generated
- `CommandSwerveDrivetrain` is entirely from a template
- `*IO.java` is most copied and adapted to our drivebase from a template provided by AdvantageKit

## Attribution
LLMs (GitHub Copilot, Sonnet4.6) may sometimes be used for autocomplete and documentation, vibecoding is highly condemned.
