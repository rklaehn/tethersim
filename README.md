# tethersim

A real-time mass-spring simulation of a rotating lunar tether system, built with
Bevy and egui.

## Approach

The tether is discretized as a chain of point masses connected by tension-only
springs. Each segment derives its mass and spring constant from the material
properties (density ρ, Young's modulus E, cross-sectional area A) of the
corresponding tether section.

An exponential taper profile keeps stress approximately constant along the
tether:

    A(r) = A_tip · exp(ρ·ω²·(R_tip² - r²) / (2·σ₀))

Time integration uses classical 4th-order Runge-Kutta (RK4) with configurable
sub-stepping for stability.

The simulation covers:

- **Settling**: initialization with increased damping until steady-state rotation
- **Instant release**: payload mass removed instantaneously (demonstrates stress
  wave failure)
- **Soft release**: cubic reel-out (l = c·t³) with brake-controlled force ramp-down
- **Active damping**: hub reel feedback control to absorb longitudinal and lateral
  oscillations post-release

## Structure

```
src/
  main.rs          – Bevy app setup and plugin registration
  physics.rs       – simulation state, RK4 integrator, material database,
                     taper computation, spring breaking, soft release, hub damper
  visualization.rs – 3D rendering, stress monitor, camera, payload tracking,
                     video recording
  gui.rs           – egui control panel and info display
assets/
  textures/moon.jpg
```

## Dependencies

| Crate       | Purpose                                      |
|-------------|----------------------------------------------|
| bevy        | ECS framework, 3D rendering, gizmos, input   |
| bevy_egui   | Immediate-mode GUI for parameter controls     |
| openh264    | H.264 video encoding for simulation recording |
| minimp4     | MP4 container muxing                          |

## Materials

Six tether materials with real physical properties:

| Material          | E (GPa) | ρ (kg/m³) | σ_ult (GPa) |
|-------------------|---------|-----------|-------------|
| Kevlar 49         | 112     | 1440      | 3.6         |
| Dyneema SK78      | 120     | 970       | 3.6         |
| Dyneema SK99      | 150     | 970       | 4.1         |
| Carbon Fiber T700 | 230     | 1800      | 4.9         |
| Zylon             | 180     | 1560      | 5.8         |
| Steel             | 200     | 7800      | 2.0         |

## Usage

```
cargo run --release
```

Left panel: simulation controls, tether parameters, material selection, release
settings, hub damper config.

Right panel: live readouts (tip velocity, kinetic energy, stress monitor).

Press **G** to toggle video recording. Auto-record captures a window around the
release event.

## Getting started (no Rust experience needed)

1. Install Rust: https://rustup.rs/
2. Clone and run:
   ```
   git clone <this repo>
   cd tethersim
   cargo run --release
   ```
   The first build takes a few minutes. Subsequent runs are fast.

## Blog post

[Lunar Sling Launcher - Tether Dynamics](https://blog.klaehn.org/blog/lunar-sling-launcher-tether-dynamics/)

## Credits

Created by [Rüdiger Klaehn](https://blog.klaehn.org/) using [Claude](https://claude.ai/).
