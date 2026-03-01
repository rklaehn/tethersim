//! Tether physics simulation: generic spring-mass system with RK4 integration.
//!
//! 3D simulation. Origin is at the surface below the hub.
//! Hub at (0, hub_height, 0). Moon center at (0, -MOON_RADIUS, 0).
//! The drive point (index 0) orbits in the XZ plane at y=hub_height.
//! Springs can break when stress exceeds the material's tensile strength.

use bevy::prelude::*;

/// Named tether materials with real physical properties.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TetherMaterial {
    Kevlar49,
    DyneemaSK78,
    DyneemaSK99,
    CarbonFiberT700,
    Zylon,
    Steel,
}

impl TetherMaterial {
    pub const ALL: &[TetherMaterial] = &[
        TetherMaterial::Kevlar49,
        TetherMaterial::DyneemaSK78,
        TetherMaterial::DyneemaSK99,
        TetherMaterial::CarbonFiberT700,
        TetherMaterial::Zylon,
        TetherMaterial::Steel,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Self::Kevlar49 => "Kevlar 49",
            Self::DyneemaSK78 => "Dyneema SK78",
            Self::DyneemaSK99 => "Dyneema SK99",
            Self::CarbonFiberT700 => "Carbon Fiber T700",
            Self::Zylon => "Zylon",
            Self::Steel => "Steel",
        }
    }

    /// Young's modulus (Pa).
    pub fn youngs_modulus(self) -> f64 {
        match self {
            Self::Kevlar49 => 112e9,
            Self::DyneemaSK78 => 120e9,
            Self::DyneemaSK99 => 150e9,
            Self::CarbonFiberT700 => 230e9,
            Self::Zylon => 180e9,
            Self::Steel => 200e9,
        }
    }

    /// Density (kg/m³).
    pub fn density(self) -> f64 {
        match self {
            Self::Kevlar49 => 1440.0,
            Self::DyneemaSK78 => 970.0,
            Self::DyneemaSK99 => 970.0,
            Self::CarbonFiberT700 => 1800.0,
            Self::Zylon => 1560.0,
            Self::Steel => 7800.0,
        }
    }

    /// Tensile strength (Pa).
    pub fn tensile_strength(self) -> f64 {
        match self {
            Self::Kevlar49 => 3.6e9,
            Self::DyneemaSK78 => 3.6e9,
            Self::DyneemaSK99 => 4.1e9,
            Self::CarbonFiberT700 => 4.9e9,
            Self::Zylon => 5.8e9,
            Self::Steel => 2.0e9,
        }
    }
}

/// Hub damper mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HubDamperMode {
    /// No hub damper.
    None,
    /// Active: adjust first spring rest_length to match expected centrifugal force.
    Active,
}

impl HubDamperMode {
    pub const ALL: &[HubDamperMode] = &[
        HubDamperMode::None,
        HubDamperMode::Active,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Self::None => "None",
            Self::Active => "Active",
        }
    }
}

/// Lunar surface gravity (m/s²).
pub const MOON_G: f64 = 1.625;
/// Moon radius (m).
pub const MOON_RADIUS: f64 = 1_737_400.0;
/// Gravitational parameter mu = g * R² (m³/s²).
pub const MOON_MU: f64 = MOON_G * MOON_RADIUS * MOON_RADIUS;

/// Parameters describing the tether configuration.
#[derive(Resource, Clone, Debug)]
pub struct TetherParams {
    /// Height of the hub above the lunar surface (m).
    pub hub_height: f64,
    /// Radius of the driven attachment point from the hub (m).
    pub r_drive: f64,
    /// Length of the tether from drive point to tip (m).
    pub tether_length: f64,
    /// Mass of the tip payload (kg) — released on command.
    pub tip_mass: f64,
    /// Fixed end mass (kg) — release mechanism, stays attached.
    pub end_mass: f64,
    /// Number of mass segments along the tether.
    pub num_segments: usize,
    /// Rotation rate (Hz).
    pub rotation_rate: f64,
    /// Damping ratio for springs (0 = undamped).
    pub damping: f64,
    /// Hub damper mode: None or Active (force feedback).
    pub hub_damper: HubDamperMode,
    /// Active damper gain (1/s). Controls how fast the hub spool responds.
    pub active_damper_rate: f64,
    /// Active damper EMA averaging: number of wave round-trips.
    pub active_damper_tau_rounds: f64,
    /// Active damper dead zone: fraction of baseline force to ignore.
    pub active_damper_dead_zone: f64,
    /// Tether material.
    pub material: TetherMaterial,
    /// Diameter of the tether cross-section at the tip (m).
    pub tip_diameter: f64,
}

impl Default for TetherParams {
    fn default() -> Self {
        let hub_height = 1000.0;
        let r_drive = 50.0;
        let tether_length = 10_000.0 - r_drive;
        let tip_radius = r_drive + tether_length;
        // Default tip velocity = lunar escape velocity at ground level
        let v_esc = (2.0 * MOON_G * MOON_RADIUS).sqrt();
        let rotation_rate = v_esc / (2.0 * std::f64::consts::PI * tip_radius);
        let tip_mass = 1000.0;
        let end_mass = 200.0;
        let total_tip = tip_mass + end_mass;
        // Size tip diameter for 75% of UTS using total tip mass
        let target_uts = 0.75;
        let sigma_ult = TetherMaterial::DyneemaSK99.tensile_strength();
        let omega = rotation_rate * 2.0 * std::f64::consts::PI;
        let f_tip = total_tip * omega * omega * tip_radius;
        let a_tip = f_tip / (target_uts * sigma_ult);
        let tip_diameter = (4.0 * a_tip / std::f64::consts::PI).sqrt();
        Self {
            hub_height,
            r_drive,
            tether_length,
            tip_mass,
            end_mass,
            num_segments: 100,
            rotation_rate,
            damping: 0.01,
            hub_damper: HubDamperMode::None,
            active_damper_rate: 2.0,
            active_damper_tau_rounds: 5.0,
            active_damper_dead_zone: 0.01,
            material: TetherMaterial::DyneemaSK99,
            tip_diameter,
        }
    }
}

impl TetherParams {
    /// Tip radius from hub (m).
    pub fn tip_radius(&self) -> f64 {
        self.r_drive + self.tether_length
    }

    /// Tip velocity (m/s).
    pub fn tip_velocity(&self) -> f64 {
        2.0 * std::f64::consts::PI * self.rotation_rate * self.tip_radius()
    }

    /// Set rotation rate from desired tip velocity (m/s).
    pub fn set_tip_velocity(&mut self, v: f64) {
        self.rotation_rate = v / (2.0 * std::f64::consts::PI * self.tip_radius());
    }

    /// Angular velocity (rad/s).
    pub fn omega(&self) -> f64 {
        2.0 * std::f64::consts::PI * self.rotation_rate
    }

    pub fn rest_length(&self) -> f64 {
        self.tether_length / self.num_segments as f64
    }

    /// Total mass at the tip (payload + end mass).
    pub fn total_tip_mass(&self) -> f64 {
        self.tip_mass + self.end_mass
    }

    /// Tip cross-section area (m²).
    pub fn tip_area(&self) -> f64 {
        std::f64::consts::PI / 4.0 * self.tip_diameter * self.tip_diameter
    }

    /// Design stress at the tip: σ₀ = F_tip / A_tip = m_total * ω² * r_tip / A_tip.
    /// Uses total tip mass (payload + end mass) for the taper profile.
    /// Returns None if ω ≈ 0 (no centrifugal loading).
    fn design_stress(&self) -> Option<f64> {
        let omega = self.omega();
        if omega < 1e-12 {
            return None;
        }
        let r_tip = self.tip_radius();
        Some(self.total_tip_mass() * omega * omega * r_tip / self.tip_area())
    }

    /// Cross-section area at distance r from the hub (rotation center).
    /// Optimal constant-stress exponential taper:
    ///   A(r) = A_tip * exp(rho * omega² * (R_tip² - r²) / (2 * σ₀))
    /// where σ₀ is the actual tip stress (not σ_ult).
    pub fn area_at_radius(&self, r: f64) -> f64 {
        let a_tip = self.tip_area();
        let Some(sigma) = self.design_stress() else {
            return a_tip;
        };
        let rho = self.material.density();
        let omega = self.omega();
        let r_tip = self.tip_radius();
        let exponent = rho * omega * omega * (r_tip * r_tip - r * r) / (2.0 * sigma);
        a_tip * exponent.exp()
    }

    /// Diameter at distance r from the hub (m).
    pub fn diameter_at_radius(&self, r: f64) -> f64 {
        (4.0 * self.area_at_radius(r) / std::f64::consts::PI).sqrt()
    }

    /// Hub-end diameter (m) — the thickest point.
    pub fn hub_diameter(&self) -> f64 {
        self.diameter_at_radius(self.r_drive)
    }

    /// Taper ratio A_hub / A_tip.
    pub fn taper_ratio(&self) -> f64 {
        let Some(sigma) = self.design_stress() else {
            return 1.0;
        };
        let rho = self.material.density();
        let omega = self.omega();
        let r_tip = self.tip_radius();
        let r_hub = self.r_drive;
        let exponent = rho * omega * omega * (r_tip * r_tip - r_hub * r_hub) / (2.0 * sigma);
        exponent.exp()
    }

    /// Total tether mass by integrating rho * A(r) over segments (midpoint rule).
    pub fn total_tether_mass(&self) -> f64 {
        let n = self.num_segments;
        let l = self.rest_length();
        let rho = self.material.density();
        let mut total = 0.0;
        for i in 0..n {
            let r_mid = self.r_drive + (i as f64 + 0.5) * l;
            total += rho * self.area_at_radius(r_mid) * l;
        }
        total
    }

    /// Tip stress as a fraction of ultimate tensile strength (0..1+).
    /// Uses total tip mass (payload + end mass).
    pub fn tip_uts(&self) -> f64 {
        let omega = self.omega();
        let r_tip = self.tip_radius();
        let f = self.total_tip_mass() * omega * omega * r_tip;
        let sigma = f / self.tip_area();
        sigma / self.material.tensile_strength()
    }

    /// Set tip diameter from a desired tip UTS ratio (0..1).
    /// Uses total tip mass (payload + end mass).
    pub fn set_tip_uts(&mut self, uts: f64) {
        let omega = self.omega();
        let r_tip = self.tip_radius();
        let sigma_ult = self.material.tensile_strength();
        let f = self.total_tip_mass() * omega * omega * r_tip;
        let a_tip = f / (uts * sigma_ult);
        self.tip_diameter = (4.0 * a_tip / std::f64::consts::PI).sqrt();
    }

    /// Position of Moon center. Origin is at the surface below the hub.
    pub fn moon_center(&self) -> [f64; 3] {
        [0.0, -MOON_RADIUS, 0.0]
    }

    /// Speed of sound in the tether material (m/s): sqrt(E / ρ).
    pub fn wave_speed(&self) -> f64 {
        (self.material.youngs_modulus() / self.material.density()).sqrt()
    }

    /// Wave round-trip time hub→tip→hub (s).
    pub fn wave_round_trip(&self) -> f64 {
        2.0 * self.tether_length / self.wave_speed()
    }
}

/// A spring connecting two mass points.
#[derive(Clone, Debug)]
pub struct Spring {
    /// Index of first endpoint.
    pub a: usize,
    /// Index of second endpoint.
    pub b: usize,
    /// Rest length (m).
    pub rest_length: f64,
    /// Spring stiffness E*A/L (N/m).
    pub stiffness: f64,
    /// Damping coefficient (N·s/m).
    pub damping_coeff: f64,
    /// Cross-section area at midpoint (m²), for stress calculation.
    pub area: f64,
}

/// State of the spring-mass simulation.
/// Point 0 is the driven point (kinematic, infinite mass).
/// All other points are free masses subject to spring forces and gravity.
#[derive(Resource, Clone, Debug)]
pub struct TetherState {
    /// Position of each mass point [x, y, z].
    pub positions: Vec<[f64; 3]>,
    /// Velocity of each mass point [vx, vy, vz].
    pub velocities: Vec<[f64; 3]>,
    /// Mass of each point (kg). Point 0 = INFINITY.
    pub masses: Vec<f64>,
    /// Springs connecting pairs of points.
    pub springs: Vec<Spring>,
    /// Current angle of the driven point (rad), in the XZ plane.
    pub drive_angle: f64,
    /// Simulation time (s).
    pub time: f64,
    /// Active damper: current rest length of the first spring (tracks adjustments).
    pub active_damper_rest: f64,
    /// Active damper: exponential moving average of hub tension (baseline).
    pub active_damper_f_smooth: f64,
    /// Active damper: current reel speed (m/s). Positive = reel in, negative = reel out.
    pub active_damper_reel_speed: f64,
}

/// Gap between spool and payload at initialization (m).
const SPOOL_GAP: f64 = 1.0;

impl TetherState {
    /// Create initial state: tether stretched along +X from hub, at y=hub_height.
    /// The tip is split into spool (end_mass) and payload (tip_mass) connected
    /// by a short spring (SPOOL_GAP = 1m). This settles naturally with the tether.
    pub fn new(params: &TetherParams) -> Self {
        let n_tether = params.num_segments + 1; // drive + segment points including spool
        let n_total = n_tether + 1; // +1 for payload
        let mut positions = Vec::with_capacity(n_total);
        let mut velocities = Vec::with_capacity(n_total);
        let mut masses = Vec::with_capacity(n_total);
        let omega = params.omega();
        let h = params.hub_height;
        let rest_len = params.rest_length();
        let rho = params.material.density();
        let e = params.material.youngs_modulus();
        let damping_ratio = params.damping;

        // Points 0..num_segments: tether points including spool at tip
        for i in 0..n_tether {
            let t = i as f64 / params.num_segments as f64;
            let r = params.r_drive + t * params.tether_length;
            positions.push([r, h, 0.0]);
            velocities.push([0.0, 0.0, omega * r]);

            if i == 0 {
                masses.push(f64::INFINITY);
            } else {
                let r_mid = params.r_drive + (i as f64 - 0.5) * rest_len;
                let area = params.area_at_radius(r_mid);
                let m = rho * area * rest_len;
                if i == n_tether - 1 {
                    // Spool point: segment mass + end_mass (no payload)
                    masses.push(m + params.end_mass);
                } else {
                    masses.push(m);
                }
            }
        }

        // Payload point: SPOOL_GAP beyond spool
        let r_payload = params.r_drive + params.tether_length + SPOOL_GAP;
        positions.push([r_payload, h, 0.0]);
        velocities.push([0.0, 0.0, omega * r_payload]);
        masses.push(params.tip_mass);

        // Tether springs between consecutive tether points
        let mut springs = Vec::with_capacity(n_tether);
        for i in 0..n_tether - 1 {
            let r_mid = params.r_drive + (i as f64 + 0.5) * rest_len;
            let area = params.area_at_radius(r_mid);
            let k = e * area / rest_len;
            let seg_m = rho * area * rest_len;
            let c = damping_ratio * 2.0 * (k * seg_m).sqrt();
            springs.push(Spring {
                a: i,
                b: i + 1,
                rest_length: rest_len,
                stiffness: k,
                damping_coeff: c,
                area,
            });
        }

        // Spool→payload spring (1m rest length)
        // Only payload mass hangs from this spring, not end_mass,
        // so scale area down by payload fraction to maintain the same UTS.
        let tip_area = params.tip_area();
        let total_tip = params.total_tip_mass();
        let spool_area = if total_tip > 0.0 {
            tip_area * (params.tip_mass / total_tip)
        } else {
            tip_area
        };
        let k_spool = e * spool_area / SPOOL_GAP;
        let c_spool = damping_ratio * 2.0 * (k_spool * params.tip_mass).sqrt();
        springs.push(Spring {
            a: n_tether - 1,
            b: n_tether,
            rest_length: SPOOL_GAP,
            stiffness: k_spool,
            damping_coeff: c_spool,
            area: spool_area,
        });

        let active_damper_rest = springs[0].rest_length;

        Self {
            positions,
            velocities,
            masses,
            springs,
            drive_angle: 0.0,
            time: 0.0,
            active_damper_rest,
            active_damper_f_smooth: 0.0,
            active_damper_reel_speed: 0.0,
        }
    }

    pub fn num_points(&self) -> usize {
        self.positions.len()
    }

    /// Total kinetic energy of the tether system (J). Skips the driven point (infinite mass).
    pub fn kinetic_energy(&self) -> f64 {
        let mut ke = 0.0;
        for i in 1..self.positions.len() {
            let m = self.masses[i];
            if !m.is_finite() {
                continue;
            }
            let v = &self.velocities[i];
            let v_sq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
            ke += 0.5 * m * v_sq;
        }
        ke
    }
}

/// Compute accelerations for all mass points from springs + gravity.
/// `settle_drag` > 0 adds damping relative to the rotating frame (kills all oscillation modes).
fn compute_accelerations(
    positions: &[[f64; 3]],
    velocities: &[[f64; 3]],
    masses: &[f64],
    springs: &[Spring],
    moon: &[f64; 3],
    settle_drag: f64,
    omega: f64,
) -> Vec<[f64; 3]> {
    let n = positions.len();
    let mut accels = vec![[0.0f64; 3]; n];

    // Spring forces
    for s in springs {
        let (a, b) = (s.a, s.b);
        let dx = positions[b][0] - positions[a][0];
        let dy = positions[b][1] - positions[a][1];
        let dz = positions[b][2] - positions[a][2];
        let dist = (dx * dx + dy * dy + dz * dz).sqrt();
        if dist < 1e-12 {
            continue;
        }
        let inv_dist = 1.0 / dist;
        let nx = dx * inv_dist;
        let ny = dy * inv_dist;
        let nz = dz * inv_dist;

        // Tether: tension only, no compression
        let extension = dist - s.rest_length;
        if extension <= 0.0 {
            continue;
        }
        let f_spring = s.stiffness * extension;

        let dvx = velocities[b][0] - velocities[a][0];
        let dvy = velocities[b][1] - velocities[a][1];
        let dvz = velocities[b][2] - velocities[a][2];
        let rel_v = dvx * nx + dvy * ny + dvz * nz;
        let f_damp = s.damping_coeff * rel_v;

        let f_total = (f_spring + f_damp).max(0.0);
        let fx = f_total * nx;
        let fy = f_total * ny;
        let fz = f_total * nz;

        if masses[a].is_finite() {
            let inv_m = 1.0 / masses[a];
            accels[a][0] += fx * inv_m;
            accels[a][1] += fy * inv_m;
            accels[a][2] += fz * inv_m;
        }
        if masses[b].is_finite() {
            let inv_m = 1.0 / masses[b];
            accels[b][0] -= fx * inv_m;
            accels[b][1] -= fy * inv_m;
            accels[b][2] -= fz * inv_m;
        }
    }

    // Gravity toward Moon center (for all non-driven points)
    for i in 1..n {
        let rx = positions[i][0] - moon[0];
        let ry = positions[i][1] - moon[1];
        let rz = positions[i][2] - moon[2];
        let r_sq = rx * rx + ry * ry + rz * rz;
        let r = r_sq.sqrt();
        if r < 1e-6 {
            continue;
        }
        let g_accel = MOON_MU / r_sq;
        let inv_r = 1.0 / r;
        accels[i][0] -= g_accel * rx * inv_r;
        accels[i][1] -= g_accel * ry * inv_r;
        accels[i][2] -= g_accel * rz * inv_r;
    }

    // Settling: drag relative to the co-rotating frame
    if settle_drag > 0.0 {
        for i in 1..n {
            let px = positions[i][0];
            let pz = positions[i][2];
            // Rigid-body co-rotation velocity: ω_vec × r_from_hub
            // ω_vec = (0, -ω, 0) for rotation from +X toward +Z (right-hand rule)
            // v_rot = (-ω*z, 0, ω*x)
            let v_rel_x = velocities[i][0] - (-omega * pz);
            let v_rel_y = velocities[i][1];
            let v_rel_z = velocities[i][2] - (omega * px);

            accels[i][0] -= settle_drag * v_rel_x;
            accels[i][1] -= settle_drag * v_rel_y;
            accels[i][2] -= settle_drag * v_rel_z;
        }
    }

    accels
}

/// Advance the simulation by dt seconds using RK4 integration.
/// `settle_drag` > 0 enables rotating-frame drag for the settling phase.
pub fn step_rk4(state: &mut TetherState, params: &TetherParams, dt: f64, settle_drag: f64) {
    let n = state.num_points();
    let omega = params.omega();
    let r_drive = params.r_drive;
    let hub_y = params.hub_height;
    let moon = params.moon_center();
    let springs = &state.springs;
    let masses = &state.masses;

    let drive_pos = |angle: f64| -> [f64; 3] {
        [r_drive * angle.cos(), hub_y, r_drive * angle.sin()]
    };
    let drive_vel = |angle: f64| -> [f64; 3] {
        [
            -r_drive * omega * angle.sin(),
            0.0,
            r_drive * omega * angle.cos(),
        ]
    };

    let pack = |pos: &[[f64; 3]], vel: &[[f64; 3]]| -> (Vec<f64>, Vec<f64>) {
        let mut p = Vec::with_capacity((n - 1) * 3);
        let mut v = Vec::with_capacity((n - 1) * 3);
        for i in 1..n {
            p.extend_from_slice(&pos[i]);
            v.extend_from_slice(&vel[i]);
        }
        (p, v)
    };

    let unpack = |p: &[f64], v: &[f64], angle: f64| -> (Vec<[f64; 3]>, Vec<[f64; 3]>) {
        let mut pos = Vec::with_capacity(n);
        let mut vel = Vec::with_capacity(n);
        pos.push(drive_pos(angle));
        vel.push(drive_vel(angle));
        for i in 0..(n - 1) {
            let j = i * 3;
            pos.push([p[j], p[j + 1], p[j + 2]]);
            vel.push([v[j], v[j + 1], v[j + 2]]);
        }
        (pos, vel)
    };

    let derivatives = |pos: &[[f64; 3]], vel: &[[f64; 3]]| -> (Vec<f64>, Vec<f64>) {
        let accels = compute_accelerations(pos, vel, masses, springs, &moon, settle_drag, omega);
        let mut dp = Vec::with_capacity((n - 1) * 3);
        let mut dv = Vec::with_capacity((n - 1) * 3);
        for i in 1..n {
            dp.extend_from_slice(&vel[i]);
            dv.extend_from_slice(&accels[i]);
        }
        (dp, dv)
    };

    let angle0 = state.drive_angle;
    let (p0, v0) = pack(&state.positions, &state.velocities);
    let dim = p0.len();

    let (dp1, dv1) = derivatives(&state.positions, &state.velocities);

    let angle_half = angle0 + omega * dt * 0.5;
    let p_tmp: Vec<f64> = (0..dim).map(|i| p0[i] + 0.5 * dt * dp1[i]).collect();
    let v_tmp: Vec<f64> = (0..dim).map(|i| v0[i] + 0.5 * dt * dv1[i]).collect();
    let (pos2, vel2) = unpack(&p_tmp, &v_tmp, angle_half);
    let (dp2, dv2) = derivatives(&pos2, &vel2);

    let p_tmp: Vec<f64> = (0..dim).map(|i| p0[i] + 0.5 * dt * dp2[i]).collect();
    let v_tmp: Vec<f64> = (0..dim).map(|i| v0[i] + 0.5 * dt * dv2[i]).collect();
    let (pos3, vel3) = unpack(&p_tmp, &v_tmp, angle_half);
    let (dp3, dv3) = derivatives(&pos3, &vel3);

    let angle_full = angle0 + omega * dt;
    let p_tmp: Vec<f64> = (0..dim).map(|i| p0[i] + dt * dp3[i]).collect();
    let v_tmp: Vec<f64> = (0..dim).map(|i| v0[i] + dt * dv3[i]).collect();
    let (pos4, vel4) = unpack(&p_tmp, &v_tmp, angle_full);
    let (dp4, dv4) = derivatives(&pos4, &vel4);

    let p_new: Vec<f64> = (0..dim)
        .map(|i| p0[i] + dt / 6.0 * (dp1[i] + 2.0 * dp2[i] + 2.0 * dp3[i] + dp4[i]))
        .collect();
    let v_new: Vec<f64> = (0..dim)
        .map(|i| v0[i] + dt / 6.0 * (dv1[i] + 2.0 * dv2[i] + 2.0 * dv3[i] + dv4[i]))
        .collect();

    state.drive_angle = angle_full;
    state.time += dt;
    let (new_pos, new_vel) = unpack(&p_new, &v_new, angle_full);
    state.positions = new_pos;
    state.velocities = new_vel;
}

/// Compute per-spring stress ratio (sigma / sigma_ultimate).
pub fn spring_stress_ratios(state: &TetherState, params: &TetherParams) -> Vec<f64> {
    let e = params.material.youngs_modulus();
    let sigma_ult = params.material.tensile_strength();

    state
        .springs
        .iter()
        .map(|s| {
            let dx = state.positions[s.b][0] - state.positions[s.a][0];
            let dy = state.positions[s.b][1] - state.positions[s.a][1];
            let dz = state.positions[s.b][2] - state.positions[s.a][2];
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
            let strain = (dist - s.rest_length) / s.rest_length;
            let stress = e * strain.max(0.0);
            stress / sigma_ult
        })
        .collect()
}

/// Remove springs whose stress exceeds the material's tensile strength.
/// Returns the number of springs that broke.
pub fn break_failed_springs(state: &mut TetherState, params: &TetherParams) -> usize {
    let ratios = spring_stress_ratios(state, params);
    let before = state.springs.len();
    let mut i = 0;
    state.springs.retain(|_| {
        let keep = ratios[i] <= 1.0;
        i += 1;
        keep
    });
    before - state.springs.len()
}

/// Expected centrifugal force at the hub: Σ(m_i · ω² · r_i) for all masses i ≥ 1.
/// Uses horizontal radius r = sqrt(x² + z²).
pub fn expected_hub_force(state: &TetherState, omega: f64) -> f64 {
    let mut f = 0.0;
    for i in 1..state.positions.len() {
        let m = state.masses[i];
        if !m.is_finite() {
            continue;
        }
        let p = &state.positions[i];
        let r = (p[0] * p[0] + p[2] * p[2]).sqrt();
        f += m * omega * omega * r;
    }
    f
}

/// Tension in a spring (scalar, positive = tension).
pub fn spring_tension_pub(state: &TetherState, spring_idx: usize) -> f64 {
    let s = &state.springs[spring_idx];
    let dx = state.positions[s.b][0] - state.positions[s.a][0];
    let dy = state.positions[s.b][1] - state.positions[s.a][1];
    let dz = state.positions[s.b][2] - state.positions[s.a][2];
    let dist = (dx * dx + dy * dy + dz * dz).sqrt();
    let extension = dist - s.rest_length;
    if extension <= 0.0 {
        return 0.0;
    }
    s.stiffness * extension
}

/// Expected static stress ratio per spring, computed outside-in in O(N).
/// For each spring, the expected tension = sum of centrifugal forces of all masses
/// outboard of that spring. Stress ratio = tension / (area · σ_ult).
pub fn expected_stress_ratios(state: &TetherState, params: &TetherParams) -> Vec<f64> {
    let omega = params.omega();
    let sigma_ult = params.material.tensile_strength();
    let n_springs = state.springs.len();
    let mut ratios = vec![0.0; n_springs];

    // Accumulate centrifugal force from outside in.
    // Start with the outermost spring and work inward.
    let mut cumulative_force = 0.0;
    for si in (0..n_springs).rev() {
        let s = &state.springs[si];
        // Add centrifugal force of the outboard point (s.b)
        let m = state.masses[s.b];
        if m.is_finite() {
            let p = &state.positions[s.b];
            let r = (p[0] * p[0] + p[2] * p[2]).sqrt();
            cumulative_force += m * omega * omega * r;
        }
        // Stress ratio for this spring
        let stress = cumulative_force / s.area;
        ratios[si] = stress / sigma_ult;
    }

    ratios
}

/// Soft release state — the spool→payload spring already exists from initialization.
///
/// During release (0 ≤ t ≤ T), the spool spring's rest_length grows cubically:
///   rest = original_rest_length + D₁·(t/T)³
/// Stiffness and damping track rest_length each substep.
/// The release spring uses critical damping (ζ=1) during release.
///
/// At t = T the payload detaches: the spool spring + payload point are popped.
/// For instant release (T=0): payload is popped immediately.
///
/// D₁ = a_tip · T² / 6.
pub struct SoftRelease {
    pub start_time: f64,
    /// Release duration T (s). 0 = instant.
    pub release_duration: f64,
    /// D₁: total payout = a_tip · T² / 6.
    pub d1: f64,
    /// E·A product for the spool spring (N).
    pub ea: f64,
    /// Rest length of spool spring at trigger time.
    pub original_rest_length: f64,
    /// Whether the payload has detached.
    pub detached: bool,
    /// Payload position at detach.
    pub detach_pos: [f64; 3],
    /// Payload velocity at detach.
    pub detach_vel: [f64; 3],
}


/// Run multiple sub-steps per frame for stability.
/// `settle_duration` and `settle_drag` control the settling phase.
///
/// `release`: if Some, either instant release (duration=0) or soft release (spool model).
/// Returns true if the payload detached during this frame.
pub fn step_simulation(
    state: &mut TetherState,
    params: &TetherParams,
    frame_dt: f64,
    sub_steps: usize,
    breaking_enabled: bool,
    settle_duration: f64,
    settle_drag: f64,
    mut release: Option<&mut SoftRelease>,
) -> bool {
    let dt = frame_dt / sub_steps as f64;
    let mut detached_this_frame = false;
    let active_damper = params.hub_damper == HubDamperMode::Active;

    for _ in 0..sub_steps {
        if let Some(ref mut sr) = release {
            if !sr.detached {
                if sr.release_duration <= 0.0 {
                    // Instant release: pop payload point + spool spring
                    let last = state.num_points() - 1;
                    sr.detach_pos = state.positions[last];
                    sr.detach_vel = state.velocities[last];
                    state.positions.pop();
                    state.velocities.pop();
                    state.masses.pop();
                    state.springs.pop();
                    sr.detached = true;
                    detached_this_frame = true;
                    // Reset active damper EMA to expected force after detach
                    if active_damper {
                        let omega = params.omega();
                        state.active_damper_f_smooth = expected_hub_force(state, omega);
                    }
                } else {
                    // Soft release: ramp spool spring rest_length cubically
                    let elapsed = state.time - sr.start_time;
                    let u = (elapsed / sr.release_duration).clamp(0.0, 1.0);
                    let rest = sr.original_rest_length + sr.d1 * u * u * u;
                    let last_si = state.springs.len() - 1;
                    let k = sr.ea / rest;
                    state.springs[last_si].rest_length = rest;
                    state.springs[last_si].stiffness = k;
                    state.springs[last_si].damping_coeff =
                        2.0 * (k * params.tip_mass).sqrt(); // critical damping

                    if elapsed >= sr.release_duration {
                        // Detach: save payload state, pop spool spring + payload point
                        let last = state.num_points() - 1;
                        sr.detach_pos = state.positions[last];
                        sr.detach_vel = state.velocities[last];
                        state.positions.pop();
                        state.velocities.pop();
                        state.masses.pop();
                        state.springs.pop();
                        sr.detached = true;
                        detached_this_frame = true;
                        // Reset active damper EMA to expected force after detach
                        if active_damper {
                            let omega = params.omega();
                            state.active_damper_f_smooth = expected_hub_force(state, omega);
                        }
                    }
                }
            }
        }

        let drag = if state.time < settle_duration {
            settle_drag
        } else {
            0.0
        };
        step_rk4(state, params, dt, drag);

        // Active damper: adjust first spring rest_length to absorb transients.
        // Uses EMA of tension as baseline — only reacts to fast deviations (waves),
        // not to the steady-state force level.
        // Only active after settling — during settling, the drag handles equilibrium.
        if active_damper && !state.springs.is_empty() && state.time >= settle_duration {
            let f_actual = spring_tension_pub(state, 0);
            // Seed EMA on first activation
            if state.active_damper_f_smooth == 0.0 {
                state.active_damper_f_smooth = f_actual;
            }
            // Update EMA: alpha = dt / tau (first-order low-pass)
            // tau = N * wave round-trip time, so we average over N complete wave cycles
            let tau = params.active_damper_tau_rounds * params.wave_round_trip();
            let alpha = (dt / tau).min(1.0);
            state.active_damper_f_smooth += alpha * (f_actual - state.active_damper_f_smooth);
            let raw_error = f_actual - state.active_damper_f_smooth;
            // Dead zone: ignore small errors (noise / EMA lag)
            let threshold = params.active_damper_dead_zone * state.active_damper_f_smooth.abs();
            let error = if raw_error.abs() < threshold { 0.0 } else { raw_error };
            let k = state.springs[0].stiffness;
            // Adjust rest length: positive error (transient spike) → increase rest (reel out)
            let d_rest = (error / k) * params.active_damper_rate * dt;
            state.active_damper_rest += d_rest;
            // Clamp to reasonable range (50%–200% of original segment rest length)
            let nominal = params.rest_length();
            state.active_damper_rest = state.active_damper_rest.clamp(nominal * 0.5, nominal * 2.0);
            // Reel speed: positive = reel in (rest shrinks), negative = reel out
            state.active_damper_reel_speed = -d_rest / dt;
            // Keep stiffness consistent: k = EA/L (EA is invariant)
            let ea = state.springs[0].stiffness * state.springs[0].rest_length;
            state.springs[0].rest_length = state.active_damper_rest;
            state.springs[0].stiffness = ea / state.active_damper_rest;
        }

        if breaking_enabled && state.time >= settle_duration {
            break_failed_springs(state, params);
        }
    }
    detached_this_frame
}
