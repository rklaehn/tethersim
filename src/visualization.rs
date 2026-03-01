//! Bevy visualization for the tether simulation.
//!
//! The tether rotates in a plane parallel to the lunar surface.
//! Origin is at the surface point directly below the hub.
//! Hub is at y=hub_height, Moon center at y=-MOON_RADIUS.

use bevy::{
    input::mouse::{MouseMotion, MouseWheel},
    prelude::*,
    render::view::screenshot::{Screenshot, ScreenshotCaptured},
};

use std::sync::{Arc, Mutex};

use crate::physics::{self, TetherParams, TetherState, MOON_RADIUS};

/// 1 world unit = 1 km.
const M2W: f32 = 0.001;

#[derive(Component)]
pub struct MoonMarker;

#[derive(Component)]
pub struct TetherPointMarker {
    pub index: usize,
}

#[derive(Component)]
pub struct DrivePointMarker;

#[derive(Component)]
pub struct HubMarker;

/// Released payload flying under lunar gravity.
#[derive(Component)]
pub struct PayloadMarker {
    pub pos: [f64; 3],
    pub vel: [f64; 3],
}

/// Camera orbit state.
#[derive(Resource)]
pub struct CameraOrbit {
    pub distance: f32,
    pub azimuth: f32,
    pub elevation: f32,
}

impl Default for CameraOrbit {
    fn default() -> Self {
        Self {
            distance: 25.0,
            azimuth: 0.0,
            // Start looking down from above (elevation ~80°)
            elevation: 1.3,
        }
    }
}

/// Simulation control state.
#[derive(Resource)]
pub struct SimControl {
    pub paused: bool,
    pub speed: f64,
    pub sub_steps: usize,
    pub needs_reset: bool,
    /// True once the simulation has been unpaused at least once.
    /// While false, parameters can be edited freely.
    pub started: bool,
    /// Whether springs can break when stress exceeds tensile strength.
    pub breaking_enabled: bool,
    /// Duration of the settling phase (s). During settling, rotating-frame drag is applied.
    pub settle_duration: f64,
    /// Drag coefficient during settling (1/s). Higher = faster settling.
    pub settle_drag: f64,
    /// Auto-release: if enabled, release payload automatically after this many seconds post-settle.
    pub auto_release: bool,
    /// Delay after settling before auto-release triggers (s).
    pub auto_release_time: f64,
    /// Duration of the soft release phase (0 = instant).
    pub release_duration: f64,
    /// Whether release has been triggered.
    pub release_triggered: bool,
    /// Sim time at which release was triggered.
    pub release_start_time: f64,
    /// Whether the payload has actually detached.
    pub release_detached: bool,
    /// Transient: payload detached this frame, spawn entity.
    pub detach_this_frame: bool,
    /// D₁: release payout = a_tip · T² / 6.
    pub d1: f64,
    /// E·A product for the spool spring (N).
    pub ea: f64,
    /// Rest length of spool spring at trigger time.
    pub original_rest_length: f64,
    /// Payload position at detach (for spawning).
    pub detach_pos: [f64; 3],
    /// Payload velocity at detach.
    pub detach_vel: [f64; 3],
    /// Auto-record: record around the release event.
    pub auto_record: bool,
    /// Seconds to record before release.
    pub auto_record_before: f64,
    /// Seconds to record after release.
    pub auto_record_after: f64,
    /// Whether auto-record has already started recording for the current release.
    pub auto_record_started: bool,
}

impl Default for SimControl {
    fn default() -> Self {
        Self {
            paused: true,
            speed: 1.0,
            sub_steps: 100,
            needs_reset: false,
            started: false,
            breaking_enabled: false,
            settle_duration: 10.0,
            settle_drag: 2.0,
            auto_release: true,
            auto_release_time: 10.0,
            release_duration: 0.0,
            release_triggered: false,
            release_start_time: 0.0,
            release_detached: false,
            detach_this_frame: false,
            d1: 0.0,
            ea: 0.0,
            original_rest_length: 0.0,
            detach_pos: [0.0; 3],
            detach_vel: [0.0; 3],
            auto_record: false,
            auto_record_before: 2.0,
            auto_record_after: 10.0,
            auto_record_started: false,
        }
    }
}

/// Tracks current and peak stress per spring (VU-meter display).
#[derive(Resource, Default)]
pub struct StressMonitor {
    pub current: Vec<f64>,
    pub peak: Vec<f64>,
    /// Expected static stress ratios (centrifugal loading, outside-in).
    pub expected: Vec<f64>,
    /// Whether peaks have been reset after settling completed.
    pub settled_reset_done: bool,
    /// Set to true to request a manual peak reset.
    pub reset_peaks: bool,
}

/// Convert simulation [x, y, z] (meters) to Bevy world coords.
/// Direct mapping: sim xyz -> Bevy xyz. Hub at origin, Moon center below on -Y.
fn sim_to_world(pos: &[f64; 3]) -> Vec3 {
    Vec3::new(pos[0] as f32 * M2W, pos[1] as f32 * M2W, pos[2] as f32 * M2W)
}

fn compute_camera_pos(orbit: &CameraOrbit) -> Vec3 {
    let (az_sin, az_cos) = orbit.azimuth.sin_cos();
    let (el_sin, el_cos) = orbit.elevation.sin_cos();
    Vec3::new(
        orbit.distance * el_cos * az_sin,
        orbit.distance * el_sin,
        orbit.distance * el_cos * az_cos,
    )
}

/// Spawn the scene: camera, light, moon, drive/tip point entities.
pub fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
    params: Res<TetherParams>,
    state: Res<TetherState>,
) {
    let orbit = CameraOrbit::default();
    let hub_pos = Vec3::new(0.0, params.hub_height as f32 * M2W, 0.0);
    let cam_pos = compute_camera_pos(&orbit) + hub_pos;
    commands.spawn((
        Camera3d::default(),
        Transform::from_translation(cam_pos).looking_at(hub_pos, Vec3::Y),
    ));

    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 500.0,
        ..default()
    });

    commands.spawn((
        DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: false,
            ..default()
        },
        // Sun shines from below — hub side of moon in shadow, tether visible
        // Light along +Y (from below moon toward hub) offset slightly so
        // the surface near the hub is in shade but limb is illuminated.
        Transform::from_rotation(Quat::from_euler(
            bevy::math::EulerRot::YXZ,
            0.3,  // slight azimuth offset
            1.2,  // shines upward (~70° from horizontal), illuminating far side
            0.0,
        )),
    ));

    // Moon sphere — center at y=-MOON_RADIUS, surface touches y=0
    let moon_r = MOON_RADIUS as f32 * M2W;
    let moon_y = -moon_r;
    let moon_texture: Handle<Image> = asset_server.load("textures/moon.jpg");
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(moon_r).mesh().uv(256, 128))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color_texture: Some(moon_texture),
            perceptual_roughness: 0.9,
            metallic: 0.0,
            ..default()
        })),
        // Rotate so hub (+Y pole) is at 210°E selenographic (far side equator,
        // 30° prograde from anti-Earth). rotation_z(+π/2) brings equator to pole,
        // rotation_y(+π/6) selects the 30° prograde offset.
        Transform::from_translation(Vec3::new(0.0, moon_y, 0.0))
            .with_rotation(
                Quat::from_rotation_z(std::f32::consts::FRAC_PI_2)
                    * Quat::from_rotation_y(std::f32::consts::FRAC_PI_6),
            ),
        MoonMarker,
    ));

    // Hub marker (small white sphere at hub height)
    let hub_y = params.hub_height as f32 * M2W;
    let hub_mesh = meshes.add(Sphere::new(params.tether_length as f32 * M2W * 0.003).mesh().uv(32, 16));
    let hub_mat = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        emissive: LinearRgba::new(0.5, 0.5, 0.5, 1.0),
        ..default()
    });
    commands.spawn((
        Mesh3d(hub_mesh),
        MeshMaterial3d(hub_mat),
        Transform::from_translation(Vec3::new(0.0, hub_y, 0.0)),
        HubMarker,
    ));

    // Sphere sizes proportional to tether length
    let tether_len_w = params.tether_length as f32 * M2W;
    let drive_r = tether_len_w * 0.005;
    let tip_r = tether_len_w * 0.01;

    // Drive point (orange)
    let drive_mesh = meshes.add(Sphere::new(drive_r).mesh().uv(32, 16));
    let drive_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.3, 0.1),
        emissive: LinearRgba::new(0.5, 0.15, 0.05, 1.0),
        ..default()
    });
    commands.spawn((
        Mesh3d(drive_mesh),
        MeshMaterial3d(drive_mat),
        Transform::from_translation(sim_to_world(&state.positions[0])),
        TetherPointMarker { index: 0 },
        DrivePointMarker,
    ));

    // Tip point (yellow, larger)
    let tip_mesh = meshes.add(Sphere::new(tip_r).mesh().uv(32, 16));
    let tip_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 1.0, 0.0),
        emissive: LinearRgba::new(0.5, 0.5, 0.0, 1.0),
        ..default()
    });
    let last = state.num_points() - 1;
    commands.spawn((
        Mesh3d(tip_mesh),
        MeshMaterial3d(tip_mat),
        Transform::from_translation(sim_to_world(&state.positions[last])),
        TetherPointMarker { index: last },
    ));
}

/// System: advance physics simulation.
pub fn update_physics(
    mut state: ResMut<TetherState>,
    params: Res<TetherParams>,
    mut control: ResMut<SimControl>,
    mut recording: ResMut<RecordingState>,
    time: Res<Time>,
) {
    if control.paused {
        return;
    }

    // Auto-record: start recording before release (only with auto-release, since we know the time)
    if control.auto_release && control.auto_record && !control.auto_record_started && !control.release_triggered {
        let release_time = control.settle_duration + control.auto_release_time;
        let record_start = release_time - control.auto_record_before;
        if state.time >= record_start {
            recording.start();
            control.auto_record_started = true;
        }
    }

    // Auto-release: trigger when sim time passes settle_duration + auto_release_time
    if control.auto_release && !control.release_triggered && state.time >= control.settle_duration + control.auto_release_time {
        control.release_triggered = true;
        control.release_start_time = state.time;
        let last_si = state.springs.len() - 1;
        let s = &state.springs[last_si];
        control.ea = s.stiffness * s.rest_length;
        control.original_rest_length = s.rest_length;
        let tip = &state.positions[state.num_points() - 1];
        let r_tip = (tip[0] * tip[0] + tip[2] * tip[2]).sqrt();
        let omega = params.omega();
        let tip_accel = omega * omega * r_tip;
        let t1 = control.release_duration;
        control.d1 = tip_accel * t1 * t1 / 6.0;
    }

    // Auto-record: stop after configured duration post-release
    if control.auto_record && control.auto_record_started && recording.recording && control.release_triggered {
        let elapsed = state.time - control.release_start_time;
        if elapsed >= control.auto_record_after {
            recording.stop_and_save();
        }
    }

    let frame_dt = time.delta_secs_f64() * control.speed;
    let frame_dt = frame_dt.min(0.1);

    let detached = if control.release_triggered {
        let mut sr = physics::SoftRelease {
            start_time: control.release_start_time,
            release_duration: control.release_duration,
            d1: control.d1,
            ea: control.ea,
            original_rest_length: control.original_rest_length,
            detached: control.release_detached,
            detach_pos: [0.0; 3],
            detach_vel: [0.0; 3],
        };
        let result = physics::step_simulation(
            &mut state,
            &params,
            frame_dt,
            control.sub_steps,
            control.breaking_enabled,
            control.settle_duration,
            control.settle_drag,
            Some(&mut sr),
        );
        control.release_detached = sr.detached;
        if result {
            control.detach_pos = sr.detach_pos;
            control.detach_vel = sr.detach_vel;
        }
        result
    } else {
        physics::step_simulation(
            &mut state,
            &params,
            frame_dt,
            control.sub_steps,
            control.breaking_enabled,
            control.settle_duration,
            control.settle_drag,
            None,
        );
        false
    };

    if detached {
        control.detach_this_frame = true;
    }
}

/// System: update stress monitor with current ratios and track peaks.
pub fn update_stress_monitor(
    state: Res<TetherState>,
    params: Res<TetherParams>,
    control: Res<SimControl>,
    mut monitor: ResMut<StressMonitor>,
) {
    let ratios = physics::spring_stress_ratios(&state, &params);

    // Reset peaks when transitioning past settling phase (once)
    if state.time >= control.settle_duration && !monitor.settled_reset_done {
        monitor.peak = vec![0.0; ratios.len()];
        monitor.settled_reset_done = true;
    }

    // Manual peak reset
    if monitor.reset_peaks {
        monitor.peak = vec![0.0; ratios.len()];
        monitor.reset_peaks = false;
    }

    // Resize if needed (after reset or segment count change) — preserve existing peaks
    if monitor.peak.len() < ratios.len() {
        monitor.peak.resize(ratios.len(), 0.0);
    } else if monitor.peak.len() > ratios.len() {
        monitor.peak.truncate(ratios.len());
    }

    monitor.current = ratios;
    monitor.expected = physics::expected_stress_ratios(&state, &params);

    // Accumulate peaks only after settling
    if monitor.settled_reset_done {
        for i in 0..monitor.current.len() {
            if monitor.current[i] > monitor.peak[i] {
                monitor.peak[i] = monitor.current[i];
            }
        }
    }
}

/// System: sync drive/tip point entity positions.
pub fn sync_tether_points(
    state: Res<TetherState>,
    mut query: Query<(&TetherPointMarker, &mut Transform)>,
) {
    for (marker, mut transform) in query.iter_mut() {
        // Clamp index: after soft release the last point is removed,
        // so the tip marker should follow the new tip (spool).
        let idx = marker.index.min(state.positions.len() - 1);
        transform.translation = sim_to_world(&state.positions[idx]);
    }
}

/// Map stress ratio (0..1+) to color: green → yellow → red.
fn stress_color(ratio: f32) -> Color {
    let r = ratio.clamp(0.0, 1.0);
    if r < 0.5 {
        // green (0,1,0) → yellow (1,1,0)
        let t = r * 2.0;
        Color::srgb(t, 1.0, 0.0)
    } else {
        // yellow (1,1,0) → red (1,0,0)
        let t = (r - 0.5) * 2.0;
        Color::srgb(1.0, 1.0 - t, 0.0)
    }
}

/// System: draw tether springs with gizmos, colored by stress.
pub fn draw_tether_gizmos(state: Res<TetherState>, params: Res<TetherParams>, mut gizmos: Gizmos) {
    if !state.springs.is_empty() {
        let ratios = physics::spring_stress_ratios(&state, &params);
        for (i, spring) in state.springs.iter().enumerate() {
            let a = sim_to_world(&state.positions[spring.a]);
            let b = sim_to_world(&state.positions[spring.b]);
            let color = stress_color(ratios[i] as f32);
            gizmos.line(a, b, color);
        }
    }
    // Draw a small cross at the hub
    let s = 0.05;
    let hy = params.hub_height as f32 * M2W;
    gizmos.line(Vec3::new(-s, hy, 0.0), Vec3::new(s, hy, 0.0), Color::WHITE);
    gizmos.line(Vec3::new(0.0, hy, -s), Vec3::new(0.0, hy, s), Color::WHITE);
}

/// System: handle camera orbit with mouse.
pub fn camera_input(
    mut orbit: ResMut<CameraOrbit>,
    mouse_button: Res<ButtonInput<MouseButton>>,
    mut mouse_motion: MessageReader<MouseMotion>,
    mut mouse_wheel: MessageReader<MouseWheel>,
) {
    if mouse_button.pressed(MouseButton::Left) {
        for ev in mouse_motion.read() {
            orbit.azimuth -= ev.delta.x * 0.005;
            orbit.elevation += ev.delta.y * 0.005;
            orbit.elevation = orbit.elevation.clamp(-std::f32::consts::FRAC_PI_2 + 0.01, std::f32::consts::FRAC_PI_2 - 0.01);
        }
    } else {
        mouse_motion.clear();
    }

    for ev in mouse_wheel.read() {
        orbit.distance *= (1.0 - ev.y * 0.1).clamp(0.1, 10.0);
        orbit.distance = orbit.distance.clamp(0.1, 10000.0);
    }
}

/// System: update camera transform — always looks at hub.
pub fn update_camera(
    orbit: Res<CameraOrbit>,
    params: Res<TetherParams>,
    mut query: Query<&mut Transform, With<Camera3d>>,
) {
    let hub_pos = Vec3::new(0.0, params.hub_height as f32 * M2W, 0.0);
    let cam_pos = compute_camera_pos(&orbit) + hub_pos;
    for mut transform in query.iter_mut() {
        *transform = Transform::from_translation(cam_pos).looking_at(hub_pos, Vec3::Y);
    }
}

/// System: keep Moon sphere position in sync (constant at -MOON_RADIUS).
pub fn sync_moon_position(mut query: Query<&mut Transform, With<MoonMarker>>) {
    let moon_y = -(MOON_RADIUS as f32 * M2W);
    for mut transform in query.iter_mut() {
        transform.translation.y = moon_y;
    }
}

/// System: keep hub sphere at current hub_height, and re-init state while editing.
pub fn sync_hub_and_state(
    params: Res<TetherParams>,
    control: Res<SimControl>,
    mut state: ResMut<TetherState>,
    mut query: Query<&mut Transform, With<HubMarker>>,
) {
    let hub_y = params.hub_height as f32 * M2W;
    for mut transform in query.iter_mut() {
        transform.translation.y = hub_y;
    }
    // While not started, keep the state in sync with params
    if !control.started {
        *state = TetherState::new(&params);
    }
}

/// System: spawn a payload sphere when the payload detaches.
/// Both instant and soft release set detach_this_frame and save detach_pos/vel.
pub fn spawn_payload(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut control: ResMut<SimControl>,
    params: Res<TetherParams>,
    existing: Query<Entity, With<PayloadMarker>>,
) {
    if !existing.is_empty() || !control.detach_this_frame {
        return;
    }
    control.detach_this_frame = false;
    let pos = control.detach_pos;
    let vel = control.detach_vel;
    let tether_len_w = params.tether_length as f32 * M2W;
    let payload_r = tether_len_w * 0.012;
    let mesh = meshes.add(Sphere::new(payload_r).mesh().uv(32, 16));
    let mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.2, 0.6, 1.0),
        emissive: LinearRgba::new(0.1, 0.3, 0.5, 1.0),
        ..default()
    });
    commands.spawn((
        Mesh3d(mesh),
        MeshMaterial3d(mat),
        Transform::from_translation(sim_to_world(&pos)),
        PayloadMarker { pos, vel },
    ));
}

/// System: update payload ballistic trajectory under lunar gravity.
pub fn update_payload(
    control: Res<SimControl>,
    params: Res<TetherParams>,
    time: Res<Time>,
    mut query: Query<(&mut PayloadMarker, &mut Transform)>,
) {
    if control.paused {
        return;
    }
    let dt = time.delta_secs_f64() * control.speed;
    let dt = dt.min(0.1);
    let moon = params.moon_center();
    for (mut payload, mut transform) in query.iter_mut() {
        // Gravity toward Moon center
        let rx = payload.pos[0] - moon[0];
        let ry = payload.pos[1] - moon[1];
        let rz = payload.pos[2] - moon[2];
        let r_sq = rx * rx + ry * ry + rz * rz;
        let r = r_sq.sqrt();
        if r > 1e-6 {
            let g = physics::MOON_MU / r_sq;
            let inv_r = 1.0 / r;
            payload.vel[0] -= g * rx * inv_r * dt;
            payload.vel[1] -= g * ry * inv_r * dt;
            payload.vel[2] -= g * rz * inv_r * dt;
        }
        payload.pos[0] += payload.vel[0] * dt;
        payload.pos[1] += payload.vel[1] * dt;
        payload.pos[2] += payload.vel[2] * dt;
        transform.translation = sim_to_world(&payload.pos);
    }
}

/// System: handle reset requests.
pub fn handle_reset(
    mut commands: Commands,
    mut state: ResMut<TetherState>,
    params: Res<TetherParams>,
    mut control: ResMut<SimControl>,
    mut monitor: ResMut<StressMonitor>,
    payloads: Query<Entity, With<PayloadMarker>>,
) {
    if !control.needs_reset {
        return;
    }
    control.needs_reset = false;
    control.paused = true;
    control.started = false;
    control.release_triggered = false;
    control.release_detached = false;
    control.original_rest_length = 0.0;
    control.detach_this_frame = false;
    control.auto_record_started = false;
    *state = TetherState::new(&params);
    *monitor = StressMonitor::default();
    // Despawn any payload entities
    for entity in payloads.iter() {
        commands.entity(entity).despawn();
    }
}

// --- Video Recording ---

/// State for MP4 recording.
#[derive(Resource)]
pub struct RecordingState {
    pub recording: bool,
    frame_counter: u32,
    /// Capture every N render frames (~10fps at 60fps).
    pub frame_skip: u32,
    /// Collected frames (RGBA).
    frames: Vec<Vec<u8>>,
    width: u32,
    height: u32,
    pub max_frames: usize,
    /// Status message shown in UI (shared with encoding thread).
    pub status: Arc<Mutex<String>>,
}

impl Default for RecordingState {
    fn default() -> Self {
        Self {
            recording: false,
            frame_counter: 0,
            frame_skip: 2,
            frames: Vec::new(),
            width: 0,
            height: 0,
            max_frames: 9000,
            status: Arc::new(Mutex::new(String::new())),
        }
    }
}

impl RecordingState {
    pub fn frame_count(&self) -> usize {
        self.frames.len()
    }

    /// Start recording: clear frames and set flag.
    pub fn start(&mut self) {
        self.recording = true;
        self.frames.clear();
        self.frame_counter = 0;
        *self.status.lock().unwrap() = String::new();
    }

    /// Stop recording and encode MP4 on a background thread.
    pub fn stop_and_save(&mut self) {
        self.recording = false;
        let frames = std::mem::take(&mut self.frames);
        if frames.is_empty() {
            return;
        }
        let w = self.width;
        let h = self.height;
        let skip = self.frame_skip;
        let status = self.status.clone();
        *status.lock().unwrap() = format!("Encoding {} frames...", frames.len());
        std::thread::spawn(move || {
            save_recording_mp4(frames, w, h, skip, status);
        });
    }
}

/// RGB8 wrapper for RGBA frame data (crops to even dimensions, strips alpha).
struct RgbFrame<'a> {
    rgba: &'a [u8],
    src_width: usize,
    width: usize,
    height: usize,
}

impl openh264::formats::RGBSource for RgbFrame<'_> {
    fn dimensions(&self) -> (usize, usize) {
        (self.width, self.height)
    }
    fn pixel_f32(&self, x: usize, y: usize) -> (f32, f32, f32) {
        let i = (y * self.src_width + x) * 4;
        // Bevy screenshots are BGRA on most backends — swap R↔B
        (self.rgba[i + 2] as f32, self.rgba[i + 1] as f32, self.rgba[i] as f32)
    }
}

/// Encode collected RGBA frames as H.264 MP4 using openh264 + minimp4.
fn save_recording_mp4(
    frames: Vec<Vec<u8>>,
    width: u32,
    height: u32,
    frame_skip: u32,
    status: Arc<Mutex<String>>,
) {
    use openh264::encoder::Encoder;
    use openh264::formats::YUVBuffer;

    let total = frames.len();

    // Timestamp-based filename
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs();
    let secs_per_day = 86400u64;
    let days = now / secs_per_day;
    let rem = now % secs_per_day;
    let (hr, mi, se) = (rem / 3600, (rem % 3600) / 60, rem % 60);
    let (year, month, day) = {
        let mut y = 1970i32;
        let mut d = days as i32;
        loop {
            let yd = if y % 4 == 0 && (y % 100 != 0 || y % 400 == 0) { 366 } else { 365 };
            if d < yd { break; }
            d -= yd;
            y += 1;
        }
        let leap = y % 4 == 0 && (y % 100 != 0 || y % 400 == 0);
        let mdays = [31, if leap { 29 } else { 28 }, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];
        let mut mo = 0u32;
        for &md in &mdays {
            if d < md { break; }
            d -= md;
            mo += 1;
        }
        (y, mo + 1, d + 1)
    };
    let path = format!("recording-{year:04}{month:02}{day:02}T{hr:02}{mi:02}{se:02}.mp4");

    // H.264 requires even dimensions
    let enc_w = (width & !1) as usize;
    let enc_h = (height & !1) as usize;
    if enc_w == 0 || enc_h == 0 {
        *status.lock().unwrap() = "Frame too small".into();
        return;
    }

    // Initialize H.264 encoder — high bitrate for clean screen recording
    use openh264::encoder::{EncoderConfig, RateControlMode};
    use openh264::OpenH264API;
    let config = EncoderConfig::new()
        .bitrate(openh264::encoder::BitRate::from_bps(10_000_000))
        .rate_control_mode(RateControlMode::Bitrate)
        .skip_frames(false);
    let mut encoder = match Encoder::with_api_config(OpenH264API::from_source(), config) {
        Ok(e) => e,
        Err(e) => {
            *status.lock().unwrap() = format!("Encoder: {e}");
            return;
        }
    };

    // Encode all frames → concatenated Annex-B bitstream
    let mut h264_data = Vec::new();
    for (i, rgba) in frames.iter().enumerate() {
        let rgb_frame = RgbFrame {
            rgba,
            src_width: width as usize,
            width: enc_w,
            height: enc_h,
        };
        let yuv = YUVBuffer::from_rgb_source(rgb_frame);
        let bitstream = match encoder.encode(&yuv) {
            Ok(bs) => bs,
            Err(e) => {
                *status.lock().unwrap() = format!("Frame {i}: {e}");
                return;
            }
        };
        bitstream.write_vec(&mut h264_data);
        if (i + 1) % 50 == 0 {
            *status.lock().unwrap() = format!("Encoding {}/{}...", i + 1, total);
        }
    }

    // Mux into MP4
    *status.lock().unwrap() = format!("Muxing {} frames...", total);
    let file = match std::fs::File::create(&path) {
        Ok(f) => f,
        Err(e) => {
            *status.lock().unwrap() = format!("File: {e}");
            return;
        }
    };
    let fps = (60 / frame_skip).max(1);
    let mut muxer = minimp4::Mp4Muxer::new(file);
    muxer.init_video(enc_w as i32, enc_h as i32, false, "tethersim");
    muxer.write_video_with_fps(&h264_data, fps);
    muxer.close();
    *status.lock().unwrap() = format!("Saved {path}");
}

/// System: spawn screenshot entities while recording.
pub fn capture_recording_frames(mut recording: ResMut<RecordingState>, mut commands: Commands) {
    // Auto-save when limit was reached
    if !recording.recording && !recording.frames.is_empty() {
        let frames = std::mem::take(&mut recording.frames);
        let w = recording.width;
        let h = recording.height;
        let skip = recording.frame_skip;
        let status = recording.status.clone();
        *status.lock().unwrap() = format!("Encoding {} frames (limit reached)...", frames.len());
        std::thread::spawn(move || {
            save_recording_mp4(frames, w, h, skip, status);
        });
        return;
    }
    if !recording.recording {
        return;
    }
    recording.frame_counter += 1;
    if recording.frame_counter % recording.frame_skip == 0 {
        commands.spawn(Screenshot::primary_window());
    }
}

/// Global observer: collect screenshot data into recording buffer.
pub fn collect_recording_frame(
    trigger: On<ScreenshotCaptured>,
    mut recording: ResMut<RecordingState>,
) {
    if !recording.recording {
        return;
    }
    let img = trigger.event();
    let w = img.width();
    let h = img.height();
    if let Some(ref raw) = img.data {
        recording.width = w;
        recording.height = h;
        recording.frames.push(raw.clone());
        if recording.frames.len() >= recording.max_frames {
            recording.recording = false;
        }
    }
}

/// System: keyboard shortcuts (G = toggle recording).
pub fn keyboard_input(
    keys: Res<ButtonInput<KeyCode>>,
    mut recording: ResMut<RecordingState>,
) {
    if keys.just_pressed(KeyCode::KeyG) {
        if recording.recording {
            recording.stop_and_save();
        } else {
            recording.start();
        }
    }
}
