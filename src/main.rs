//! Lunar rotating tether simulation.
//!
//! Mass-spring chain model with RK4 integration, visualized with Bevy.

use bevy::prelude::*;
use bevy_egui::{EguiPlugin, EguiPrimaryContextPass};

mod gui;
mod physics;
mod visualization;

fn main() {
    let params = physics::TetherParams::default();
    let state = physics::TetherState::new(&params);

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Lunar Tether Simulator".into(),
                resolution: (1280u32, 720u32).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .insert_resource(params)
        .insert_resource(state)
        .insert_resource(visualization::CameraOrbit::default())
        .insert_resource(visualization::SimControl::default())
        .insert_resource(visualization::StressMonitor::default())
        .insert_resource(visualization::RecordingState::default())
        .add_systems(Startup, visualization::setup_scene)
        .add_observer(visualization::collect_recording_frame)
        .add_systems(EguiPrimaryContextPass, gui::parameter_panel)
        .add_systems(
            Update,
            (
                visualization::update_physics,
                visualization::update_stress_monitor,
                visualization::sync_tether_points,
                visualization::sync_moon_position,
                visualization::sync_hub_and_state,
                visualization::draw_tether_gizmos,
                visualization::camera_input,
                visualization::update_camera,
                visualization::spawn_payload,
                visualization::update_payload,
                visualization::handle_reset,
                visualization::capture_recording_frames,
                visualization::keyboard_input,
            )
                .chain(),
        )
        .run();
}
