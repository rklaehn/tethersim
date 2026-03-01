//! egui-based GUI for tether simulation parameters.

use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};

use crate::physics::{HubDamperMode, TetherMaterial, TetherParams, TetherState, MOON_RADIUS};
use crate::visualization::{RecordingState, SimControl, StressMonitor};

/// Map stress ratio to egui color: green → yellow → red.
fn stress_color_egui(ratio: f32) -> egui::Color32 {
    let r = ratio.clamp(0.0, 1.0);
    if r < 0.5 {
        let t = r * 2.0;
        egui::Color32::from_rgb((t * 255.0) as u8, 255, 0)
    } else {
        let t = (r - 0.5) * 2.0;
        egui::Color32::from_rgb(255, ((1.0 - t) * 255.0) as u8, 0)
    }
}

/// System: render the parameter panel (left) and info panel (right).
pub fn parameter_panel(
    mut contexts: EguiContexts,
    mut params: ResMut<TetherParams>,
    state: Res<TetherState>,
    mut control: ResMut<SimControl>,
    mut monitor: ResMut<StressMonitor>,
    mut recording: ResMut<RecordingState>,
) {
    let Ok(ctx) = contexts.ctx_mut() else { return };

    // --- Left panel: controls ---
    egui::SidePanel::left("tether_params").show(ctx, |ui| {
        let editable = !control.started;

        // --- Simulation (always visible at top) ---
        egui::CollapsingHeader::new("Simulation")
            .default_open(true)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    if ui.button("\u{23EE}").clicked() {
                        control.needs_reset = true;
                    }
                    if ui
                        .button(if control.paused { "\u{25B6}" } else { "\u{23F8}" })
                        .clicked()
                    {
                        control.paused = !control.paused;
                        if !control.paused && !control.started {
                            control.started = true;
                        }
                    }
                });

                ui.label("Speed:");
                ui.add(
                    egui::Slider::new(&mut control.speed, 0.1..=100.0)
                        .logarithmic(true)
                        .text("x"),
                );

                ui.label("Sub-steps per frame:");
                let mut sub = control.sub_steps as f64;
                if ui
                    .add(
                        egui::Slider::new(&mut sub, 10.0..=1000.0)
                            .logarithmic(true)
                            .text("n"),
                    )
                    .changed()
                {
                    control.sub_steps = sub as usize;
                }

                ui.checkbox(&mut control.breaking_enabled, "Enable spring breaking");

                ui.separator();
                ui.strong("Settling");
                ui.label("Settle duration (s):");
                ui.add(egui::Slider::new(&mut control.settle_duration, 0.0..=300.0).text("s"));
                ui.label("Settle drag (1/s):");
                ui.add(
                    egui::Slider::new(&mut control.settle_drag, 0.01..=10.0)
                        .logarithmic(true)
                        .text("γ"),
                );

                ui.separator();
                ui.strong("Release");
                ui.label("Release duration (s):");
                ui.add(egui::Slider::new(&mut control.release_duration, 0.0..=5.0).text("s"));
                {
                    let tip_accel = params.omega() * params.omega() * params.tip_radius();
                    let t1 = control.release_duration;
                    let rope_len = tip_accel * t1 * t1 / 6.0;
                    ui.label(format!("Release rope: {:.1} m", rope_len));
                }
                ui.checkbox(&mut control.auto_release, "Auto-release");
                if control.auto_release {
                    ui.label("Delay after settle (s):");
                    ui.add(egui::Slider::new(&mut control.auto_release_time, 1.0..=120.0).text("s"));
                }
                ui.add_enabled_ui(!control.release_triggered, |ui| {
                    if ui.button("Release now").clicked() {
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
                        // Auto-record on manual release
                        if control.auto_record && !control.auto_record_started {
                            recording.start();
                            control.auto_record_started = true;
                        }
                    }
                });

                ui.separator();
                ui.strong("Recording");
                ui.checkbox(&mut control.auto_record, "Auto-record");
                if control.auto_record {
                    ui.label("Before release (s):");
                    ui.add(egui::Slider::new(&mut control.auto_record_before, 0.0..=30.0).text("s"));
                    ui.label("After release (s):");
                    ui.add(egui::Slider::new(&mut control.auto_record_after, 1.0..=60.0).text("s"));
                }
                if recording.recording {
                    let btn = egui::Button::new("Stop recording")
                        .fill(egui::Color32::from_rgb(180, 40, 40));
                    if ui.add(btn).clicked() {
                        recording.stop_and_save();
                    }
                    ui.label(format!("Frames: {}", recording.frame_count()));
                } else if ui.button("Record (G)").clicked() {
                    recording.start();
                }
                let status = recording.status.lock().unwrap();
                if !status.is_empty() {
                    ui.label(status.as_str());
                }
            });

        ui.separator();

        // --- Hub Damper (collapsible, own section) ---
        egui::CollapsingHeader::new("Hub Damper")
            .default_open(true)
            .show(ui, |ui| {
                ui.add_enabled_ui(editable, |ui| {
                    egui::ComboBox::from_id_salt("hub_damper_combo")
                        .selected_text(params.hub_damper.name())
                        .show_ui(ui, |ui| {
                            for &mode in HubDamperMode::ALL {
                                ui.selectable_value(&mut params.hub_damper, mode, mode.name());
                            }
                        });
                    if params.hub_damper == HubDamperMode::Active {
                        ui.label("Gain (1/s):");
                        ui.add(egui::Slider::new(&mut params.active_damper_rate, 0.1..=100.0).logarithmic(true).text("rate"));
                        ui.label("Averaging (wave round-trips):");
                        ui.add(egui::Slider::new(&mut params.active_damper_tau_rounds, 1.0..=20.0).text("τ"));
                        ui.label("Dead zone (%):");
                        let mut dz_pct = params.active_damper_dead_zone * 100.0;
                        if ui.add(egui::Slider::new(&mut dz_pct, 0.1..=10.0).text("%")).changed() {
                            params.active_damper_dead_zone = dz_pct / 100.0;
                        }
                    }
                });
            });

        ui.separator();

        // --- Tether Parameters (collapsible, below hub damper) ---
        egui::CollapsingHeader::new("Tether Parameters")
            .default_open(true)
            .show(ui, |ui| {
                ui.add_enabled_ui(editable, |ui| {
                    ui.label("Payload mass (kg):");
                    let old_uts_pm = params.tip_uts();
                    if ui.add(
                        egui::Slider::new(&mut params.tip_mass, 0.1..=10000.0)
                            .logarithmic(true)
                            .text("kg"),
                    ).changed() {
                        if old_uts_pm > 1e-6 { params.set_tip_uts(old_uts_pm); }
                    }

                    ui.label("End mass (kg):");
                    let old_uts_em = params.tip_uts();
                    if ui.add(
                        egui::Slider::new(&mut params.end_mass, 0.0..=5000.0)
                            .logarithmic(true)
                            .text("kg"),
                    ).changed() {
                        if old_uts_em > 1e-6 { params.set_tip_uts(old_uts_em); }
                    }

                    ui.separator();
                    ui.strong("Rotation");

                    let mut tip_v = params.tip_velocity();
                    ui.label("Tip velocity (m/s):");
                    if ui
                        .add(egui::Slider::new(&mut tip_v, 0.0..=3000.0).text("v_tip"))
                        .changed()
                    {
                        params.set_tip_velocity(tip_v);
                    }

                    ui.label(format!("Rotation rate: {:.4} Hz", params.rotation_rate));

                    ui.separator();
                    ui.strong("Geometry");

                    ui.label("Hub height (m above surface):");
                    ui.add(egui::Slider::new(&mut params.hub_height, 10.0..=1000.0).text("m"));

                    ui.label("Drive radius (m from hub):");
                    let old_tip_v_d = params.tip_velocity();
                    let old_uts_d = params.tip_uts();
                    if ui.add(egui::Slider::new(&mut params.r_drive, 1.0..=500.0).text("m")).changed() {
                        params.set_tip_velocity(old_tip_v_d);
                        if old_uts_d > 1e-6 {
                            params.set_tip_uts(old_uts_d);
                        }
                    }

                    ui.label("Total length (m):");
                    let mut total_length = params.r_drive + params.tether_length;
                    if ui
                        .add(
                            egui::Slider::new(&mut total_length, 100.0..=500000.0)
                                .logarithmic(true)
                                .text("m"),
                        )
                        .changed()
                    {
                        let old_tip_v = params.tip_velocity();
                        let old_uts = params.tip_uts();
                        params.tether_length = (total_length - params.r_drive).max(1.0);
                        params.set_tip_velocity(old_tip_v);
                        if old_uts > 1e-6 {
                            params.set_tip_uts(old_uts);
                        }
                    }

                    ui.label(format!("Target tip radius: {:.0} m", params.tip_radius()));

                    ui.label("Segments:");
                    let mut segs = params.num_segments as f64;
                    if ui
                        .add(egui::Slider::new(&mut segs, 10.0..=500.0).text("n"))
                        .changed()
                    {
                        params.num_segments = segs as usize;
                    }

                    ui.separator();
                    ui.strong("Material & Sizing");

                    ui.label("Material:");
                    let old_uts_mat = params.tip_uts();
                    let prev_mat = params.material;
                    egui::ComboBox::from_id_salt("material_combo")
                        .selected_text(params.material.name())
                        .show_ui(ui, |ui| {
                            for &mat in TetherMaterial::ALL {
                                ui.selectable_value(&mut params.material, mat, mat.name());
                            }
                        });
                    if params.material != prev_mat && old_uts_mat > 1e-6 {
                        params.set_tip_uts(old_uts_mat);
                    }

                    ui.label("Damping ζ:");
                    ui.add(egui::Slider::new(&mut params.damping, 0.001..=10.0).logarithmic(true).text("ζ"));

                    ui.label(format!("Tip diameter: {:.2} mm", params.tip_diameter * 1000.0));

                    let mut uts = params.tip_uts();
                    ui.label("Tip UTS (fraction of σ_ult):");
                    if ui
                        .add(
                            egui::Slider::new(&mut uts, 0.01..=1.0)
                                .logarithmic(true)
                                .text("UTS"),
                        )
                        .changed()
                    {
                        params.set_tip_uts(uts);
                    }
                });
            });
    });

    // --- Right panel: info + stress monitor ---
    egui::SidePanel::right("tether_info").default_width(250.0).show(ctx, |ui| {
        ui.heading("Info");
        ui.label(format!("Time: {:.2} s", state.time));
        if state.time < control.settle_duration {
            ui.label(format!(
                "Settling: {:.1} / {:.0} s",
                state.time, control.settle_duration
            ));
        }
        if control.release_triggered {
            let elapsed = state.time - control.release_start_time;
            let rd = control.release_duration;
            if rd <= 0.0 {
                ui.label("Released (instant)");
            } else if elapsed < rd {
                ui.label(format!(
                    "Releasing: {:.1} / {:.1} s",
                    elapsed, rd
                ));
            } else {
                ui.label("Released");
            }
        }

        ui.separator();
        ui.strong("Rotation");
        ui.label(format!("omega: {:.4} rad/s", params.omega()));
        ui.label(format!(
            "Nominal tip v: {:.1} m/s (at r={:.0} m)",
            params.tip_velocity(),
            params.tip_radius()
        ));
        if let Some(tip_vel) = state.velocities.last() {
            let v = (tip_vel[0] * tip_vel[0] + tip_vel[1] * tip_vel[1] + tip_vel[2] * tip_vel[2]).sqrt();
            ui.label(format!("Actual tip speed: {:.1} m/s", v));
        }
        let ke_mwh = state.kinetic_energy() / 3.6e9;
        ui.label(format!("Kinetic energy: {:.2} MWh", ke_mwh));

        ui.separator();
        ui.strong("Geometry");
        if let Some(tip_pos) = state.positions.last() {
            let r_horiz = (tip_pos[0] * tip_pos[0] + tip_pos[2] * tip_pos[2]).sqrt();
            ui.label(format!("Actual tip r: {:.0} m", r_horiz));
            let dy = tip_pos[1] + MOON_RADIUS;
            let dist = (tip_pos[0] * tip_pos[0] + dy * dy + tip_pos[2] * tip_pos[2]).sqrt();
            let tip_altitude = dist - MOON_RADIUS;
            ui.label(format!("Tip droop: {:.1} m", params.hub_height - tip_altitude));
        }
        ui.label(format!(
            "Springs: {} / {}",
            state.springs.len(),
            params.num_segments
        ));

        ui.separator();
        ui.strong("Material & Sizing");
        ui.label(format!(
            "E={:.0} GPa, \u{03C1}={:.0} kg/m\u{00B3}, \u{03C3}={:.1} GPa",
            params.material.youngs_modulus() / 1e9,
            params.material.density(),
            params.material.tensile_strength() / 1e9
        ));
        ui.label(format!(
            "Tether mass: {:.2} kg",
            params.total_tether_mass()
        ));
        ui.label(format!("Taper ratio: {:.2}x", params.taper_ratio()));
        ui.label(format!(
            "Hub diameter: {:.2} mm",
            params.hub_diameter() * 1000.0
        ));
        ui.label(format!(
            "Tip diameter: {:.2} mm",
            params.tip_diameter * 1000.0
        ));
        ui.label(format!(
            "Tip UTS: {:.1}%",
            params.tip_uts() * 100.0
        ));
        let ratios = crate::physics::spring_stress_ratios(&state, &params);
        if let Some(&max_ratio) = ratios.iter().max_by(|a, b| a.partial_cmp(b).unwrap()) {
            ui.label(format!("Max stress: {:.1}% of \u{03C3}_ult", max_ratio * 100.0));
        }

        // Active damper info
        if params.hub_damper == HubDamperMode::Active {
            ui.separator();
            ui.strong("Hub Damper");
            let v = state.active_damper_reel_speed;
            let dir = if v > 0.001 { "in" } else if v < -0.001 { "out" } else { "" };
            ui.label(format!("Reel speed: {:.3} m/s {}", v.abs(), dir));
            if !state.springs.is_empty() {
                let f = crate::physics::spring_tension_pub(&state, 0);
                let power = f * v;
                if power.abs() > 1e3 {
                    ui.label(format!("Reel power: {:.2} kW", power / 1e3));
                } else {
                    ui.label(format!("Reel power: {:.1} W", power));
                }
            }
        }

        // --- Stress monitor (VU meter) ---
        ui.separator();
        let n = monitor.current.len();
        if n > 0 {
            ui.horizontal(|ui| {
                ui.strong("Stress Monitor");
                if let Some(&max_peak) = monitor
                    .peak
                    .iter()
                    .max_by(|a, b| a.partial_cmp(b).unwrap())
                {
                    if monitor.settled_reset_done {
                        ui.label(format!(" Peak: {:.1}%", max_peak * 100.0));
                    } else {
                        ui.label(" (settling...)");
                    }
                }
                if ui.small_button("Reset").clicked() {
                    monitor.reset_peaks = true;
                }
            });
            // Compact legend
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 3.0;
                let sz = egui::vec2(10.0, 3.0);
                let (r, p) = ui.allocate_painter(sz, egui::Sense::hover());
                p.rect_filled(r.rect, 0.0, egui::Color32::from_rgb(0, 200, 255));
                ui.label("expected");
                let (r, p) = ui.allocate_painter(sz, egui::Sense::hover());
                p.rect_filled(r.rect, 0.0, egui::Color32::WHITE);
                ui.label("peak");
                let (r, p) = ui.allocate_painter(sz, egui::Sense::hover());
                p.rect_filled(r.rect, 0.0, egui::Color32::from_rgb(255, 60, 60));
                ui.label("UTS");
            });

            // Use all remaining vertical space
            let available = ui.available_size();
            let chart_h = available.y.max(80.0);
            let (response, painter) = ui.allocate_painter(
                egui::vec2(available.x, chart_h),
                egui::Sense::hover(),
            );
            let rect = response.rect;

            painter.rect_filled(rect, 0.0, egui::Color32::from_gray(20));

            let max_y = 1.25_f32;
            let bar_w = rect.width() / n as f32;

            // Grid lines at 0.25, 0.5, 0.75, 1.0 (UTS)
            for &level in &[0.25_f32, 0.5, 0.75, 1.0] {
                let y = rect.bottom() - rect.height() * level / max_y;
                let color = if (level - 1.0).abs() < 0.01 {
                    egui::Color32::from_rgb(255, 60, 60)
                } else {
                    egui::Color32::from_gray(50)
                };
                painter.line_segment(
                    [egui::pos2(rect.left(), y), egui::pos2(rect.right(), y)],
                    egui::Stroke::new(1.0, color),
                );
                painter.text(
                    egui::pos2(rect.left() + 2.0, y - 12.0),
                    egui::Align2::LEFT_TOP,
                    format!("{:.0}%", level * 100.0),
                    egui::FontId::proportional(10.0),
                    egui::Color32::from_gray(140),
                );
            }

            // Bars and peak markers
            for i in 0..n {
                let x = rect.left() + i as f32 * bar_w;

                let ratio = (monitor.current[i] as f32).clamp(0.0, max_y);
                let bar_h = rect.height() * ratio / max_y;
                if bar_h > 0.5 {
                    let bar_rect = egui::Rect::from_min_max(
                        egui::pos2(x, rect.bottom() - bar_h),
                        egui::pos2(x + bar_w, rect.bottom()),
                    );
                    painter.rect_filled(bar_rect, 0.0, stress_color_egui(monitor.current[i] as f32));
                }

                // Expected static stress marker (cyan) — hide once tether has fractured
                if state.springs.len() >= params.num_segments {
                    if i < monitor.expected.len() {
                        let exp = (monitor.expected[i] as f32).clamp(0.0, max_y);
                        if exp > 0.01 {
                            let exp_y = rect.bottom() - rect.height() * exp / max_y;
                            painter.line_segment(
                                [egui::pos2(x, exp_y), egui::pos2(x + bar_w, exp_y)],
                                egui::Stroke::new(1.0, egui::Color32::from_rgb(0, 200, 255)),
                            );
                        }
                    }
                }

                // Peak marker (white)
                if i < monitor.peak.len() && monitor.settled_reset_done {
                    let peak = (monitor.peak[i] as f32).clamp(0.0, max_y);
                    if peak > 0.01 {
                        let peak_y = rect.bottom() - rect.height() * peak / max_y;
                        painter.line_segment(
                            [egui::pos2(x, peak_y), egui::pos2(x + bar_w, peak_y)],
                            egui::Stroke::new(2.0, egui::Color32::WHITE),
                        );
                    }
                }
            }

        }
    });
}
