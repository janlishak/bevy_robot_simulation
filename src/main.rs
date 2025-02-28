mod other;

use bevy::{diagnostic::LogDiagnosticsPlugin, diagnostic::FrameTimeDiagnosticsPlugin, prelude::*};
use bevy_rapier3d::prelude::*;

use other::*;
use std::time::Instant;

fn main() {
    App::new()
        // plugins
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
        .add_plugins(LogDiagnosticsPlugin::default())
        // .add_plugins(FrameTimeDiagnosticsPlugin)

        // system
        .add_systems(Startup, setup_graphics)
        .add_systems(Startup, setup_physics)
        .add_systems(Startup, rapier_config)

        .add_systems(Update, draw_debug_axes)
        .add_systems(Update, manual_step_physics)
        // .add_systems(Update, print_ball_altitude)
        // .add_systems(Update, reset_ball_if_fallen)

        // Run
        .run();
}

fn rapier_config(mut query: Query<&mut RapierConfiguration>) {
    println!("Setting up Rapier configuration");
    if let Ok(mut config) = query.get_single_mut() {
        config.gravity = Vec3::Y * -9.81;
        config.physics_pipeline_active = false; // Disabling automatic physics stepping
    }
}

const PHYSICS_STEPS_PER_FRAME: usize = 50; // Run physics 10 times per frame

fn manual_step_physics(
    mut context_query: Query<(
        &mut RapierContextSimulation,
        &mut RapierContextColliders,
        &mut RapierContextJoints,
        &mut RapierRigidBodySet,
    )>,
    time: Res<Time>,
    mut body_query: Query<&RapierRigidBodyHandle, With<Collider>>,
) {
    if let Ok((mut simulation, mut colliders, mut joints, mut rigidbody_set)) =
        context_query.get_single_mut()
    {
        let gravity = Vec3::Y * -9.81;
        let dt = 1.0 / 60.0;

        // Start measuring time
        let start_time = Instant::now();

        for _ in 0..PHYSICS_STEPS_PER_FRAME {
            simulation.step_simulation(
                &mut colliders,
                &mut joints,
                &mut rigidbody_set,
                gravity,
                TimestepMode::Fixed { dt, substeps: 1 },
                None,
                &(),
                &time,
                &mut SimulationToRenderTime { diff: 0.0 },
                None,
            );

            // ✅ Check for ball reset in each physics step
            for rigid_body_handle in body_query.iter_mut() {
                if let Some(body) = rigidbody_set.bodies.get_mut(rigid_body_handle.0) {
                    if body.translation().y < 0.0 {
                        // println!("Cooool fell! Resetting position and velocity.");
                        let new_position = Vec3::new(0.0, 9.0, 0.0);
                        // Reset physics body position
                        body.set_translation(new_position.into(), true);
                        body.set_linvel(Vec3::ZERO.into(), true);
                        body.set_angvel(Vec3::ZERO.into(), true);
                    }
                }
            }
        }

        // Stop measuring time
        let elapsed = start_time.elapsed();
        // println!("Physics step time: {:.6} seconds", elapsed.as_secs_f64());
    }
}

// fn manual_writeback(
//     transform_query: &mut Query<(&RapierRigidBodyHandle, &mut Transform)>,
//     rigid_body_set: &RapierRigidBodySet,
// ) {
//     for (handle, mut transform) in transform_query.iter_mut() {
//         if let Some(body) = rigid_body_set.bodies.get(handle.0) {
//             transform.translation = (*body.translation()).into();
//             transform.rotation = (*body.rotation()).into();
//         }
//     }
// }

// pub fn reset_ball_if_fallen(
//     mut query: Query<&RapierRigidBodyHandle, With<Collider>>,
//     mut context_query: Query<&mut RapierRigidBodySet>,
// ) {
//     if let Ok(mut rigid_body_set) = context_query.get_single_mut() {
//         for rigid_body_handle in query.iter_mut() {
//             if let Some(body) = rigid_body_set.bodies.get_mut(rigid_body_handle.0) {
//                 if body.translation().y < 0.0 {
//                     println!("NEW Ball fell! Resetting position and velocity.");

//                     let new_position = Vec3::new(0.0, 9.0, 0.0);

//                     // Reset physics body position
//                     body.set_translation(new_position.into(), true);
//                     body.set_linvel(Vec3::ZERO.into(), true);
//                     body.set_angvel(Vec3::ZERO.into(), true);
//                 }
//             }
//         }
//     }
// }