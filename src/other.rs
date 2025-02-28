use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

pub fn setup_graphics(mut commands: Commands) {
    // debug pring
    println!("Setting up graphics");
    // Add a camera so we can see the debug-render.
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-6.0, 6.0, 50.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

pub fn setup_physics(mut commands: Commands) {
    /* Create the ground. */
    commands
    .spawn(Collider::cuboid(4.0, 0.25, 4.0))
    .insert(Transform::from_xyz(0.0, 0.0, 0.0));

    /* Create platforms. */
    commands
    .spawn(Collider::cuboid(1.0, 0.125, 1.0))
    .insert(Transform {
        translation: Vec3::new(0.0, 1.0, 0.0),
        rotation: Quat::from_rotation_z(std::f32::consts::FRAC_PI_4),
        scale: Vec3::ONE,
    });

    commands
    .spawn(Collider::cuboid(1.0, 0.125, 1.0))
    .insert(Transform {
        translation: Vec3::new(-2.0, 3.0, 0.0),
        rotation: Quat::from_rotation_z(-std::f32::consts::FRAC_PI_4),
        scale: Vec3::ONE,
    });

    commands
    .spawn(Collider::cuboid(1.0, 0.125, 1.0))
    .insert(Transform {
        translation: Vec3::new(0.0, 5.0, 0.0),
        rotation: Quat::from_rotation_z(std::f32::consts::FRAC_PI_4),
        scale: Vec3::ONE,
    });

    /* Create the bouncing ball. */
    for x in 0..100 {
        commands
            .spawn((RigidBody::Dynamic, Collider::ball(0.5), Restitution::coefficient(0.7)))
            .insert(Transform::from_xyz(0.0, 8.0 + x as f32, 0.0)) // Cast `x` to `f32`
            .insert(Velocity::zero());
    }
    

    
}

pub fn print_ball_altitude(positions: Query<&Transform, With<RigidBody>>) {
    for transform in positions.iter() {
        println!("Ball altitude: {}", transform.translation.y);
    }
}

pub fn draw_debug_axes(mut gizmos: Gizmos) {
    // X-axis (Red)
    gizmos.line(Vec3::ZERO, Vec3::X * 10.0, Color::rgb(1.0, 0.0, 0.0));
    // Y-axis (Green)
    gizmos.line(Vec3::ZERO, Vec3::Y * 10.0, Color::rgb(0.0, 1.0, 0.0));
    // Z-axis (Blue)
    gizmos.line(Vec3::ZERO, Vec3::Z * 10.0, Color::rgb(0.0, 0.0, 1.0));
}