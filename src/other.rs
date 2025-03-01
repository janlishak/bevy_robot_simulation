use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use iyes_perf_ui::prelude::*;
use rand::prelude::*;
use bevy_spectator::*;

#[derive(Component)]
pub struct RobotMovement {
    pub speed: f32,
    pub direction: Vec3,
}

pub fn setup_camera(mut commands: Commands) {
    /* Create the 3D Camera. */
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-40.0, 40.0, 80.0).looking_at(Vec3::ZERO, Vec3::Y),
        Spectator
    ));

    // commands.spawn((
    //     Camera3dBundle::default(), Spectator
    // ));
}

pub fn setup_scene(mut commands: Commands) {
    /* Create the ground. */
    commands
    .spawn((
        RigidBody::Fixed, // Make sure the ground is a static rigid body
        Collider::cuboid(40.0, 1.0, 40.0),
        Friction::coefficient(0.0),
    ))
    .insert(Transform::from_xyz(0.0, 0.0, 0.0));


    // /* Create platforms. */
    // commands
    //     .spawn(Collider::cuboid(1.0, 0.125, 1.0))
    //     .insert(Transform {
    //         translation: Vec3::new(0.0, 1.0, 0.0),
    //         rotation: Quat::from_rotation_z(std::f32::consts::FRAC_PI_4),
    //         scale: Vec3::ONE,
    //     });

    // commands
    //     .spawn(Collider::cuboid(1.0, 0.125, 1.0))
    //     .insert(Transform {
    //         translation: Vec3::new(-2.0, 3.0, 0.0),
    //         rotation: Quat::from_rotation_z(-std::f32::consts::FRAC_PI_4),
    //         scale: Vec3::ONE,
    //     });

    // commands
    //     .spawn(Collider::cuboid(1.0, 0.125, 1.0))
    //     .insert(Transform {
    //         translation: Vec3::new(0.0, 5.0, 0.0),
    //         rotation: Quat::from_rotation_z(std::f32::consts::FRAC_PI_4),
    //         scale: Vec3::ONE,
    //     });

    // commands
    // .spawn(Collider::cuboid(0.125, 4.0, 20.0))
    // .insert(Transform {
    //     translation: Vec3::new(-10.0, 2.0, 0.0),
    //     rotation: Quat::IDENTITY,
    //     scale: Vec3::ONE,
    // });

    // commands
    // .spawn(Collider::cuboid(0.125, 4.0, 20.0))
    // .insert(Transform {
    //     translation: Vec3::new(10.0, 2.0, 0.0),
    //     rotation: Quat::from_rotation_y(5.0_f32.to_radians()),
    //     scale: Vec3::ONE,
    // });

    /* Create the bouncing ball. */
    // for x in 0..10 {
    //     for z in 0..10 {
    //         let pos_x: f32 = x as f32 * 2.0;
    //         let pos_z: f32 = z as f32 * 2.0;

    //         commands
    //             .spawn((RigidBody::Dynamic, Collider::ball(0.5), Restitution::coefficient(0.7)))
    //             .insert(Transform::from_xyz(pos_x, 10.0, pos_z))
    //             .insert(Velocity::zero());
    //     }
    // }

    // const ROBOT_COUNT: i32 = 100;
    // for y in 0..ROBOT_COUNT {
    //     let pos_y: f32 = y as f32 * 2.0;

    //     commands
    //         .spawn((
    //             RigidBody::Dynamic,
    //             Collider::ball(0.5),
    //             Restitution::coefficient(0.7),
    //             LockedAxes::ROTATION_LOCKED,
    //         ))
    //         .insert(Transform::from_xyz(0.0, 10.0 + pos_y, 0.0))
    //         .insert(Velocity::zero());

    // }

    const ROBOT_COUNT: i32 = 100;
    const FOOD_COUNT: i32 = 5;

    for y in 0..ROBOT_COUNT {
        let pos_y: f32 = y as f32 * 2.0;

        // Generate random direction
        let mut rng = rand::thread_rng();
        let direction =
            Vec3::new(rng.gen_range(-1.0..1.0), 0.0, rng.gen_range(-1.0..1.0)).normalize_or_zero(); // Ensures we don't get a zero vector

        let speed = rng.gen_range(0.001..0.002); // Random speed between 1.0 and 3.0

        commands
            .spawn((
                RigidBody::Dynamic,
                Collider::ball(0.5),
                Restitution::coefficient(0.7),
                Friction::new(0.0),
                LockedAxes::ROTATION_LOCKED,
                RobotMovement { speed, direction }, // ✅ Attach movement component
            ))
            .insert(Transform::from_xyz(0.0, 10.0 + pos_y, 0.0))
            .insert(Velocity::zero());
    }


    for y in 0..FOOD_COUNT {
        let pos_y: f32 = y as f32 * 2.0;

        // Generate random direction
        let mut rng = rand::thread_rng();

        let direction = Vec3::ZERO;

        let speed = 0.0;

        commands
            .spawn((
                RigidBody::Dynamic,
                Collider::cylinder(0.2, 1.5),
                Restitution::coefficient(0.7),
                Friction::new(0.1),
                LockedAxes::ROTATION_LOCKED,
                RobotMovement { speed, direction },
            ))
            .insert(Transform::from_xyz(0.0, 10.0 + pos_y, 0.0))
            .insert(Velocity::zero());
    }
}

pub fn print_ball_altitude_old(positions: Query<&Transform, With<RigidBody>>) {
    for transform in positions.iter() {
        println!("Ball altitude: {}", transform.translation.y);
    }
}

pub fn draw_debug_axes(mut gizmos: Gizmos) {
    // X-axis (Red)
    gizmos.line(Vec3::ZERO, Vec3::X * 10.0, Color::srgb(1.0, 0.0, 0.0));
    // Y-axis (Green)
    gizmos.line(Vec3::ZERO, Vec3::Y * 10.0, Color::srgb(0.0, 1.0, 0.0));
    // Z-axis (Blue)
    gizmos.line(Vec3::ZERO, Vec3::Z * 10.0, Color::srgb(0.0, 0.0, 1.0));
}

pub fn setup_debug_info(mut commands: Commands) {
    commands.spawn(PerfUiAllEntries::default());
}
