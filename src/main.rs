use bevy::{prelude::*, sprite::MaterialMesh2dBundle};
use bevy_prototype_debug_lines::*;

const LEG_LENGTH: f32 = 150.0;
const WAVE_AMPLITUDE: f32 = 50.0;
const WAVE_SPEED: f32 = 2.5;

struct SolvedIK {
    angle1: f32,
    knee: Vec3,
    angle2: f32,
    foot: Vec3,
}

fn main(){
  App::new()
    .add_plugins(DefaultPlugins)
    .add_plugin(DebugLinesPlugin::default())
    .add_startup_system(setup)
    .add_system(leg_update)
    .run();
}

#[derive(Component)]
struct Leg {
    points: [Vec3; 3],
    // points: Vec<Vec3>,
}

// #[derive(Component)]
// enum LegSegment {
//     Upper,
//     Lower,
// }

// Bevy setup system
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    // 2D orthographic camera
    commands.spawn(Camera2dBundle::default());

    // Spawn spatial bundle to hold leg segments
    commands.spawn(
        (
            SpatialBundle {
                transform: Transform::from_translation(Vec3::ZERO),
                ..default()
            },
            Leg {
                points: [
                    Vec3::new(0., 0., 0.),
                    Vec3::new(0., -LEG_LENGTH/2., 0.),
                    Vec3::new(0., -LEG_LENGTH/1.5, 0.),
                ],
            },
        )
    ).with_children(|parent| {
        parent.spawn((
            SpriteBundle {
                sprite: Sprite {
                    color: Color::rgb(0.25, 0.25, 0.75),
                    // custom_size: Some(Vec2::new(25.0, LEG_LENGTH/2.0)),
                    ..default()
                },
                // transform: Transform::from_translation(Vec3::new(0., 0., 0.)),
                transform: Transform {
                    translation: Vec3::new(0., 0., 0.),
                    scale: Vec3::new(25.0, LEG_LENGTH/2.0, 1.0),
                    ..default()
                },
                ..default()
            }, 
        ));
        parent.spawn((
            SpriteBundle {
                sprite: Sprite {
                    color: Color::rgb(0.25, 0.25, 0.75),
                    // custom_size: Some(Vec2::new(25.0, LEG_LENGTH/2.0)),
                    ..default()
                },
                // transform: Transform::from_translation(Vec3::new(0., 0., 0.)),
                transform: Transform {
                    translation: Vec3::new(0., 0., 0.),
                    scale: Vec3::new(25.0, LEG_LENGTH/2.0, 1.0),
                    ..default()
                },
                ..default()
            }, 
        ));
        parent.spawn(MaterialMesh2dBundle {
            mesh: meshes.add(shape::Circle::new(12.5).into()).into(),
            material: materials.add(ColorMaterial::from(Color::rgb(0.25, 0.25, 0.75))),
            transform: Transform::from_translation(Vec3::new(0., 0., 0.)),
            ..default()
        });
    });
}

// Update leg segments
fn leg_update(
    mut query: Query<(&mut Leg, &Children)>,
    mut transform_query: Query<&mut Transform>,
    mut lines: ResMut<DebugLines>,
    time: Res<Time>,
) {
    for (mut leg, children) in query.iter_mut() {
        // Set leg [2] point to wave
        leg.points[2] = Vec3::new(
            0.0,
            -LEG_LENGTH + WAVE_AMPLITUDE * (time.elapsed_seconds() * WAVE_SPEED).sin(),
            0.0,
        );

        // Use inverse kinematics to calculate joint angles
        let ik = inverse_kinematics(
            leg.points[2],
            leg.points[0],
            LEG_LENGTH/2.0,
            LEG_LENGTH/2.0,
        );

        // println!("angle1: {}, angle2: {}", angle1, angle2);

        // Update leg second point
        leg.points[1] = ik.knee;

        // Clamp last point to leg length
        // leg.points[2] = leg.points[1] + (leg.points[2] - leg.points[1]).normalize() * LEG_LENGTH/2.0;

        // Update last leg point to start at joint and move by angle2
        leg.points[2] = ik.foot;
        for (i, child) in children.iter().enumerate() {
            let mut transform = transform_query.get_mut(*child).unwrap();
            // transform.translation = leg.points[i];

            // println!("leg.points[0]: {:?}", leg.points[0].y);
            // println!("leg.points[1]: {:?}", leg.points[1].y);
            // println!("leg.points[2]: {:?}", leg.points[2].y);

            // Update child transform
            if i == 0 {
                transform.translation = (leg.points[0] + leg.points[1])/2.0;
                // transform.translation = leg.points[0];
                // println!("transform: {:?}", transform.translation);
                transform.rotation = Quat::from_rotation_z(ik.angle1+std::f32::consts::PI/2.0);

                // Set rotation to go from point 0 to point 1
                // let angle = (leg.points[1] - leg.points[0]).angle_between(Vec3::new(0.0, 1.0, 0.0));
                // transform.rotation = Quat::from_rotation_z(angle);
            } else if i == 1 {
                transform.translation = (leg.points[1] + leg.points[2])/2.0;
                // transform.translation = leg.points[2];
                // println!("transform: {:?}", transform.translation);
                transform.rotation = Quat::from_rotation_z(ik.angle2+std::f32::consts::PI/2.0);

                // Set rotation to go from point 1 to point 2
                // let angle = (leg.points[2] - leg.points[1]).angle_between(Vec3::new(0.0, 1.0, 0.0));
                // transform.rotation = Quat::from_rotation_z(angle+std::f32::consts::PI/2.0);
            } else {
                transform.translation = leg.points[1];
            }

        }
        // Draw debug lines
        lines.line(leg.points[0], leg.points[1], 0.);
        lines.line(leg.points[1], leg.points[2], 0.);
    }
}

// 2-joint inverse kinematics with isosceles triangle
// fn inverse_kinematics(
//     target: Vec3,
//     base: Vec3,
//     length1: f32,
//     length2: f32,
// ) -> (f32, f32) {
//     let mut target = target - base;
//     let target_length = target.length();

//     // If target is too far away, clamp to max length
//     if target_length > length1 + length2 {
//         target = target.normalize() * (length1 + length2);
//     }

//     let cos_angle1 = (target_length.powi(2) + length1.powi(2) - length2.powi(2))
//         / (2.0 * target_length * length1);
//     let mut angle1 = cos_angle1.acos();

//     let cos_angle2 = (length1.powi(2) + length2.powi(2) - target_length.powi(2))
//         / (2.0 * length1 * length2);
//     let mut angle2 = cos_angle2.acos();

//     let angle1_sign = target.y.signum();
//     let angle2_sign = -target.x.signum();

//     angle1 *= angle1_sign;
//     angle2 *= angle2_sign;

//     (angle1, angle2)
// }

// fn inverse_kinematics(
//     target: Vec3,
//     base: Vec3,
//     leg_length: f32,
// ) -> (f32, Vec3, f32) {
//     let a = leg_length/2.0;
//     let b = (target - base).length();
//     let c = (target - base).normalize();
//     // let d = c * 

//     todo!()
// }

fn inverse_kinematics(
    mut target: Vec3,
    base: Vec3,
    length1: f32,
    length2: f32,
) -> SolvedIK {
    let mut dist = (target - base).length();

    if dist > length1 + length2 {
        target = target.normalize() * (length1 + length2);
        dist = (target - base).length();
    }

    let atan = (target.y - base.y).atan2(target.x - base.x);

    let cos_angle1 = (dist.powi(2) + length1.powi(2) - length2.powi(2)) / (2.0 * dist * length1);
    let angle1 = atan - cos_angle1.acos();

    let knee = Vec3::new(
        base.x + length1 * angle1.cos(),
        base.y + length1 * angle1.sin(),
        0.0,
    );

    // local cosAngle1 = ((l1 * l1) + (l2 * l2) - (dist * dist)) / (2 * l1 * l2)
    // let cos_angle2 = (length1.powi(2) + length2.powi(2) - dist.powi(2)) / (2.0 * length1 * length2);
    // local theta2 = math.pi - math.acos(cosAngle1)
    // let angle2 = std::f32::consts::PI - cos_angle2.acos();

    // calculate angle 2 by starting at the end of leg 1 and finding the angle to the target using atan2
    let angle2 = (target.y - knee.y).atan2(target.x - knee.x);

    // let angle2 = (target - knee).angle_between(Vec3::new(0.0, 1.0, 0.0));


    // (angle1, angle2)

    SolvedIK {
        angle1,
        knee,
        angle2,
        foot: target,
    }
}