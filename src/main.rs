use bevy::prelude::*;

const LEG_LENGTH: f32 = 150.0;

fn main(){
  App::new()
    .add_plugins(DefaultPlugins)
    .add_startup_system(setup)
    .add_system(leg_update)
    .run();
}

#[derive(Component)]
struct Leg {
    points: [Vec3; 3],
    // points: Vec<Vec3>,
}

// Bevy setup system
fn setup(mut commands: Commands) {
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
                    Vec3::new(0., -LEG_LENGTH, 0.),
                ],
            },
        )
    ).with_children(|parent| {
        parent.spawn((
            SpriteBundle {
                sprite: Sprite {
                    color: Color::rgb(0.25, 0.25, 0.75),
                    custom_size: Some(Vec2::new(25.0, LEG_LENGTH/2.0)),
                    ..default()
                },
                transform: Transform::from_translation(Vec3::new(0., 0., 0.)),
                ..default()
            }, 
        ));
    });
}

// Update leg segments
fn leg_update(
    // mut parents_query: Query<(Entity, &Children), With<Sprite>>,
    // mut transform_query: Query<&mut Transform, With<Sprite>>,
    mut query: Query<(&Leg, &Children)>,
    mut transform_query: Query<&mut Transform>,
) {
    for (leg, children) in query.iter_mut() {
        for (i, child) in children.iter().enumerate() {
            let mut transform = transform_query.get_mut(*child).unwrap();
            transform.translation = leg.points[i];
        }
    }
}