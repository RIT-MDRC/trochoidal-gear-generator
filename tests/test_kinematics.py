from math import isclose, pi

import cadquery as cq

from rollergear.config import GearConfig
from rollergear.geometry import (
    apply_boolean_cut_batches,
    create_blank,
    create_hub,
    create_rollers,
    create_swept_cutters,
    output_angle,
    roller_centers_world,
)


def test_default_axes_and_ratio_close_after_period() -> None:
    config = GearConfig()
    end_q = 2 * pi * config.input_revolutions
    assert isclose(output_angle(config, end_q) - output_angle(config, 0), -2 * pi)
    start = roller_centers_world(config, 0)
    end = roller_centers_world(config, end_q)
    for a, b in zip(start, end):
        assert (a - b).Length < 1e-9


def test_default_gear_center_coordinates() -> None:
    config = GearConfig()
    assert (config.driven_center_x, config.driven_center_y, config.driven_center_z) == (
        0.0,
        0.0,
        config.driven_gear_radius_mm,
    )
    assert (config.input_axis_x, config.input_axis_y, config.input_axis_z_mm) == (
        0.0,
        config.pinion_disc_radius,
        0.0,
    )


def test_barrels_project_from_one_face_of_pinion_disc() -> None:
    config = GearConfig()
    hub = create_hub(config).BoundingBox()
    roller = create_rollers(config)[0].BoundingBox()

    assert isclose(hub.zmax, roller.zmin)
    assert hub.zmax <= config.input_axis_z_mm


def test_lengths_grow_from_position_coordinates_in_positive_axis_directions() -> None:
    config = GearConfig(driven_hub_length_mm=0.0)
    blank = create_blank(config).BoundingBox()
    roller = create_rollers(config)[0].BoundingBox()

    assert isclose(blank.ymin, 0.0)
    assert isclose(blank.ymax, config.driven_gear_thickness_mm)
    assert isclose(roller.zmin, config.input_axis_z_mm)
    assert isclose(roller.zmax, config.input_axis_z_mm + config.roller_length)


def test_driven_mounting_hub_projects_from_negative_y_face() -> None:
    config = GearConfig()
    blank = create_blank(config)
    bounds = blank.BoundingBox()

    assert isclose(bounds.ymin, -config.driven_hub_length_mm)
    assert isclose(bounds.ymax, config.driven_gear_thickness_mm)
    assert blank.isValid()
    assert len(blank.Solids()) == 1


def test_driven_base_radius_defaults_to_driven_gear_radius() -> None:
    default = GearConfig()
    overridden = GearConfig(driven_hub_radius_mm=20.0)

    assert isclose(default.driven_hub_radius, default.driven_gear_radius_mm)
    assert isclose(overridden.driven_hub_radius, 20.0)


def test_default_barrel_length_equals_driven_radius_and_can_be_overridden() -> None:
    default = GearConfig()
    overridden = GearConfig(roller_length_mm=12.0)

    assert isclose(default.roller_length, default.driven_gear_radius_mm)
    assert isclose(overridden.roller_length, 12.0)


def test_input_and_output_axes_are_perpendicular() -> None:
    input_axis = (0.0, 0.0, 1.0)
    output_axis = (0.0, 1.0, 0.0)
    assert sum(a * b for a, b in zip(input_axis, output_axis)) == 0.0


def test_default_motion_is_one_to_one() -> None:
    config = GearConfig()
    assert config.gear_ratio == 1.0
    assert isclose(abs(output_angle(config, 2 * pi) - output_angle(config, 0)), 2 * pi)


def test_pinion_and_driven_discs_have_equal_radii() -> None:
    config = GearConfig()
    assert isclose(config.pinion_disc_radius, config.driven_gear_radius_mm)


def test_barrel_pitch_circle_is_inset_by_barrel_radius() -> None:
    config = GearConfig()
    assert isclose(
        config.roller_pitch_radius + config.roller_radius,
        config.pinion_disc_radius,
    )


def test_driven_axis_is_lifted_for_edge_engagement() -> None:
    config = GearConfig()
    assert isclose(config.driven_center_z, config.driven_gear_radius_mm)


def test_raising_pinion_does_not_move_driven_gear() -> None:
    original = GearConfig()
    raised_pinion = GearConfig(input_axis_z_mm=12.0)

    assert raised_pinion.input_axis_z_mm == 12.0
    assert raised_pinion.driven_center_z == original.driven_center_z


def test_default_pinion_carrier_does_not_intersect_driven_blank() -> None:
    config = GearConfig()
    placed_blank = create_blank(config).translate(
        (config.driven_center_x, config.driven_center_y, config.driven_center_z)
    )
    assert create_hub(config).intersect(placed_blank).Volume() < 1e-7


def test_sweep_accounts_for_every_barrel_and_only_keeps_overlaps() -> None:
    config = GearConfig(samples_per_input_revolution=4)
    cutters, metrics = create_swept_cutters(config)
    assert metrics["sampled_motion_states"] == config.total_samples
    assert metrics["total_barrel_poses_considered"] == config.total_samples * config.roller_count
    assert len(cutters) == metrics["exact_overlapping_cutter_solids"]
    assert (
        metrics["exact_overlapping_cutter_solids"]
        + metrics["non_overlapping_poses_skipped"]
        == metrics["total_barrel_poses_considered"]
    )


def test_boolean_batch_accepts_valid_disconnected_results() -> None:
    left = cq.Solid.makeCylinder(2, 2, cq.Vector(-5, 0, 0), cq.Vector(0, 0, 1))
    right = cq.Solid.makeCylinder(2, 2, cq.Vector(5, 0, 0), cq.Vector(0, 0, 1))
    disconnected = cq.Compound.makeCompound([left, right])
    cutter = cq.Solid.makeCylinder(0.5, 2, cq.Vector(-5, 0, 0), cq.Vector(0, 0, 1))

    result, operations, fallbacks = apply_boolean_cut_batches(
        disconnected,
        [cutter],
        batch_size=8,
    )

    assert result.isValid()
    assert len(result.Solids()) == 2
    assert operations == 1
    assert fallbacks == 0


def test_sector_cut_defaults_on_only_for_one_to_one_ratio() -> None:
    assert GearConfig().use_sector_cut is True
    assert GearConfig(gear_ratio=2.0).use_sector_cut is False
    assert GearConfig(experimental_sector_cut=False).use_sector_cut is False
