from __future__ import annotations

from math import cos, pi, radians, sin
from pathlib import Path
from typing import Callable

import cadquery as cq

from .config import GearConfig


Y_AXIS = cq.Vector(0, 1, 0)
Z_AXIS = cq.Vector(0, 0, 1)
ORIGIN = cq.Vector(0, 0, 0)
ProgressCallback = Callable[[str, int, int], None]


def rotate_y(vector: cq.Vector, angle_rad: float) -> cq.Vector:
    c, s = cos(angle_rad), sin(angle_rad)
    return cq.Vector(c * vector.x + s * vector.z, vector.y, -s * vector.x + c * vector.z)


def roller_centers_world(config: GearConfig, input_angle_rad: float) -> list[cq.Vector]:
    centers = []
    for k in range(config.roller_count):
        angle = input_angle_rad + 2 * pi * k / config.roller_count
        centers.append(
            cq.Vector(
                config.input_axis_x + config.roller_pitch_radius * cos(angle),
                config.input_axis_y + config.roller_pitch_radius * sin(angle),
                config.input_axis_z_mm,
            )
        )
    return centers


def output_angle(config: GearConfig, input_angle_rad: float) -> float:
    return (
        config.output_direction * input_angle_rad / config.gear_ratio
        + radians(config.output_phase_deg)
    )


def create_blank(config: GearConfig) -> cq.Shape:
    # The driven coordinates locate the negative-Y start face. Thickness grows
    # from that plane toward +Y instead of straddling the coordinate.
    outer = cq.Solid.makeCylinder(
        config.driven_gear_radius_mm,
        config.driven_gear_thickness_mm,
        ORIGIN,
        Y_AXIS,
    )
    if config.driven_hub_length_mm > 0:
        hub = cq.Solid.makeCylinder(
            config.driven_hub_radius,
            config.driven_hub_length_mm,
            cq.Vector(0, -config.driven_hub_length_mm, 0),
            Y_AXIS,
        )
        outer = outer.fuse(hub)
    hole = cq.Solid.makeCylinder(
        config.driven_shaft_hole_mm / 2,
        config.driven_hub_length_mm + config.driven_gear_thickness_mm + 2,
        cq.Vector(0, -config.driven_hub_length_mm - 1, 0),
        Y_AXIS,
    )
    return outer.cut(hole)


def create_hub(config: GearConfig) -> cq.Shape:
    # Barrels start at input_axis_z and grow toward +Z. Put the pinion disc
    # behind that start plane so they project from its positive-Z face.
    hub_face_z = config.input_axis_z_mm - config.pinion_face_gap_mm
    hub_base_z = hub_face_z - config.pinion_hub_thickness_mm
    hub = cq.Solid.makeCylinder(
        config.pinion_disc_radius,
        config.pinion_hub_thickness_mm,
        cq.Vector(
            config.input_axis_x,
            config.input_axis_y,
            hub_base_z,
        ),
        Z_AXIS,
    )
    hole = cq.Solid.makeCylinder(
        config.pinion_shaft_hole_mm / 2,
        config.pinion_hub_thickness_mm + 2,
        cq.Vector(
            config.input_axis_x,
            config.input_axis_y,
            hub_base_z - 1,
        ),
        Z_AXIS,
    )
    return hub.cut(hole)


def create_roller_part(config: GearConfig) -> cq.Shape:
    """Create one printable barrel roller growing from the CAD origin toward +Z."""
    return cq.Solid.makeCylinder(
        config.roller_radius,
        config.roller_length,
        ORIGIN,
        Z_AXIS,
    )


def create_rollers(config: GearConfig, clearance: float = 0.0) -> list[cq.Shape]:
    rollers = []
    for center in roller_centers_world(config, 0.0):
        # Clearance expands the cutter below the requested start plane as well
        # as beyond its positive-Z end; the printable barrel itself starts at
        # the exact input coordinate.
        rollers.append(
            cq.Solid.makeCylinder(
                config.roller_radius + clearance,
                config.roller_length + (2 * clearance if clearance else 0),
                center - Z_AXIS.multiply(clearance),
                Z_AXIS,
            )
        )
    return rollers


def create_pinion_assembly(config: GearConfig) -> cq.Shape:
    """Pinion disc plus all projecting barrel teeth in their assembly positions."""
    return cq.Compound.makeCompound([create_hub(config), *create_rollers(config)])


def create_right_angle_assembly(driven_gear: cq.Shape, config: GearConfig) -> cq.Shape:
    """Both perpendicular discs at input angle zero for orientation inspection."""
    placed_gear = driven_gear.rotate(
        (0, 0, 0),
        (0, 1, 0),
        config.output_phase_deg,
    ).translate(
        (config.driven_center_x, config.driven_center_y, config.driven_center_z)
    )
    return cq.Compound.makeCompound([placed_gear, create_pinion_assembly(config)])


def create_swept_cutters(
    config: GearConfig,
    progress: ProgressCallback | None = None,
    blank: cq.Shape | None = None,
    overlap_volume_tolerance_mm3: float = 1e-9,
) -> tuple[list[cq.Shape], dict[str, int]]:
    """Return only barrel poses proven to overlap the original driven blank."""
    blank = blank or create_blank(config)
    blank_bounds = blank.BoundingBox()
    cutters: list[cq.Shape] = []
    total_poses = config.total_samples * config.roller_count
    aabb_candidates = 0
    exact_overlaps = 0
    cut_radius = config.roller_radius + config.clearance_mm
    cut_length = config.roller_length + 2 * config.clearance_mm

    for sample in range(config.total_samples):
        q = 2 * pi * sample / config.samples_per_input_revolution
        inverse_output = -output_angle(config, q)
        for roller_index, world_center in enumerate(roller_centers_world(config, q)):
            relative_center = world_center - cq.Vector(
                config.driven_center_x,
                config.driven_center_y,
                config.driven_center_z,
            )
            center = rotate_y(relative_center, inverse_output)
            direction = rotate_y(Z_AXIS, inverse_output)

            cutter = cq.Solid.makeCylinder(
                cut_radius,
                cut_length,
                center - direction.multiply(config.clearance_mm),
                direction,
            )

            # Broad phase: unlike the old center-distance shortcut, the actual
            # transformed cylinder bounds include its full length and direction.
            bounds = cutter.BoundingBox()
            bounds_overlap = not (
                bounds.xmax < blank_bounds.xmin
                or bounds.xmin > blank_bounds.xmax
                or bounds.ymax < blank_bounds.ymin
                or bounds.ymin > blank_bounds.ymax
                or bounds.zmax < blank_bounds.zmin
                or bounds.zmin > blank_bounds.zmax
            )
            if bounds_overlap:
                aabb_candidates += 1
                # Narrow phase uses the original simple blank, which is much
                # cheaper than cutting the increasingly complex generated gear.
                if blank.intersect(cutter).Volume() > overlap_volume_tolerance_mm3:
                    cutters.append(cutter)
                    exact_overlaps += 1

            if progress is not None:
                completed = sample * config.roller_count + roller_index + 1
                progress("Prechecking barrel overlap", completed, total_poses)

    if not cutters:
        raise ValueError(
            "Roller sweep never intersects the driven blank; adjust input-axis position, "
            "roller length, or blank dimensions"
        )
    return cutters, {
        "sampled_motion_states": config.total_samples,
        "barrels_per_state": config.roller_count,
        "total_barrel_poses_considered": total_poses,
        "aabb_overlap_candidates": aabb_candidates,
        "exact_overlapping_cutter_solids": exact_overlaps,
        "non_overlapping_poses_skipped": total_poses - exact_overlaps,
    }


def validate_all_cutter_clearance(
    gear: cq.Shape,
    cutters: list[cq.Shape],
    progress: ProgressCallback | None = None,
    volume_tolerance_mm3: float = 1e-7,
) -> tuple[float, int]:
    """Check the final gear against every expanded barrel pose used to cut it."""
    maximum_overlap = 0.0
    overlapping_poses = 0
    for index, cutter in enumerate(cutters, start=1):
        overlap = gear.intersect(cutter).Volume()
        maximum_overlap = max(maximum_overlap, overlap)
        overlapping_poses += int(overlap > volume_tolerance_mm3)
        if progress is not None:
            progress("Verifying barrel clearance", index, len(cutters))
    return maximum_overlap, overlapping_poses


def create_sector_prism(config: GearConfig, half_angle_rad: float) -> cq.Shape:
    """Create a local driven-gear wedge spanning its hub and +Y thickness."""
    # Keep the outer chord beyond the circular blank so intersecting this prism
    # with the blank retains the true cylindrical edge rather than faceting it.
    outer_radius = 2.0 * config.driven_gear_radius_mm
    base_y = -config.driven_hub_length_mm
    points = [
        cq.Vector(0, base_y, 0),
        cq.Vector(
            outer_radius * cos(-half_angle_rad),
            base_y,
            outer_radius * sin(-half_angle_rad),
        ),
        cq.Vector(
            outer_radius * cos(half_angle_rad),
            base_y,
            outer_radius * sin(half_angle_rad),
        ),
    ]
    wire = cq.Wire.makePolygon(points, close=True)
    return cq.Solid.extrudeLinear(
        wire,
        [],
        cq.Vector(
            0,
            config.driven_hub_length_mm + config.driven_gear_thickness_mm,
            0,
        ),
    )


def apply_boolean_cut_batches(
    shape: cq.Shape,
    cutters: list[cq.Shape],
    batch_size: int,
    progress: ProgressCallback | None = None,
) -> tuple[cq.Shape, int, int]:
    """Cut explicit tools in batches, falling back on topology failure."""
    result = shape
    boolean_operations = 0
    sequential_fallback_batches = 0
    for start in range(0, len(cutters), batch_size):
        batch = cutters[start : start + batch_size]
        try:
            candidate = result.cut(*batch)
            if not candidate.isValid():
                raise ValueError("batched Boolean cut produced invalid topology")
            result = candidate
            boolean_operations += 1
        except Exception:
            sequential_fallback_batches += 1
            for cutter in batch:
                result = result.cut(cutter)
                boolean_operations += 1
        if progress is not None:
            progress("Applying Boolean cuts", start + len(batch), len(cutters))
    return result, boolean_operations, sequential_fallback_batches


def cut_driven_gear(
    config: GearConfig,
    progress: ProgressCallback | None = None,
    verify_clearance: bool = False,
) -> tuple[cq.Shape, dict[str, float | int | None]]:
    blank = create_blank(config)
    initial_volume = blank.Volume()
    cutters, precheck_metrics = create_swept_cutters(config, progress, blank)
    batch_size = config.boolean_cut_batch_size
    sector_cutter_solids: int | None = None
    symmetry_sectors = 1
    if config.use_sector_cut:
        symmetry_sectors = config.symmetry_sector_count
        exact_half_angle = pi / symmetry_sectors
        # Work beyond both exact boundaries by one driven-output sample. Cutting
        # on the expanded shape avoids coincident cutter/slice faces; trimming
        # afterward restores the exact repeatable cake slice.
        boundary_overlap = 2 * pi / (
            config.samples_per_input_revolution * config.gear_ratio
        )
        exact_prism = create_sector_prism(config, exact_half_angle)
        work_prism = create_sector_prism(
            config,
            exact_half_angle + boundary_overlap,
        )
        work_sector = blank.intersect(work_prism)
        sector_cutters = [
            cutter
            for cutter in cutters
            if work_sector.intersect(cutter).Volume() > 1e-9
        ]
        sector_cutter_solids = len(sector_cutters)
        if not sector_cutters:
            raise ValueError("Experimental sector has no overlapping barrel cutters")

        # Smaller batches are more stable on the temporary radial boundaries.
        sector_batch_size = min(batch_size, 8)
        cut_sector, boolean_operations, sequential_fallback_batches = (
            apply_boolean_cut_batches(
                work_sector,
                sector_cutters,
                sector_batch_size,
                progress,
            )
        )
        exact_sector = cut_sector.intersect(exact_prism)
        pieces = [
            exact_sector.rotate(
                (0, 0, 0),
                (0, 1, 0),
                360 * index / symmetry_sectors,
            )
            for index in range(symmetry_sectors)
        ]
        gear = pieces[0]
        for index, piece in enumerate(pieces[1:], start=2):
            gear = gear.fuse(piece)
            if progress is not None:
                progress("Replicating symmetry sectors", index, symmetry_sectors)
        if not gear.isValid():
            raise ValueError("Experimental sector reconstruction produced invalid topology")
    else:
        # Pass cutter solids directly to OpenCascade in bounded batches. This
        # avoids rebuilding the increasingly complex gear once per barrel pose.
        gear, boolean_operations, sequential_fallback_batches = (
            apply_boolean_cut_batches(blank, cutters, batch_size, progress)
        )

    maximum_overlap: float | None = None
    overlapping_poses: int | None = None
    if verify_clearance:
        maximum_overlap, overlapping_poses = validate_all_cutter_clearance(
            gear,
            cutters,
            progress,
        )
        if overlapping_poses:
            raise ValueError(
                f"Clearance validation found {overlapping_poses} barrel poses with residual "
                f"overlap (maximum {maximum_overlap:.6g} mm^3)"
            )

    final_volume = gear.Volume()
    removed_fraction = (initial_volume - final_volume) / initial_volume
    if removed_fraction < 1e-5:
        raise ValueError("Sweep touched the blank but removed no meaningful material")
    if removed_fraction > 0.85:
        raise ValueError(f"Sweep removed {removed_fraction:.1%} of the blank; configuration is invalid")

    return gear, {
        "initial_blank_volume_mm3": initial_volume,
        "final_gear_volume_mm3": final_volume,
        "removed_volume_fraction": removed_fraction,
        "cutter_solids": len(cutters),
        "boolean_cut_batch_size": batch_size,
        "boolean_operations": boolean_operations,
        "sequential_fallback_batches": sequential_fallback_batches,
        "experimental_sector_cut": config.use_sector_cut,
        "symmetry_sectors": symmetry_sectors,
        "sector_cutter_solids": sector_cutter_solids,
        "connected_solid_count": len(gear.Solids()),
        **precheck_metrics,
        "maximum_residual_barrel_overlap_mm3": maximum_overlap,
        "overlapping_barrel_poses": overlapping_poses,
        "clearance_verification_performed": verify_clearance,
    }


def export_shape(shape: cq.Shape, step_path: Path, stl_path: Path, config: GearConfig) -> None:
    cq.exporters.export(shape, str(step_path))
    cq.exporters.export(
        shape,
        str(stl_path),
        tolerance=config.stl_tolerance_mm,
        angularTolerance=radians(config.stl_angular_tolerance_deg),
    )
