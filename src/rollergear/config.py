from __future__ import annotations

from dataclasses import asdict, dataclass
from math import isclose
from typing import Any


@dataclass(frozen=True)
class GearConfig:
    """Dimensions are millimetres; angles are degrees at the user boundary."""

    module_mm: float = 2.0
    roller_count: int = 10
    roller_diameter_coeff: float = 0.8
    roller_pitch_radius_mm: float | None = None
    roller_length_mm: float | None = None
    pinion_hub_radius_mm: float = 7.0
    pinion_disc_radius_mm: float | None = None
    pinion_hub_thickness_mm: float = 5.0
    pinion_shaft_hole_mm: float = 5.0
    pinion_face_gap_mm: float = 0.0

    driven_gear_radius_mm: float = 30.0
    driven_gear_thickness_mm: float = 10.0
    driven_shaft_hole_mm: float = 10.0
    driven_hub_radius_mm: float | None = None
    driven_hub_length_mm: float = 5.0
    gear_ratio: float = 1.0

    # Default origins describe the perpendicular layout directly. Driven
    # thickness starts at Y=0 and grows toward +Y; barrels start at Z=0 and
    # grow toward +Z. The driven circular center remains lifted by its radius.
    input_axis_x_mm: float = 0.0
    input_axis_y_mm: float | None = None
    input_axis_z_mm: float = 0.0
    driven_center_x_mm: float = 0.0
    driven_center_y_mm: float | None = None
    driven_center_z_mm: float | None = None
    output_phase_deg: float = 0.0
    output_direction: int = -1

    samples_per_input_revolution: int = 120
    clearance_mm: float = 0.15
    boolean_cut_batch_size: int = 32
    experimental_sector_cut: bool | None = None
    stl_tolerance_mm: float = 0.08
    stl_angular_tolerance_deg: float = 0.5

    @property
    def roller_pitch_radius(self) -> float:
        if self.roller_pitch_radius_mm is not None:
            return self.roller_pitch_radius_mm
        # Put the complete barrel footprint on the shared pinion radius.
        return self.driven_gear_radius_mm - self.roller_radius

    @property
    def roller_radius(self) -> float:
        return self.module_mm * self.roller_diameter_coeff / 2.0

    @property
    def roller_length(self) -> float:
        if self.roller_length_mm is not None:
            return self.roller_length_mm
        return self.driven_gear_radius_mm

    @property
    def pinion_disc_radius(self) -> float:
        # The user-requested geometry uses equal pinion and driven radii.
        return self.driven_gear_radius_mm

    @property
    def input_axis_x(self) -> float:
        return self.input_axis_x_mm

    @property
    def input_axis_y(self) -> float:
        if self.input_axis_y_mm is not None:
            return self.input_axis_y_mm
        return self.pinion_disc_radius

    @property
    def driven_center_y(self) -> float:
        if self.driven_center_y_mm is not None:
            return self.driven_center_y_mm
        return 0.0

    @property
    def driven_center_x(self) -> float:
        return self.driven_center_x_mm

    @property
    def driven_center_z(self) -> float:
        if self.driven_center_z_mm is not None:
            return self.driven_center_z_mm
        # The driven axis has its own fixed world position. Moving the pinion
        # with --input-z must not implicitly move the driven gear.
        return self.driven_gear_radius_mm

    @property
    def driven_hub_radius(self) -> float:
        if self.driven_hub_radius_mm is not None:
            return self.driven_hub_radius_mm
        return self.driven_gear_radius_mm

    @property
    def input_revolutions(self) -> int:
        rounded = round(self.gear_ratio)
        if not isclose(self.gear_ratio, rounded, abs_tol=1e-9):
            raise ValueError("A closed STEP/STL requires an integer gear ratio in this proof of concept")
        return int(rounded)

    @property
    def total_samples(self) -> int:
        return self.samples_per_input_revolution * self.input_revolutions

    @property
    def use_sector_cut(self) -> bool:
        if self.experimental_sector_cut is not None:
            return self.experimental_sector_cut
        return isclose(self.gear_ratio, 1.0, abs_tol=1e-9)

    def validate(self) -> None:
        positive = {
            "module_mm": self.module_mm,
            "roller_count": self.roller_count,
            "roller_diameter_coeff": self.roller_diameter_coeff,
            "roller_length_mm_resolved": self.roller_length,
            "driven_gear_radius_mm": self.driven_gear_radius_mm,
            "driven_gear_thickness_mm": self.driven_gear_thickness_mm,
            "driven_hub_radius_mm_resolved": self.driven_hub_radius,
            "gear_ratio": self.gear_ratio,
            "samples_per_input_revolution": self.samples_per_input_revolution,
            "boolean_cut_batch_size": self.boolean_cut_batch_size,
        }
        bad = [name for name, value in positive.items() if value <= 0]
        if bad:
            raise ValueError(f"These values must be positive: {', '.join(bad)}")
        if self.output_direction not in (-1, 1):
            raise ValueError("output_direction must be -1 or +1")
        if self.roller_radius >= self.driven_gear_radius_mm:
            raise ValueError("roller radius must be smaller than the shared gear radius")
        if self.pinion_disc_radius_mm is not None and not isclose(
            self.pinion_disc_radius_mm,
            self.driven_gear_radius_mm,
            abs_tol=1e-9,
        ):
            raise ValueError("pinion_disc_radius_mm must equal driven_gear_radius_mm")
        if self.pinion_face_gap_mm < 0:
            raise ValueError("pinion_face_gap_mm must be zero or positive")
        if self.driven_shaft_hole_mm >= 2 * self.driven_gear_radius_mm:
            raise ValueError("Driven shaft hole consumes the entire gear blank")
        if self.driven_hub_length_mm < 0:
            raise ValueError("driven_hub_length_mm must be zero or positive")
        if self.driven_hub_radius <= self.driven_shaft_hole_mm / 2:
            raise ValueError("Driven hub radius must be larger than the driven hole radius")
        if self.driven_hub_radius > self.driven_gear_radius_mm:
            raise ValueError("Driven hub radius cannot exceed the driven gear radius")
        if self.use_sector_cut:
            if not isclose(self.gear_ratio, 1.0, abs_tol=1e-9):
                raise ValueError("Sector cutting is supported only for a 1:1 gear ratio")
            if self.roller_count < 3:
                raise ValueError("Sector cutting requires at least 3 barrels")
            if self.samples_per_input_revolution % self.roller_count:
                raise ValueError(
                    "Sector cutting requires samples to be divisible by the barrel count"
                )
        _ = self.input_revolutions

    def as_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result.update(
            roller_pitch_radius_mm_resolved=self.roller_pitch_radius,
            roller_radius_mm=self.roller_radius,
            roller_length_mm_resolved=self.roller_length,
            pinion_disc_radius_mm_resolved=self.pinion_disc_radius,
            input_axis_origin_mm=[self.input_axis_x, self.input_axis_y, self.input_axis_z_mm],
            input_axis_direction=[0.0, 0.0, 1.0],
            driven_center_x_mm_resolved=self.driven_center_x,
            driven_center_y_mm_resolved=self.driven_center_y,
            driven_center_z_mm_resolved=self.driven_center_z,
            driven_start_face_y_mm=self.driven_center_y,
            driven_positive_face_y_mm=self.driven_center_y + self.driven_gear_thickness_mm,
            driven_hub_negative_face_y_mm=self.driven_center_y - self.driven_hub_length_mm,
            driven_hub_radius_mm_resolved=self.driven_hub_radius,
            output_axis_origin_mm=[self.driven_center_x, self.driven_center_y, self.driven_center_z],
            output_axis_direction=[0.0, 1.0, 0.0],
            total_samples=self.total_samples,
            experimental_sector_cut_resolved=self.use_sector_cut,
        )
        return result
