from __future__ import annotations

import argparse
import hashlib
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import TextIO

from .config import GearConfig
from .geometry import (
    create_hub,
    create_pinion_assembly,
    create_right_angle_assembly,
    create_roller_part,
    cut_driven_gear,
    export_shape,
)


class ProgressReporter:
    """Small dependency-free terminal progress display."""

    def __init__(self, stream: TextIO = sys.stderr) -> None:
        self.stream = stream
        self.interactive = bool(getattr(stream, "isatty", lambda: False)())
        self.previous_stage: str | None = None
        self.previous_bucket = -1

    def update(self, stage: str, current: int, total: int) -> None:
        total = max(total, 1)
        percent = min(100, round(current * 100 / total))
        bucket = percent if self.interactive else percent // 10
        stage_changed = stage != self.previous_stage
        if not stage_changed and bucket == self.previous_bucket and current != total:
            return

        if self.interactive:
            filled = min(24, round(percent * 24 / 100))
            bar = "#" * filled + "-" * (24 - filled)
            self.stream.write(f"\r{stage:<24} [{bar}] {percent:3d}%  {current}/{total}")
            if current == total:
                self.stream.write("\n")
        else:
            self.stream.write(f"{stage}: {percent}% ({current}/{total})\n")
        self.stream.flush()
        self.previous_stage = stage
        self.previous_bucket = bucket

    def status(self, message: str) -> None:
        self.stream.write(f"{message}\n")
        self.stream.flush()


def _file_metadata(path: Path) -> dict[str, str | int]:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return {
        "filename": path.name,
        "size_bytes": path.stat().st_size,
        "sha256": digest.hexdigest(),
    }


def _prepare_output_directory(
    requested: Path | None,
    base: Path = Path("output"),
    now: datetime | None = None,
) -> Path:
    """Create an explicit directory or a unique timestamped default directory."""
    if requested is not None:
        requested.mkdir(parents=True, exist_ok=True)
        return requested

    base.mkdir(parents=True, exist_ok=True)
    timestamp = (now or datetime.now().astimezone()).strftime("%Y%m%d-%H%M%S")
    stem = f"run-{timestamp}"
    suffix = 1
    while True:
        name = stem if suffix == 1 else f"{stem}-{suffix}"
        candidate = base / name
        try:
            candidate.mkdir()
            return candidate
        except FileExistsError:
            suffix += 1


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Generate a 90-degree swept-volume roller gear")
    p.add_argument("--module", type=float, default=2.0, dest="module_mm")
    p.add_argument("--rollers", type=int, default=10, dest="roller_count")
    p.add_argument("--roller-coeff", type=float, default=4.0, dest="roller_diameter_coeff")
    p.add_argument(
        "--roller-length",
        type=float,
        default=None,
        dest="roller_length_mm",
        help="Barrel length in mm; defaults to --driven-radius",
    )
    p.add_argument(
        "--pinion-disc-radius",
        type=float,
        default=None,
        dest="pinion_disc_radius_mm",
        help="Input pinion-disc radius; defaults to --driven-radius (mm)",
    )
    p.add_argument(
        "--pinion-face-gap",
        type=float,
        default=0.0,
        dest="pinion_face_gap_mm",
        help="Axial gap between the pinion disc face and the projecting barrel teeth (mm)",
    )
    p.add_argument("--gear-ratio", type=float, default=1.0)
    p.add_argument("--driven-radius", type=float, default=30.0, dest="driven_gear_radius_mm")
    p.add_argument("--driven-thickness", type=float, default=10.0, dest="driven_gear_thickness_mm")
    p.add_argument("--driven-hole", type=float, default=10.0, dest="driven_shaft_hole_mm")
    p.add_argument(
        "--driven-hub-radius",
        type=float,
        default=None,
        dest="driven_hub_radius_mm",
        help="Radius of the driven gear base on its negative-Y face; defaults to --driven-radius",
    )
    p.add_argument(
        "--driven-hub-length",
        type=float,
        default=5.0,
        dest="driven_hub_length_mm",
        help="Length of the driven mounting hub toward -Y; use 0 to disable it (mm)",
    )
    p.add_argument("--input-x", type=float, default=0.0, dest="input_axis_x_mm")
    p.add_argument("--input-y", type=float, default=None, dest="input_axis_y_mm")
    p.add_argument("--input-z", type=float, default=0.0, dest="input_axis_z_mm")
    p.add_argument("--driven-x", type=float, default=0.0, dest="driven_center_x_mm")
    p.add_argument(
        "--driven-y",
        type=float,
        default=None,
        dest="driven_center_y_mm",
        help="Driven-disc center along Y; defaults to 0 mm",
    )
    p.add_argument(
        "--driven-z",
        type=float,
        default=None,
        dest="driven_center_z_mm",
        help="Driven-axis Z height; defaults to one shared radius above the barrel plane",
    )
    # Hidden compatibility aliases for commands written before the shorter names.
    p.add_argument("--input-axis-x", type=float, dest="input_axis_x_mm", default=argparse.SUPPRESS, help=argparse.SUPPRESS)
    p.add_argument("--input-axis-y", type=float, dest="input_axis_y_mm", default=argparse.SUPPRESS, help=argparse.SUPPRESS)
    p.add_argument("--input-axis-z", type=float, dest="input_axis_z_mm", default=argparse.SUPPRESS, help=argparse.SUPPRESS)
    p.add_argument("--driven-center-y", type=float, dest="driven_center_y_mm", default=argparse.SUPPRESS, help=argparse.SUPPRESS)
    p.add_argument("--driven-center-z", type=float, dest="driven_center_z_mm", default=argparse.SUPPRESS, help=argparse.SUPPRESS)
    p.add_argument("--phase", type=float, default=0.0, dest="output_phase_deg")
    p.add_argument("--direction", type=int, choices=(-1, 1), default=-1, dest="output_direction")
    p.add_argument("--samples", type=int, default=120, dest="samples_per_input_revolution")
    p.add_argument("--clearance", type=float, default=0.15, dest="clearance_mm")
    p.add_argument(
        "--cut-batch-size",
        type=int,
        default=32,
        dest="boolean_cut_batch_size",
        help="Barrel cutters per OpenCascade Boolean operation; use 1 for legacy sequential cuts",
    )
    p.add_argument(
        "--experimental-sector-cut",
        action=argparse.BooleanOptionalAction,
        default=None,
        help=(
            "Cut one of rollers * ratio sectors and reconstruct the driven gear; "
            "enabled by default; use --no-experimental-sector-cut to disable"
        ),
    )
    p.add_argument(
        "--verify-clearance",
        action="store_true",
        help="Run the optional post-cut intersection check for every retained barrel pose",
    )
    p.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output directory; omitted creates output/run-YYYYMMDD-HHMMSS",
    )
    p.add_argument("--preview", action="store_true", help="Use 30 samples/revolution for a quick draft")
    return p


def main(argv: list[str] | None = None) -> int:
    raw_arguments = list(sys.argv[1:] if argv is None else argv)
    args = parser().parse_args(raw_arguments)
    parsed_arguments = {
        key: str(value) if isinstance(value, Path) else value
        for key, value in vars(args).items()
    }
    config_values = vars(args).copy()
    config_values.pop("output")
    preview = config_values.pop("preview")
    verify_clearance = config_values.pop("verify_clearance")
    if preview:
        config_values["samples_per_input_revolution"] = 30
    config = GearConfig(**config_values)

    try:
        config.validate()
        output_dir = _prepare_output_directory(args.output)
        progress = ProgressReporter()
        progress.status(
            f"Starting generation: {config.total_samples} synchronized states "
            f"({config.samples_per_input_revolution} samples/input revolution)"
        )
        gear, metrics = cut_driven_gear(
            config,
            progress.update,
            verify_clearance=verify_clearance,
        )

        driven_step = output_dir / "driven_gear.step"
        driven_stl = output_dir / "driven_gear.stl"
        hub_step = output_dir / "roller_pinion_hub.step"
        hub_stl = output_dir / "roller_pinion_hub.stl"
        roller_step = output_dir / "roller.step"
        roller_stl = output_dir / "roller.stl"
        pinion_step = output_dir / "roller_pinion_assembly.step"
        pinion_stl = output_dir / "roller_pinion_assembly.stl"
        assembly_step = output_dir / "right_angle_assembly.step"
        assembly_stl = output_dir / "right_angle_assembly.stl"
        generated_paths = [
            driven_step,
            driven_stl,
            hub_step,
            hub_stl,
            roller_step,
            roller_stl,
            pinion_step,
            pinion_stl,
            assembly_step,
            assembly_stl,
        ]

        progress.status("Exporting driven gear STEP and STL...")
        export_shape(gear, driven_step, driven_stl, config)
        hub = create_hub(config)
        progress.status("Exporting pinion hub STEP and STL...")
        export_shape(hub, hub_step, hub_stl, config)
        roller = create_roller_part(config)
        progress.status("Exporting roller STEP and STL...")
        export_shape(roller, roller_step, roller_stl, config)
        pinion = create_pinion_assembly(config)
        progress.status("Exporting complete pinion with projecting barrel teeth...")
        export_shape(pinion, pinion_step, pinion_stl, config)
        assembly = create_right_angle_assembly(gear, config)
        progress.status("Exporting 90-degree two-disc assembly preview...")
        export_shape(assembly, assembly_step, assembly_stl, config)

        solids = gear.Solids()
        report = {
            "status": "generated_not_mechanically_validated",
            "units": "millimetres",
            "kinematics_valid": True,
            "axes_perpendicular": True,
            "shaft_angle_deg": 90.0,
            "driven_circular_face_positive_direction": "+Y",
            "driven_start_face_world_y_mm": config.driven_center_y,
            "driven_positive_face_world_y_mm": config.driven_center_y + config.driven_gear_thickness_mm,
            "driven_axis_world_z_mm": config.driven_center_z,
            "pinion_disc_radius_mm": config.pinion_disc_radius,
            "driven_gear_radius_mm": config.driven_gear_radius_mm,
            "motion_ratio_input_to_output": "1:1" if config.gear_ratio == 1 else f"{config.gear_ratio}:1",
            "pinion_barrels_project_from_disc_face": True,
            "mesh_watertight": gear.isValid() and len(solids) == 1,
            "positive_volume": gear.Volume() > 0,
            "connected_solid_count": len(solids),
            "max_penetration_mm": None,
            "minimum_clearance_mm": None,
            "all_sampled_barrel_poses_clear": (
                metrics["overlapping_barrel_poses"] == 0
                if metrics["clearance_verification_performed"]
                else None
            ),
            "minimum_engaged_rollers": None,
            "maximum_backlash_deg": None,
            "torque_transfer_feasible": None,
            "convergence_passed": False,
            "physical_test_completed": False,
            "warnings": [
                "The prescribed motion generated a solid, but torque transfer and continuous contact are unverified.",
                "Post-cut clearance verification is optional; use --verify-clearance when required.",
                "Assembly STL files contain multiple separate shells; use STEP for CAD inspection.",
                "Print at low load and inspect a full hand-driven cycle before powered testing.",
            ],
            "geometry": metrics,
            "config": config.as_dict(),
        }
        (output_dir / "validation_report.json").write_text(json.dumps(report, indent=2) + "\n")

        progress.status("Writing validation report and calculating file checksums...")
        metadata = {
            "schema_version": 1,
            "generator": "right-angle-roller-gear",
            "generator_version": "0.1.0",
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "output_directory": str(output_dir.resolve()),
            "units": {
                "length": "millimetres",
                "cli_angles": "degrees",
                "internal_angles": "radians",
            },
            "invocation": {
                "command": ["python", "-m", "rollergear", *raw_arguments],
                "raw_arguments": raw_arguments,
                "parsed_arguments": parsed_arguments,
            },
            "resolved_config": config.as_dict(),
            "derived_geometry": metrics,
            "generated_files": [_file_metadata(path) for path in generated_paths],
            "validation_report": "validation_report.json",
        }
        (output_dir / "model_metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
        progress.status(f"Complete. Files written to {output_dir.resolve()}")
        return 0
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
