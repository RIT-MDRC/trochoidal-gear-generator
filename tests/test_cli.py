from rollergear.cli import parser


def test_clearance_verification_is_opt_in() -> None:
    assert parser().parse_args([]).verify_clearance is False
    assert parser().parse_args(["--verify-clearance"]).verify_clearance is True


def test_boolean_cut_batching_defaults_to_32_and_can_be_disabled() -> None:
    assert parser().parse_args([]).boolean_cut_batch_size == 32
    assert parser().parse_args(["--cut-batch-size", "1"]).boolean_cut_batch_size == 1


def test_driven_hole_defaults_to_10_mm() -> None:
    assert parser().parse_args([]).driven_shaft_hole_mm == 10.0


def test_roller_coefficient_defaults_to_4() -> None:
    assert parser().parse_args([]).roller_diameter_coeff == 4.0


def test_driven_hub_defaults() -> None:
    args = parser().parse_args([])
    assert args.driven_hub_radius_mm is None
    assert args.driven_hub_length_mm == 5.0


def test_sector_cut_can_be_enabled_or_disabled_explicitly() -> None:
    assert parser().parse_args([]).experimental_sector_cut is None
    assert parser().parse_args(["--experimental-sector-cut"]).experimental_sector_cut is True
    assert parser().parse_args(["--no-experimental-sector-cut"]).experimental_sector_cut is False


def test_short_position_arguments_map_to_model_coordinates() -> None:
    args = parser().parse_args(
        [
            "--input-x", "1", "--input-y", "2", "--input-z", "3",
            "--driven-x", "4", "--driven-y", "5", "--driven-z", "6",
        ]
    )
    assert (args.input_axis_x_mm, args.input_axis_y_mm, args.input_axis_z_mm) == (1, 2, 3)
    assert (args.driven_center_x_mm, args.driven_center_y_mm, args.driven_center_z_mm) == (4, 5, 6)


def test_legacy_position_arguments_remain_compatible() -> None:
    args = parser().parse_args(
        ["--input-axis-x", "1", "--input-axis-y", "2", "--input-axis-z", "3"]
    )
    assert (args.input_axis_x_mm, args.input_axis_y_mm, args.input_axis_z_mm) == (1, 2, 3)
