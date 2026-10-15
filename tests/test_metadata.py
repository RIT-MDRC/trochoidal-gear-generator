from datetime import datetime, timezone
from pathlib import Path

from rollergear.cli import _file_metadata, _prepare_output_directory


def test_file_metadata_records_name_size_and_sha256(tmp_path: Path) -> None:
    model = tmp_path / "part.step"
    model.write_bytes(b"cad-data")

    metadata = _file_metadata(model)

    assert metadata == {
        "filename": "part.step",
        "size_bytes": 8,
        "sha256": "37914b5659d4adf49492c1e9d3d7de7886bdf75c6595a6c88f6c2c321fb44190",
    }


def test_default_output_creates_unique_timestamped_directories(tmp_path: Path) -> None:
    now = datetime(2026, 10, 15, 12, 34, 56, tzinfo=timezone.utc)
    base = tmp_path / "output"

    first = _prepare_output_directory(None, base, now)
    second = _prepare_output_directory(None, base, now)

    assert first == base / "run-20261015-123456"
    assert second == base / "run-20261015-123456-2"
    assert first.is_dir()
    assert second.is_dir()


def test_explicit_output_path_is_preserved(tmp_path: Path) -> None:
    requested = tmp_path / "my-model"
    assert _prepare_output_directory(requested) == requested
    assert requested.is_dir()
