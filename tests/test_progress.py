from io import StringIO

from rollergear.cli import ProgressReporter


def test_noninteractive_progress_is_throttled_to_ten_percent_buckets() -> None:
    stream = StringIO()
    progress = ProgressReporter(stream)

    progress.update("Applying Boolean cuts", 1, 100)
    progress.update("Applying Boolean cuts", 2, 100)
    progress.update("Applying Boolean cuts", 10, 100)
    progress.update("Applying Boolean cuts", 100, 100)

    assert stream.getvalue().splitlines() == [
        "Applying Boolean cuts: 1% (1/100)",
        "Applying Boolean cuts: 10% (10/100)",
        "Applying Boolean cuts: 100% (100/100)",
    ]
