import csv
import tempfile
import unittest
from pathlib import Path

from mg400_controller.common.utils.loggers.async_event_logger import AsyncEventLogger


class AsyncEventLoggerTest(unittest.TestCase):
    def test_writes_csv_rows_and_dispatches_named_handlers(self):
        handled = []

        with tempfile.TemporaryDirectory() as tmp_dir:
            csv_path = Path(tmp_dir) / "events.csv"
            logger = AsyncEventLogger(
                csv_path=str(csv_path),
                csv_header=["kind", "value"],
                handlers={
                    "CUSTOM": lambda *payload: handled.append(payload),
                },
            )

            logger.put("CSV", ["sample", 123])
            logger.put("CUSTOM", ["alpha", "beta"])
            logger.close(timeout=2.0)

            with csv_path.open(newline="") as csv_file:
                rows = list(csv.reader(csv_file))

        self.assertEqual(rows, [["kind", "value"], ["sample", "123"]])
        self.assertEqual(handled, [("alpha", "beta")])


if __name__ == "__main__":
    unittest.main()
