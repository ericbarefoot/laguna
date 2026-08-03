"""DataProcessor.save_data() used to log "Saved N data points" and return
True while writing nothing — an active hazard for any caller trusting the
return value. It now raises rather than lying about success.
"""

import pytest

from laguna.data import DataProcessor


class TestSaveDataIsHonestlyUnimplemented:
    def test_save_data_raises_rather_than_reporting_false_success(self, tmp_path):
        processor = DataProcessor({"output_directory": str(tmp_path)})
        with pytest.raises(NotImplementedError):
            processor.save_data("out.csv", data=[{"x": 1}])
        assert list(tmp_path.glob("*")) == [], "must not claim success while writing nothing"

    def test_export_data_fails_safely_rather_than_returning_a_path(self, tmp_path):
        processor = DataProcessor({"output_directory": str(tmp_path)})
        processor.data_buffer = [{"x": 1}]
        assert processor.export_data("csv") is None
