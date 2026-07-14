# Copyright 2025 QoS-Guard Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for the publish-period auto-extractor (pp_extractor)."""

from pathlib import Path

from qos_guard import pp_extractor


def _min_ms(content: str, name: str = "node.cpp"):
    """Helper: smallest period (ms) extracted from a source snippet."""
    periods = pp_extractor.periods_in_source(Path(name), content)
    return min((ms for ms, _ in periods), default=None)


def test_min_of_two_wall_timers_is_20():
    """create_wall_timer(100ms) + create_wall_timer(20ms) -> min PP = 20."""
    snippet = """
    auto t1 = node->create_wall_timer(100ms, cb1);
    auto t2 = node->create_wall_timer(20ms, cb2);
    """
    assert _min_ms(snippet) == 20


def test_chrono_factory_and_seconds_literal():
    """std::chrono::milliseconds(X) and Xs literals convert correctly."""
    assert _min_ms("node->create_timer(std::chrono::milliseconds(250), cb);") == 250
    assert _min_ms("node->create_wall_timer(1s, cb);") == 1000
    # Sub-ms period stays fractional at the text level; the package API floors it.
    assert _min_ms("node->create_wall_timer(500us, cb);") == 0.5


def test_sub_ms_period_floored_to_one(tmp_path):
    """The package-level API floors a sub-millisecond period to a minimum of 1ms."""
    (tmp_path / "a.cpp").write_text(
        "node->create_wall_timer(500us, cb);", encoding="utf-8"
    )
    pp, _ = pp_extractor.extract_min_publish_period_ms(tmp_path)
    assert pp == 1


def test_cpp_create_timer_with_clock_first_arg():
    """create_timer(clock, <dur>, cb): duration is not the first arg."""
    assert _min_ms("create_timer(this->get_clock(), 40ms, cb);") == 40


def test_cpp_rate_hz_to_period():
    """rclcpp::Rate(hz) -> period = 1000/hz ms."""
    assert _min_ms("rclcpp::Rate loop_rate(20.0);") == 50
    assert _min_ms("rclcpp::WallRate r(10);") == 100


def test_python_create_timer_seconds():
    """rclpy create_timer(period_sec, cb) -> period_sec * 1000 ms."""
    assert _min_ms("self.create_timer(0.1, self.cb)", "node.py") == 100
    assert _min_ms("self.create_timer(timer_period_sec=0.05, callback=cb)", "n.py") == 50


def test_python_create_rate_hz():
    """rclpy create_rate(hz) -> 1000/hz ms."""
    assert _min_ms("rate = self.create_rate(50)", "node.py") == 20


def test_no_timer_returns_none():
    """A snippet without any timer/rate yields no period."""
    assert _min_ms("auto pub = node->create_publisher<Msg>(\"/t\", qos);") is None


def test_extract_min_end_to_end(tmp_path):
    """Directory scan picks the smallest period across multiple files."""
    (tmp_path / "a.cpp").write_text(
        "node->create_wall_timer(100ms, cb);", encoding="utf-8"
    )
    (tmp_path / "b.cpp").write_text(
        "node->create_wall_timer(20ms, cb);", encoding="utf-8"
    )
    (tmp_path / "c.py").write_text(
        "self.create_timer(0.2, self.cb)", encoding="utf-8"
    )
    pp, source = pp_extractor.extract_min_publish_period_ms(tmp_path)
    assert pp == 20
    assert source is not None and "b.cpp" in source


def test_extract_min_none_when_no_timers(tmp_path):
    """Package with no timers returns (None, None)."""
    (tmp_path / "x.cpp").write_text("int main() { return 0; }", encoding="utf-8")
    assert pp_extractor.extract_min_publish_period_ms(tmp_path) == (None, None)


if __name__ == "__main__":
    # Allow running without pytest: execute every test_* function.
    import tempfile

    for _name, _fn in sorted(globals().items()):
        if _name.startswith("test_") and callable(_fn):
            if "tmp_path" in _fn.__code__.co_varnames[: _fn.__code__.co_argcount]:
                with tempfile.TemporaryDirectory() as _d:
                    _fn(Path(_d))
            else:
                _fn()
            print(f"PASS {_name}")
    print("ALL PP-EXTRACTOR TESTS PASSED")
