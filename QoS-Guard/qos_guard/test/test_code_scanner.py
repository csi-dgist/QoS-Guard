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

"""Functional tests for code_scanner module."""

import pytest
from pathlib import Path

from qos_guard import code_scanner
from qos_guard import qos_profile_resolver


def _qos_test_pkg_path() -> Path:
    """Return path to qos_test_pkg."""
    return Path(__file__).resolve().parent.parent.parent / "qos_test_pkg"


# ────────── C++ fixture snippets ──────────

CPP_CMD_VEL_PUB = '''
  rclcpp::QoS qos(10);
  qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  qos.durability(rclcpp::DurabilityPolicy::Volatile);
  auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos);
'''

CPP_CHAINED_QOS = '''
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
      .reliability(rclcpp::ReliabilityPolicy::Reliable)
      .durability(rclcpp::DurabilityPolicy::Volatile);
  auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", qos, [](const auto&) {});
'''

CPP_RELIABLE_TRANSIENT = '''
  rclcpp::QoS qos(10);
  qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
  node->create_publisher<sensor_msgs::msg::LaserScan>("/sensor_data", qos);
'''

CPP_LATCHED_PUBLISHER_QOS = '''
  graph_vis_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "route_graph", nav2::qos::LatchedPublisherQoS());
'''

CPP_QOS_CTOR_SIMPLE = '''
  create_publisher<Msg>("/topic", rclcpp::QoS(10));
'''

# ① Composition: SensorDataQoS().keep_last(10).reliable()
CPP_COMPOSITION = '''
  auto qos = rclcpp::SensorDataQoS().keep_last(10).reliable();
  create_publisher<Msg>("/sensor", qos);
'''

# ③ Member variables: qos_.reliable(), qos_.keep_last(10) - .reliable() directly
CPP_MEMBER_RELIABLE = '''
  void init() { qos_.reliable(); qos_.keep_last(10); }
  void run() { create_publisher<Msg>("/topic", qos_); }
'''


# ────────── Python fixture snippets ──────────

PY_QOS_PROFILE = '''
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)
node.create_publisher(String, "/my_topic", qos)
'''

PY_KEYWORD_ARGS = '''
node.create_publisher(topic="/cmd_vel", qos_profile=qos, msg_type=String)
'''

PY_KEYWORD_TOPIC_ONLY = '''
node.create_publisher(topic="/status", msg_type=String)
'''

PY_POSITIONAL_2ARGS = '''
node.create_publisher(String, "/map")
'''


@pytest.mark.functional
class TestCppScanner:
    """Tests for C++ QoS extraction."""

    def test_extract_cmd_vel_bev(self):
        """Extract BestEffort + Volatile from cmd_vel style."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        cpp = pkg / "src" / "cmd_vel_pub.cpp"
        if not cpp.exists():
            pytest.skip("cmd_vel_pub.cpp not found")
        content = cpp.read_text()
        entities = code_scanner.scan_cpp_file(cpp, content)
        assert len(entities) >= 1
        e = entities[0]
        assert e.entity_type == "pub"
        assert "/cmd_vel" in e.topic_name
        assert e.qos_from_code.get("reliability") == "BEST_EFFORT"
        assert e.qos_from_code.get("durability") == "VOLATILE"
        assert e.qos_from_code.get("history_depth") == "10"

    def test_extract_chained_qos_reliable_volatile(self):
        """Extract Reliable + Volatile from chained QoS (cmd_vel_sub style)."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        cpp = pkg / "src" / "cmd_vel_sub.cpp"
        if not cpp.exists():
            pytest.skip("cmd_vel_sub.cpp not found")
        content = cpp.read_text()
        entities = code_scanner.scan_cpp_file(cpp, content)
        assert len(entities) >= 1
        e = entities[0]
        assert e.entity_type == "sub"
        assert e.qos_from_code.get("reliability") == "RELIABLE"
        assert e.qos_from_code.get("durability") == "VOLATILE"
        assert e.qos_from_code.get("history_depth") == "10"

    def test_extract_sensor_data_reliable_transient_local(self):
        """Extract Reliable + TransientLocal from sensor_pub."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        cpp = pkg / "src" / "sensor_pub.cpp"
        if not cpp.exists():
            pytest.skip("sensor_pub.cpp not found")
        content = cpp.read_text()
        entities = code_scanner.scan_cpp_file(cpp, content)
        assert len(entities) >= 1
        e = entities[0]
        assert e.qos_from_code.get("reliability") == "RELIABLE"
        assert e.qos_from_code.get("durability") == "TRANSIENT_LOCAL"

    def test_extract_status_best_effort_volatile(self):
        """Extract BestEffort + Volatile from status_pub."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        cpp = pkg / "src" / "status_pub.cpp"
        if not cpp.exists():
            pytest.skip("status_pub.cpp not found")
        content = cpp.read_text()
        entities = code_scanner.scan_cpp_file(cpp, content)
        assert len(entities) >= 1
        e = entities[0]
        assert e.entity_type == "pub"
        assert e.qos_from_code.get("reliability") == "BEST_EFFORT"
        assert e.qos_from_code.get("durability") == "VOLATILE"

    def test_extract_member_reliable(self):
        """③ Member variables: search qos_.reliable(), qos_.keep_last(10)."""
        entities = code_scanner.scan_cpp_file(
            Path("/fake/node.cpp"),
            CPP_MEMBER_RELIABLE,
        )
        assert len(entities) >= 1
        e = entities[0]
        assert e.qos_from_code.get("reliability") == "RELIABLE"
        assert e.qos_from_code.get("history_depth") == "10"

    def test_extract_composition_sensor_data_qos(self):
        """① Composition: SensorDataQoS().keep_last(10).reliable() -> RELIABLE, depth=10."""
        qos_dict = qos_profile_resolver.get_builtin_rclcpp_profiles()
        entities = code_scanner.scan_cpp_file(
            Path("/fake/node.cpp"),
            CPP_COMPOSITION,
            qos_dict=qos_dict,
        )
        assert len(entities) >= 1
        e = entities[0]
        assert e.qos_from_code.get("reliability") == "RELIABLE"
        assert e.qos_from_code.get("history_depth") == "10"

    def test_extract_rclcpp_qos_ctor_simple(self):
        """rclcpp::QoS(10) - direct number in constructor, depth=10."""
        entities = code_scanner.scan_cpp_file(
            Path("/fake/main.cpp"),
            CPP_QOS_CTOR_SIMPLE,
        )
        assert len(entities) >= 1
        e = entities[0]
        assert e.qos_from_code.get("history_depth") == "10"

    def test_extract_nav2_latched_publisher_qos(self):
        """Stage 3: Lookup LatchedPublisherQoS from Custom QoS Dictionary."""
        qos_dict = {
            "LatchedPublisherQoS": {
                "reliability": "RELIABLE",
                "durability": "TRANSIENT_LOCAL",
                "history_depth": "1",
                "history": "KEEP_LAST",
            }
        }
        entities = code_scanner.scan_cpp_file(
            Path("/fake/route_server.cpp"),
            CPP_LATCHED_PUBLISHER_QOS,
            qos_dict=qos_dict,
        )
        assert len(entities) >= 1
        e = entities[0]
        assert e.entity_type == "pub"
        assert "route_graph" in e.topic_name
        assert e.qos_from_code.get("reliability") == "RELIABLE"
        assert e.qos_from_code.get("durability") == "TRANSIENT_LOCAL"
        assert e.qos_from_code.get("history_depth") == "1"


@pytest.mark.functional
class TestScanCode:
    """Tests for full scan_code."""

    def test_returns_code_entities(self):
        """scan_code returns list of CodeEntity."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        entities = code_scanner.scan_code(pkg)
        assert isinstance(entities, list)
        for e in entities:
            assert hasattr(e, "entity_type")
            assert hasattr(e, "topic_name")
            assert hasattr(e, "qos_from_code")
            assert e.entity_type in ("pub", "sub")

    def test_finds_eight_entities_in_qos_test_pkg(self):
        """qos_test_pkg has 8 source files with pub/sub (4 pub + 4 sub)."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        entities = code_scanner.scan_code(pkg)
        assert len(entities) == 8

    def test_topics_covered(self):
        """Should cover /cmd_vel, /sensor_data, /map, /status."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        entities = code_scanner.scan_code(pkg)
        topics = {e.topic_name for e in entities}
        assert "/cmd_vel" in topics
        assert "/sensor_data" in topics
        assert "/map" in topics
        assert "/status" in topics


class TestPyScanner:
    """Tests for Python (rclpy) QoS extraction - positional/keyword args."""

    def test_positional_3args_extracts_topic_and_qos(self):
        """create_publisher(Msg, topic, qos_profile) - positional 3 args."""
        entities = code_scanner.scan_py_file(
            Path("/fake/path.py"),
            PY_QOS_PROFILE,
        )
        assert len(entities) >= 1
        e = entities[0]
        assert e.entity_type == "pub"
        assert "/my_topic" in e.topic_name
        assert e.qos_from_code.get("reliability") == "RELIABLE"
        assert e.qos_from_code.get("durability") == "VOLATILE"
        assert e.qos_from_code.get("history_depth") == "10"

    def test_keyword_args_extracts_topic_and_qos(self):
        """create_publisher(topic="/x", qos_profile=qos) - keyword args."""
        content = '''
from rclpy.qos import QoSProfile, ReliabilityPolicy
qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=5)
node.create_publisher(topic="/cmd_vel", qos_profile=qos, msg_type=String)
'''
        entities = code_scanner.scan_py_file(Path("/fake/path.py"), content)
        assert len(entities) >= 1
        e = entities[0]
        assert e.entity_type == "pub"
        assert e.topic_name == "/cmd_vel"
        assert e.qos_from_code.get("reliability") == "RELIABLE"
        assert e.qos_from_code.get("history_depth") == "5"

    def test_keyword_topic_only(self):
        """create_publisher(topic="/status", msg_type=String) - topic only."""
        entities = code_scanner.scan_py_file(
            Path("/fake/path.py"),
            PY_KEYWORD_TOPIC_ONLY,
        )
        assert len(entities) >= 1
        e = entities[0]
        assert e.entity_type == "pub"
        assert e.topic_name == "/status"

    def test_positional_2args_no_qos(self):
        """create_publisher(Msg, topic) - 2 args, default qos."""
        entities = code_scanner.scan_py_file(
            Path("/fake/path.py"),
            PY_POSITIONAL_2ARGS,
        )
        assert len(entities) >= 1
        e = entities[0]
        assert e.entity_type == "pub"
        assert "/map" in e.topic_name


@pytest.mark.functional
class TestFindSourceFiles:
    """Tests for find_source_files."""

    def test_finds_cpp_files(self):
        """Should find .cpp files in src/."""
        pkg = _qos_test_pkg_path()
        if not pkg.exists():
            pytest.skip("qos_test_pkg not found")
        files = code_scanner.find_source_files(pkg)
        cpp_files = [f for f in files if f.suffix == ".cpp"]
        assert len(cpp_files) >= 4
