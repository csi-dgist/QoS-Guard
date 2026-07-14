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

"""Tests for qos_profile_resolver - 3-stage generalized scanning."""

import tempfile
from pathlib import Path

import pytest

from qos_guard import code_scanner
from qos_guard import qos_profile_resolver


LATCHED_PUB_HPP = '''
#ifndef LATCHED_QOS_HPP
#define LATCHED_QOS_HPP
#include "rclcpp/rclcpp.hpp"

class LatchedPublisherQoS : public rclcpp::QoS
{
public:
  explicit LatchedPublisherQoS(const int depth = 1)
  : rclcpp::QoS(rclcpp::KeepLast(depth))
  {
    this->reliable();
    this->transient_local();
  }
};
#endif
'''

NAMESPACE_QOS_HPP = '''
namespace my_project {
class QoS : public rclcpp::QoS {
public:
  explicit QoS(int depth = 5) : rclcpp::QoS(rclcpp::KeepLast(depth)) {
    this->reliable();
    this->durability_volatile();
  }
};
}
'''

PY_GET_QOS = '''
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

def get_latched_qos():
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        depth=1
    )
'''


class TestBuildQosDictionary:
    """Stage 1+2: Global Symbol Collection and Internal Logic Parsing."""

    def test_extracts_cpp_class_qos(self):
        """Parse C++ class (class X : public rclcpp::QoS)."""
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp) / "qos.hpp").write_text(LATCHED_PUB_HPP)
            d = qos_profile_resolver.build_qos_dictionary(Path(tmp))
        assert "LatchedPublisherQoS" in d
        assert d["LatchedPublisherQoS"]["reliability"] == "RELIABLE"
        assert d["LatchedPublisherQoS"]["durability"] == "TRANSIENT_LOCAL"
        assert d["LatchedPublisherQoS"]["history_depth"] == "1"

    def test_extracts_namespace_class(self):
        """Register namespace my_project { class QoS } -> my_project::QoS."""
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp) / "utils.hpp").write_text(NAMESPACE_QOS_HPP)
            d = qos_profile_resolver.build_qos_dictionary(Path(tmp))
        assert "QoS" in d
        assert "my_project::QoS" in d
        assert d["my_project::QoS"]["history_depth"] == "5"

    def test_extracts_py_function_qos(self):
        """Parse Python function (def get_latched_qos)."""
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp) / "qos_utils.py").write_text(PY_GET_QOS)
            d = qos_profile_resolver.build_qos_dictionary(Path(tmp))
        assert "get_latched_qos" in d
        assert d["get_latched_qos"]["reliability"] == "RELIABLE"
        assert d["get_latched_qos"]["durability"] == "TRANSIENT_LOCAL"
        assert d["get_latched_qos"]["history_depth"] == "1"


class TestResolveQosFromExpression:
    """Stage 3: Caller Mapping."""

    def test_resolves_nav2_style(self):
        """nav2::qos::LatchedPublisherQoS() -> dictionary lookup."""
        d = {
            "LatchedPublisherQoS": {
                "reliability": "RELIABLE",
                "durability": "TRANSIENT_LOCAL",
                "history_depth": "1",
                "history": "KEEP_LAST",
            }
        }
        r = qos_profile_resolver.resolve_qos_from_expression(
            "nav2::qos::LatchedPublisherQoS()", d
        )
        assert r is not None
        assert r["reliability"] == "RELIABLE"
        assert r["history_depth"] == "1"

    def test_resolves_with_depth_arg(self):
        """LatchedPublisherQoS(5) -> history_depth=5 override."""
        d = {
            "LatchedPublisherQoS": {
                "reliability": "RELIABLE",
                "durability": "TRANSIENT_LOCAL",
                "history_depth": "1",
                "history": "KEEP_LAST",
            }
        }
        r = qos_profile_resolver.resolve_qos_from_expression(
            "LatchedPublisherQoS(5)", d
        )
        assert r is not None
        assert r["history_depth"] == "5"

    def test_resolves_variable(self):
        """qos_var -> dictionary lookup."""
        d = {
            "qos_var": {
                "reliability": "BEST_EFFORT",
                "durability": "VOLATILE",
                "history_depth": "10",
                "history": "KEEP_LAST",
            }
        }
        r = qos_profile_resolver.resolve_qos_from_expression("qos_var", d)
        assert r is not None
        assert r["reliability"] == "BEST_EFFORT"

    def test_returns_none_unknown(self):
        """Unknown symbol -> None."""
        d = {}
        assert qos_profile_resolver.resolve_qos_from_expression("UnknownQoS()", d) is None

    def test_resolves_full_namespace(self):
        """my_project::QoS() - full name capture and lookup."""
        d = {
            "my_project::QoS": {
                "reliability": "RELIABLE",
                "durability": "VOLATILE",
                "history_depth": "10",
                "history": "KEEP_LAST",
            }
        }
        r = qos_profile_resolver.resolve_qos_from_expression(
            "my_project::QoS()", d
        )
        assert r is not None
        assert r["reliability"] == "RELIABLE"

    def test_resolves_short_fallback(self):
        """nav2::qos::LatchedPublisherQoS() - fallback to short if no full name."""
        d = {"LatchedPublisherQoS": {"reliability": "RELIABLE", "durability": "TRANSIENT_LOCAL", "history_depth": "1", "history": "KEEP_LAST"}}
        r = qos_profile_resolver.resolve_qos_from_expression(
            "nav2::qos::LatchedPublisherQoS()", d
        )
        assert r is not None
        assert r["reliability"] == "RELIABLE"


@pytest.mark.functional
class TestScanCodeIntegration:
    """Stage 3 full pipeline: header definition + cpp usage -> scan_code extracts correct QoS."""

    def test_scan_code_uses_qos_dictionary(self):
        """If package has .hpp(definition) + .cpp(usage), scan_code looks up in dictionary."""
        with tempfile.TemporaryDirectory() as tmp:
            pkg = Path(tmp)
            (pkg / "include" / "qos").mkdir(parents=True)
            (pkg / "include" / "qos" / "profiles.hpp").write_text(LATCHED_PUB_HPP)
            (pkg / "src").mkdir()
            (pkg / "src" / "node.cpp").write_text('''
#include "qos/profiles.hpp"
int main() {
  auto node = rclcpp::Node::make_shared("test");
  auto pub = node->create_publisher<vision_msgs::msg::MarkerArray>(
      "route_graph", nav2::qos::LatchedPublisherQoS());
  return 0;
}
''')
            entities = code_scanner.scan_code(pkg)
        route_pubs = [e for e in entities if "route_graph" in e.topic_name]
        assert len(route_pubs) >= 1
        qos = route_pubs[0].qos_from_code
        assert qos.get("reliability") == "RELIABLE"
        assert qos.get("durability") == "TRANSIENT_LOCAL"
        assert qos.get("history_depth") == "1"

    def test_scan_code_python_qos_function(self):
        """Python: def get_qos() + create_publisher(..., qos_profile=get_qos()) -> dictionary lookup."""
        with tempfile.TemporaryDirectory() as tmp:
            pkg = Path(tmp)
            (pkg / "node.py").write_text(PY_GET_QOS + '''
def main():
    node = None  # mock
    node.create_publisher(String, "/map", qos_profile=get_latched_qos())
''')
            entities = code_scanner.scan_code(pkg)
        map_pubs = [e for e in entities if "/map" in (e.topic_name or "")]
        assert len(map_pubs) >= 1
        qos = map_pubs[0].qos_from_code
        assert qos.get("reliability") == "RELIABLE"
        assert qos.get("durability") == "TRANSIENT_LOCAL"
        assert qos.get("history_depth") == "1"
