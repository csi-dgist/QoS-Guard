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

"""Tests for yaml_qos_loader - ② YAML parameters."""

import tempfile
from pathlib import Path

import pytest

from qos_guard import yaml_qos_loader


YAML_CONFIG = """
my_node:
  ros__parameters:
    reliability: reliable
    durability: volatile
    depth: 10
"""


class TestLoadQosFromYaml:
    """② YAML: merge into existing QoS dictionary."""

    def test_loads_qos_from_yaml(self):
        """Extract reliability, durability, depth from config/*.yaml."""
        with tempfile.TemporaryDirectory() as tmp:
            pkg = Path(tmp)
            (pkg / "config").mkdir()
            (pkg / "config" / "params.yaml").write_text(YAML_CONFIG)
            result = yaml_qos_loader.load_qos_from_yaml_files(pkg)
        assert "default" in result
        assert result["default"].get("reliability") == "RELIABLE"
        assert result["default"].get("durability") == "VOLATILE"
        assert result["default"].get("history_depth") == "10"

    def test_merge_yaml_qos_into(self):
        """merge_yaml_qos_into: supplement empty fields with YAML only."""
        qos = {"reliability": "", "durability": "TRANSIENT_LOCAL", "history_depth": "5"}
        yaml_p = {"default": {"reliability": "RELIABLE", "history_depth": "10"}}
        out = yaml_qos_loader.merge_yaml_qos_into(qos, yaml_p)
        assert out["reliability"] == "RELIABLE"
        assert out["durability"] == "TRANSIENT_LOCAL"
        assert out["history_depth"] == "5"
