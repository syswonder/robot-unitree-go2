# SPDX-License-Identifier: Apache-2.0
"""Data-driven identities and startup parameters for seven skills in one package."""
import json
from pathlib import Path

DEFINITIONS = tuple(json.loads(Path(__file__).with_name("skills.json").read_text()))
BY_ACTION = {item["action"]: item for item in DEFINITIONS}
BY_PROVIDER = {item["provider_id"]: item for item in DEFINITIONS}
RUNTIME_ID = "unitree_go2_sport_runtime"
RUNTIME_NAMESPACE = "robonix/service/unitree_go2_sport_runtime"
