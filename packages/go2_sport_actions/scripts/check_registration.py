# SPDX-License-Identifier: Apache-2.0
"""Read Atlas only. Never activate providers or call motion endpoints."""
from robonix_api import ATLAS
from go2_sport_actions.skills import DEFINITIONS, RUNTIME_ID

for definition in DEFINITIONS:
    rows = ATLAS.query_skills(id=definition["provider_id"])
    assert len(rows) == 1, definition
    contracts = {c.contract_id for c in rows[0].capabilities}
    for leaf in ("execute", "status", "cancel"):
        assert f'{definition["namespace"]}/{leaf}' in contracts, contracts
    assert not any(c.endswith("/execute_utterance") for c in contracts)
    print(definition["provider_id"], "registered: PASS")
assert len(ATLAS.query_services(id=RUNTIME_ID)) == 1
assert not ATLAS.find_capability(provider_id=RUNTIME_ID, transport="mcp")
assert len(ATLAS.find_capability(provider_id=RUNTIME_ID, transport="grpc")) == 4
print("seven Skills + shared Service: PASS (read-only)")
