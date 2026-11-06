import json
from pathlib import Path
def main(ctx):
    world = json.loads(Path("runtime/world/manifest.json").read_text())
    result = ctx.call("dexmate_teacher_state")
    state = json.loads(next(b["text"] for b in result["content"] if b["type"] == "text"))
    assert state["environment_id"] == world["environment_id"]
    assert state["step_id"] == 0
    assert len(state["panels"]) == 14
    mem = ctx.call("dexmate_teacher_memory")
    memory = json.loads(next(b["text"] for b in mem["content"] if b["type"] == "text"))
    assert memory["progress"]["notes"]["next_step"] == "validate-grasp"
    assert not state["physical_progress"]["succeed"]
    Path("output/teacher_contract.json").write_text(json.dumps({"environment_id":world["environment_id"],"step_id":state["step_id"],"panels":len(state["panels"]),"physical_success":False}))
