"""One unchanged10mm side-push, original or directional community controller."""
from pathlib import Path
import argparse, hashlib, json, time
import mujoco
import contact_tests as shared
from adapter import RightArmImpedance
from directional_adapter import DirectionalRightArmImpedance
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OUT=ROOT/'validation/directional_control'


def run(variant):
    shared.OUT=OUT
    controller=None
    with shared.Episode('sidepush10mm',variant) as case:
        wall=time.monotonic()
        print(variant,'prepare',flush=True)
        try:
            case.prepare()
            cls=RightArmImpedance if variant=='original' else DirectionalRightArmImpedance
            with cls(case.env) as controller:
                case.active=True
                print(variant,'edge touch',flush=True)
                target=case.touch()
                print(variant,'10mm push',flush=True)
                target=case.task_action(target)
                print(variant,'retreat',flush=True)
                case.retreat(target)
                assert controller.calls>0
                case.report['protocol_completed']=True
        except Exception as exc:
            case.report['error']=repr(exc)
            case.report['stopped_state']=case.state()
            try:case.image('stopped')
            except Exception:pass
        case.report['wall_seconds']=time.monotonic()-wall
        case.report['experiment_source_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [HERE/'directional_test.py',HERE/'directional_adapter.py',HERE/'adapter.py',HERE/'contact_tests.py',HERE/'side_push_fixture.json']}
        result=case.save(controller)
    assert mujoco.get_mjcb_control() is None
    print(json.dumps({k:result.get(k) for k in ['controller','protocol_completed','error','blank_translation_mm','hand_action_delta_mm','translation_goal_achieved','peak_sum_normal_N','released']},ensure_ascii=False),flush=True)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variant',choices=['original','directional'],required=True)
    args=parser.parse_args()
    manifest=json.loads((HERE/'UPSTREAM.json').read_text())
    for item in manifest['files'].values():assert hashlib.sha256((HERE/item['local']).read_bytes()).hexdigest()==item['sha256']
    fixture=manifest['side_push_fixture'];assert hashlib.sha256((HERE/fixture['file']).read_bytes()).hexdigest()==fixture['sha256']
    run(args.variant)
