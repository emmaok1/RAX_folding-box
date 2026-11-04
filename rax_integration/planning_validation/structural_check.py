"""Scratch static-geometry collision audit; no mj_step, task state or masks changed."""
from pathlib import Path
import json,xml.etree.ElementTree as ET
import mujoco
from rax_integration.planning_validation.fixture import StaticRobotFixture
from rax_integration.motion_planning.model import ROOT,StaticRobotModel,robot_snapshot
from rax_integration.motion_planning.collision import CollisionChecker

def main():
    env=StaticRobotFixture();r=StaticRobotModel(env.fixture_xml,robot_snapshot(env));m,d=r.m,r.d;c=CollisionChecker(r,'right',.005)
    names=['dexmate_R_gripper_base_collision_115','dexmate_R_gripper_l1_collision_118'];ids=[m.geom(n).id for n in names]
    raw=float(mujoco.mj_geomDistance(m,d,*ids,.1,None));b=[int(m.geom_bodyid[g]) for g in ids]
    assert int(m.body_parentid[b[1]])==b[0]
    pair=lambda a,b:frozenset([int(a),int(b)])
    def matching_contacts():
        mujoco.mj_collision(m,d)
        return [float(x.dist) for x in d.contact if pair(*x.geom)==pair(*ids)]
    original_flags=int(m.opt.disableflags);normal=matching_contacts()
    # Toggle native filtering ONLY on the private diagnostic model; never live XML.
    m.opt.disableflags=original_flags|int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT)
    unfiltered=matching_contacts();m.opt.disableflags=original_flags
    assert not normal and unfiltered
    parent_exclusions=[x for x in c.excluded if x['reason'].startswith('native adjacent')]
    assert all(all(n.startswith('dexmate_') for n in x['bodies']) for x in parent_exclusions)
    exact_parent_pairs=[]
    for x in parent_exclusions:
        bodyids=[m.body(n).id for n in x['bodies']]
        geoms=[[g for g in range(m.ngeom) if m.geom_bodyid[g]==bid and (m.geom_contype[g] or m.geom_conaffinity[g])] for bid in bodyids]
        for a in geoms[0]:
            for b in geoms[1]:
                item={'geoms':[m.geom(a).name,m.geom(b).name],**x}
                if item not in exact_parent_pairs:exact_parent_pairs.append(item)
    checks={
      'pad_table_checked':any('jaw_pad' in m.geom(a).name+m.geom(b).name and 'table_collision' in [m.geom(a).name,m.geom(b).name] for a,b in c.pairs),
      'pad_opposite_arm_checked':any('jaw_pad_R_' in m.geom(a).name+m.geom(b).name and ('dexmate_L_' in m.geom(a).name+m.geom(b).name or 'jaw_pad_L_' in m.geom(a).name+m.geom(b).name) for a,b in c.pairs),
      'opposing_right_jaws_checked':pair(m.geom('jaw_pad_R_1').id,m.geom('jaw_pad_R_2').id) in {pair(a,b) for a,b in c.pairs}}
    assert all(checks.values()),checks
    srdf=ET.parse(ROOT/'models/dexmate/source/robots/humanoid/vega_1u/vega_1u.srdf').getroot()
    allowed=[e.attrib for e in srdf.findall('disable_collisions') if e.get('reason')=='Always']
    exact=[]
    for a,b in c.pairs:
        if a in ids or b in ids:exact.append([m.geom(a).name,m.geom(b).name])
    report={'scope':'static scratch collision queries only; zero physics steps; no cardboard oracle',
      'probe_pair':names,'raw_convex_distance_m':raw,'native_default_contacts':normal,'native_filterparent_disabled_contacts':unfiltered,
      'native_flags_restored':int(m.opt.disableflags)==original_flags,
      'direct_parent_exclusions':exact_parent_pairs,'source_srdf_always_entries':allowed,
      'srdf_never_default_not_imported':True,'coverage_checks':checks,'retained_pairs_involving_probe_geoms':exact,
      'mesh_collision':'MuJoCo convex mesh representation, not concave visual surfaces',
      'native_source':'https://raw.githubusercontent.com/google-deepmind/mujoco/3.4.0/src/engine/engine_collision_driver.c',
      'cardboard_coverage':'unknown; production planner refuses, no cardboard state queried',
      'relevant_pair_count':len(c.pairs),'all_checks_passed':True}
    out=ROOT/'rax_integration/planning_validation/evidence/structural_filter_check.json';out.write_text(json.dumps(report,indent=2));print(out);print(json.dumps({k:report[k] for k in ['raw_convex_distance_m','native_default_contacts','native_filterparent_disabled_contacts','coverage_checks','relevant_pair_count','all_checks_passed']},indent=2));env.close()

if __name__=='__main__':main()
