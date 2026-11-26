"""Generate and export static grasp datasets from existing metric YCB OBJ meshes."""
from __future__ import annotations
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from graspit_py.dataset import export_dataset
from graspit_py.planning import ContactPlanner, HandFrame, PlannerConfig
from prepare import prepare

YCB = {'sugar_box':'004_sugar_box','tomato_soup_can':'005_tomato_soup_can',
       'mustard_bottle':'006_mustard_bottle','tuna_fish_can':'007_tuna_fish_can',
       'banana':'011_banana','apple':'013_apple'}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-root',type=Path,required=True)
    p.add_argument('--robot',type=Path,required=True)
    p.add_argument('--profile',type=Path,required=True)
    p.add_argument('--hand-id',required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--objects',nargs='+',choices=list(YCB),default=list(YCB))
    p.add_argument('--top-k',type=int,default=24)
    a=p.parse_args()
    if a.top_k<=0:
        p.error('--top-k must be positive')
    profile=json.loads(a.profile.read_text())
    config=PlannerConfig(**profile['planner']); hand=HandFrame(**profile['hand'])
    names=profile['dof_names']
    hand_manifest=a.robot.with_suffix('.json')
    joint_map=None
    if hand_manifest.is_file():
        manifest=json.loads(hand_manifest.read_text())
        if manifest.get('dof_joints') != names:
            raise ValueError('Profile DOF order differs from the exported hand manifest')
        joint_map={key:manifest[key]for key in ('dof_joints','chains','links')}
    from graspit_py.core import bindings as gp
    gp.initialize(); world=gp.World(); world.init()
    for name in a.objects:
        object_id='ycb_'+name; source_id=YCB[name]
        base=a.source_root/source_id
        choices=[base/'google_16k/textured.obj',base/'google_64k/textured.obj',
                 base/'berkeley_processed/textured.obj',base/(source_id+'_clean.obj'),base/(source_id+'.obj')]
        source=next((path for path in choices if path.is_file()),None)
        if source is None:
            raise FileNotFoundError(f'No supported OBJ found for {source_id}; tried {choices}')
        objdir=a.out/'objects'/object_id
        prepare(source,objdir,object_id,dataset='YCB Object and Model Set',license_name='CC-BY-4.0',
                source_url='https://www.ycbbenchmarks.com/',
                source_citation='Calli et al., The YCB Object and Model Set, 2015')
        world.clear()
        if not world.load_robot(str(a.robot.resolve())):
            raise RuntimeError(f'Could not load {a.robot}')
        object_xml=objdir/(object_id+'.xml')
        if not world.load_graspable_body(str(object_xml.resolve())):
            raise RuntimeError(f'Could not load {object_xml}')
        if world.get_current_hand_num_dof()!=len(names):
            raise ValueError('Profile DOF names must match this robot')
        start=time.perf_counter()
        planner=ContactPlanner(world,0,config,hand=hand)
        candidates=planner.plan(top_k=a.top_k)
        payload=dict(robot=str(a.robot),object=str(object_xml),hand=asdict(hand),planner=asdict(config),
                     seconds=time.perf_counter()-start,rejections=planner.rejections,
                     units=dict(length='mm',angle='rad',quaternion='wxyz'),
                     object_metadata=json.loads((objdir/'object_metadata.json').read_text()),
                     profile_sha256=hashlib.sha256(a.profile.read_bytes()).hexdigest(),
                     pointcloud_generation=dict(method='area-weighted triangle-surface sampling',points=2048,seed=0,frame='object',length_unit='m'),
                     candidates=[c.to_dict()for c in candidates])
        payload['quality_conventions']=dict(
            contact_model='point contact with friction',friction_cone_edges=8,
            force_budget='unit sum of contact normal forces',
            recomputed_torque_scale='maximum contact distance to object center for each closure, with a 0.01 mm minimum',
            wrench_origin_m=payload['object_metadata']['object_cog_m'],
            task_torque_scale_m=ContactPlanner.TORQUE_SCALE_MM/1000,
            native_epsilon='unmodified native engine material and normalization conventions')
        if joint_map is not None:
            payload['hand_joint_map']=joint_map
        source_json=a.out/'records'/a.hand_id/(object_id+'.json')
        source_json.parent.mkdir(parents=True,exist_ok=True)
        source_json.write_text(json.dumps(payload,indent=1)+'\n')
        export_dataset(payload,a.out/'learning'/a.hand_id/(object_id+'.npz'),hand_id=a.hand_id,
                       object_id=object_id,dof_names=names,model_files={'hand':a.robot,'object':object_xml,'object_mesh':objdir/(object_id+'.off')},
                       object_pointcloud_m=np.load(objdir/'object_pointcloud_m.npy',allow_pickle=False),source_path=source_json)
        print(f'{a.hand_id} {object_id}: {len(candidates)} saved grasps, {payload["seconds"]:.1f} s',flush=True)

if __name__=='__main__': main()
