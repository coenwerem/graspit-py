"""Read one hand/object archive as PyTorch grasp-training batches on the CPU.

Run from the repository checkout; install PyTorch separately for this example.
This example exercises data loading and tensor conversion, not policy training.
"""
import argparse
import numpy as np
import torch
from torch.utils.data import DataLoader, Dataset
from graspit_py.dataset import load_dataset

class StaticGrasps(Dataset):
    def __init__(self,path):
        self.data=load_dataset(path).arrays
        self.points=torch.from_numpy(np.ascontiguousarray(self.data['object_pointcloud_m']))
    def __len__(self): return len(self.data['grasp_ids'])
    def __getitem__(self,index):
        return {'object_points_m':self.points,
                'T_object_hand_m':torch.from_numpy(self.data['T_object_hand_m'][index]),
                'dofs_rad':torch.from_numpy(self.data['dofs_rad'][index]),
                'epsilon':torch.tensor(self.data['nominal_epsilon'][index],dtype=torch.float64)}

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('dataset');a=p.parse_args()
    dataset=StaticGrasps(a.dataset)
    batch=next(iter(DataLoader(dataset,batch_size=8,shuffle=False,num_workers=0)))
    for field,value in batch.items(): print(field,tuple(value.shape),str(value.dtype))
