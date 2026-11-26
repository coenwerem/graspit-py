# Household-Object Grasp Datasets

This workflow generates new GraspIt! closures on metric household-object meshes,
retains their contacts and sampled quality scores, and writes per-hand/object
NumPy datasets. The same archives can be read by a learning program without
loading GraspIt!. The example covers six YCB objects: sugar box, tomato soup can,
mustard bottle, tuna fish can, banana, and apple. Use the
[dataset guide](https://clintonenwerem.com/graspit-py/datasets.html) for the
coordinate and field definitions.

## Assets and Environment

Build the repository's Docker image as described in the installation guide.
Supply the YCB meshes from the [official dataset](https://www.ycbbenchmarks.com/)
or its [public object-data registry](https://registry.opendata.aws/ycb-benchmarks/).
The example expects one directory per YCB ID, such as `005_tomato_soup_can`.
It accepts `google_16k/textured.obj`, `google_64k/textured.obj`,
`berkeley_processed/textured.obj`, `<YCB-ID>_clean.obj`, or `<YCB-ID>.obj` in that
order. The selected file and its SHA-256 are recorded. These mesh coordinates
must be in meters. Input meshes remain user-supplied; they are not included in
the code repository.

The supplied `barrett.json` profile uses the Barrett model included with the
GraspIt! engine. Its coordinate order is spread, finger 1, finger 2, and finger 3.
The spread coordinate is held at its preshape during closure. `leap.json` is a
planning profile for the exported right LEAP hand documented in the hand-model
guide. Its palm frame, ordered independent coordinates, preshapes, and closing
coordinates must match the supplied robot XML. When a same-stem hand manifest
is present, the generator verifies the coordinate order and retains the joint
map in the dataset metadata.

## Generate a Dataset

Run from the repository root. Replace `/path/to/ycb` with the mesh directory on
the host; `output/household` receives object exports, planner records, and learning
archives. The image contains MuJoCo for object conversion and Xvfb for native
closure.

```sh
mkdir -p output/household
docker run --rm --user "$(id -u):$(id -g)" \
  -v "$PWD/output/household:/output" -v /path/to/ycb:/ycb:ro \
  graspit-py xvfb-run -a python /src/examples/household_objects/generate.py \
    --source-root /ycb \
    --robot /opt/graspit/models/robots/Barrett/Barrett.xml \
    --profile /src/examples/household_objects/barrett.json \
    --hand-id barrett --out /output --top-k 24
```

Use `--objects tomato_soup_can banana` for a smaller run. For LEAP, mount the
exported hand directory, supply its robot XML through `--robot`, and use
`--profile /src/examples/household_objects/leap.json --hand-id leap`.

The generator preserves physical scale and recenters each object at the center
of its source axis-aligned bounding box. The original axes, translation, source
hash, and resulting extents are saved in `object_metadata.json`. It draws 2,048
surface points per object with area-weighted triangle sampling and seed 0.
The 0.1 kg mass permits static model compilation; the examples evaluate contact
geometry and normalized wrench quality. Dynamic lift outcomes are a separate
kind of label.

Each profile evaluates 144 approach/preshape combinations with friction values
0.4, 0.6, and 0.8 and one perturbed closure in addition to nominal closure. The
saved top 24 candidates include their individual samples.

## Use Another Mesh

`prepare.py` also accepts an individual OBJ with an explicit length scale:

```sh
python examples/household_objects/prepare.py \
  --source /assets/mug.obj --object-id mug --scale-m 1 \
  --dataset "My Object Collection" --license "CC-BY-4.0" \
  --out output/objects/mug
```

`--scale-m` is meters per source coordinate unit; use `0.001` for millimeter
coordinates. Run the ordinary planner on the resulting `mug.xml`, then export its
JSON through `python -m graspit_py.dataset`. Supply the already-applied scale to
that export. Preserve the source dataset's actual license and attribution.

## Read Training Batches

The export is a standard NPZ archive plus a JSON sidecar. NumPy can read every
array with `allow_pickle=False`; poses and contact positions use meters, joint
coordinates use radians, and independent-coordinate names retain their order.
Each archive also contains its object-frame surface point cloud. The
`load_torch.py` example uses the validated loader and PyTorch, installed by the
caller:

```sh
PYTHONPATH=$PWD python examples/household_objects/load_torch.py \
  output/household/learning/barrett/ycb_tomato_soup_can.npz
```

A batch contains object points, object-relative hand transforms, named joint
coordinates, and nominal geometric quality. For quality prediction, retain the
individual quality values, including zeros. For grasp-generation targets, choose
and document the geometric acceptance criterion. Split by object identity before
forming training and evaluation sets. Keep hands with different coordinate
counts in distinct batches or define an explicit padded representation and mask.

## Attribution

YCB object geometry and derived surface samples are attributed to Berk Calli and
collaborators, *The YCB Object and Model Set: Towards Common Benchmarks for
Manipulation Research*, 2015, and retain the dataset's CC BY 4.0 terms. Cite the
GraspIt! simulator and the graspit-py companion paper for generated contact and
quality records. Hand-model assets retain their source licenses.
