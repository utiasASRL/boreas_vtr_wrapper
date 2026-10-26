"""Checks optix_range_tracer_multi against optix_range_tracer on a real submap mesh and radar scan.

The two modules can't be loaded in one process: both link the OptiX SDK's sutil library under the same
name, so the second module would reuse the first one's copy and load the wrong device code. The
reference trace therefore runs in a subprocess (--reference).

    python3 native/optix_range_tracer_multi/test_multi_channel.py   (inside the container)
"""
import os
import sys
from pathlib import Path

import numpy as np
import torch
from pyboreas import BoreasDataset
from pyboreas.utils.odometry import interpolate_poses
from pyboreas.utils.utils import get_inverse_tf

ROOT = Path(os.environ["VTRROOT"])
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "build" / "optix_range_tracer_multi" / "python"))
REFERENCE = "--reference" in sys.argv
if REFERENCE:
    import optix_range_tracer  # noqa: E402
else:
    import optix_range_tracer_multi  # noqa: E402

from postprocessing.mesh_to_depth_image import (  # noqa: E402
    build_lidar_to_robot_transform, get_submap_vertices, load_submap_mesh_to_enu,
)

SEQ = "boreas-2024-12-03-12-54"
device = torch.device("cuda:0")
seq = BoreasDataset(os.environ["VTRRDATA"], [[SEQ]]).get_seq_from_ID(SEQ)
_, submaps = get_submap_vertices(str(Path(os.environ.get("VTRRESULT", ROOT / "results")) / "lidar" / SEQ / SEQ / "graph"))
submap = submaps[40]
lidar = next(f for f in seq.lidar_frames if int(f.frame) == submap.stamp // 1000)
verts, tris, _ = load_submap_mesh_to_enu(ROOT / "postprocessing" / "submap_meshes", SEQ, submap,
                                         build_lidar_to_robot_transform(seq), lidar.pose, device)
idx = next(i for i, f in enumerate(seq.radar_frames) if int(f.frame) >= submap.stamp // 1000) + 1
radar = seq.get_radar(idx)
frames = seq.radar_frames[idx - 1:idx + 2]
poses = np.asarray(interpolate_poses([get_inverse_tf(f.pose) for f in frames], [f.timestamp_micro for f in frames],
                                     radar.timestamps.flatten().tolist()))
odom = torch.as_tensor(poses, device=device, dtype=torch.float32).contiguous()
az = torch.as_tensor(np.asarray(radar.azimuths).reshape(-1), device=device, dtype=torch.float32).contiguous()
eye = torch.eye(4, device=device)
kw = dict(width=61, height=61, theta_min=-3.0, theta_max=3.0, phi_min=-3.0, phi_max=3.0, min_range=0.0)


def tracer(module, **channels):
    t = module.OptixRangeTracer(**kw)
    if channels:
        t.set_channels(**channels)
    t.set_mesh(verts, tris)
    t.set_scan(odom, az)
    return t


def timed(t, n=20):
    t.trace(eye)
    torch.cuda.synchronize()
    s, e = torch.cuda.Event(enable_timing=True), torch.cuda.Event(enable_timing=True)
    s.record()
    for _ in range(n):
        t.trace(eye)
    e.record()
    torch.cuda.synchronize()
    return s.elapsed_time(e) / n


REF_PATH = Path("/tmp/optix_multi_reference.pt")
if REFERENCE:
    ref_tracer = tracer(optix_range_tracer)
    torch.save({"ref": ref_tracer.trace(eye).cpu(), "ms": timed(ref_tracer)}, REF_PATH)
    sys.exit(0)
import subprocess  # noqa: E402

subprocess.run([sys.executable, __file__, "--reference"], check=True, capture_output=True)
saved = torch.load(REF_PATH)
ref = saved["ref"].to(device)
hit = ref > 0
print(f"{SEQ} submap {submap.stamp // 1000}, radar frame {radar.frame}: {hit.float().mean():.1%} of rays hit, "
      f"original {saved['ms']:.2f} ms/trace")

single = tracer(optix_range_tracer_multi, mode="single").trace(eye)
assert single.shape == ref.shape and torch.equal(single, ref), "single mode differs from optix_range_tracer"
print("  single: identical to optix_range_tracer")

max_range, k = 100.0, 3
bt = tracer(optix_range_tracer_multi, mode="bands", count=k, max_range=max_range)
bands = bt.trace(eye)
assert bands.shape == (ref.shape[0], k, 61, 61)
edges = [max_range * i / k for i in range(k + 1)]
for b in range(k):
    v = bands[:, b]
    assert ((v == 0) | ((v >= edges[b]) & (v <= edges[b + 1]))).all(), f"band {b} has hits outside its range"
    in_band = hit & (ref >= edges[b]) & (ref <= edges[b + 1])
    assert torch.equal(v[in_band], ref[in_band]), f"band {b} misses the first hit inside it"
print(f"  bands x{k} to {max_range:g} m: every hit inside its band, first hits reproduced; hit fraction per band "
      + ", ".join(f"{(bands[:, b] > 0).float().mean():.1%}" for b in range(k)) + f"; {timed(bt):.2f} ms/trace")

skip = 5.0
lt = tracer(optix_range_tracer_multi, mode="layers", count=k, layer_skip=skip)
layers = lt.trace(eye)
assert torch.equal(layers[:, 0], ref), "layer 0 differs from the first hit"
for j in range(1, k):
    prev, cur = layers[:, j - 1], layers[:, j]
    assert (cur[cur > 0] >= prev[cur > 0] + skip - 1e-3).all(), f"layer {j} closer than skip"
    assert (cur[prev == 0] == 0).all(), f"layer {j} hit after a miss"
print(f"  layers x{k}, skip {skip:g} m: layer 0 identical, each later layer >= {skip:g} m past the previous; "
      "hit fraction per layer " + ", ".join(f"{(layers[:, j] > 0).float().mean():.1%}" for j in range(k))
      + f"; {timed(lt):.2f} ms/trace")
gap = layers[:, 1] - layers[:, 0]
gap = gap[layers[:, 1] > 0]
print(f"  layer 1 - layer 0 where layer 1 hits: median {gap.median():.1f} m, "
      f"{(gap < skip + 1).float().mean():.0%} within {skip + 1:g} m of the first hit")
print("all checks passed")
