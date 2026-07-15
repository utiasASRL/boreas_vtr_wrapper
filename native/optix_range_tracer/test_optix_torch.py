import torch

import optix_range_tracer


def expect_failure(label, expected, function):
    try:
        function()
    except (RuntimeError, ValueError) as error:
        assert expected in str(error), f"{label}: unexpected error: {error}"
        print(f"validation passed: {label}")
        return
    raise AssertionError(f"{label}: expected failure")


device = torch.device("cuda:0")
identity = torch.eye(4, dtype=torch.float32, device=device)

vertices = torch.tensor(
    [
        [10.0, -0.3, -0.3],
        [10.0, 0.3, -0.3],
        [10.0, 0.3, 0.3],
        [10.0, -0.3, 0.3],
    ],
    dtype=torch.float32,
    device=device,
)
triangles = torch.tensor([[0, 1, 2], [0, 2, 3]], dtype=torch.int32, device=device)
odometry = identity.expand(400, -1, -1).clone()
azimuths = torch.zeros(400, dtype=torch.float32, device=device)

uninitialized = optix_range_tracer.OptixRangeTracer()
expect_failure("trace before set_mesh", "set_mesh()", lambda: uninitialized.trace(identity))
uninitialized.set_mesh(vertices, triangles)
expect_failure("trace before set_scan", "set_scan()", lambda: uninitialized.trace(identity))

tracer = optix_range_tracer.OptixRangeTracer(
    width=61,
    height=61,
    theta_min=-3.0,
    theta_max=3.0,
    phi_min=-3.0,
    phi_max=3.0,
)

expect_failure(
    "int64 triangles",
    "triangles must have dtype Int",
    lambda: tracer.set_mesh(vertices, triangles.to(torch.int64)),
)
expect_failure(
    "CPU mesh",
    "vertices must be a CUDA tensor",
    lambda: tracer.set_mesh(vertices.cpu(), triangles),
)
noncontiguous_vertices = torch.empty((3, 4), dtype=torch.float32, device=device).t()
expect_failure(
    "noncontiguous vertices",
    "vertices must be contiguous",
    lambda: tracer.set_mesh(noncontiguous_vertices, triangles),
)

tracer.set_mesh(vertices, triangles)
expect_failure(
    "mismatched pose counts",
    "pose counts must match",
    lambda: tracer.set_scan(odometry, azimuths[:-1]),
)
tracer.set_scan(odometry, azimuths)
depth = tracer.trace(identity)

pose0 = depth[0]
positive = pose0[pose0 > 0]
center = pose0[30, 30].item()
hit_count = int(positive.numel())
minimum = positive.min().item()
maximum = positive.max().item()

print("output shape:", tuple(depth.shape))
print("dtype:", depth.dtype)
print("device:", depth.device)
print("is_contiguous:", depth.is_contiguous())
print("center range:", center)
print("pose 0 hit count:", hit_count)
print("minimum positive range:", minimum)
print("maximum range:", maximum)

assert depth.shape == (400, 61, 61)
assert depth.dtype == torch.float32
assert depth.is_cuda and depth.is_contiguous()
assert abs(center - 10.0) < 1.0e-3
assert hit_count == 1225
assert abs(minimum - 10.0) < 1.0e-3
assert abs(maximum - 10.0088) < 1.0e-3

test_stream = torch.cuda.Stream(device=device)
with torch.cuda.stream(test_stream):
    streamed_depth = tracer.trace(identity)
test_stream.synchronize()
assert abs(streamed_depth[0, 30, 30].item() - 10.0) < 1.0e-3
print("validation passed: current non-default CUDA stream")
print("all OptiX PyTorch tests passed")
