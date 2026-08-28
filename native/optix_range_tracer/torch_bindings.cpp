#include "OptixRangeTracer.h"

#include <torch/extension.h>

#include <c10/cuda/CUDAGuard.h>
#include <c10/cuda/CUDAFunctions.h>
#include <c10/cuda/CUDAStream.h>

#include <cstdint>
#include <limits>
#include <memory>
#include <string>

namespace py = pybind11;

namespace
{
void checkCudaTensor(const torch::Tensor& tensor,
                     const char* name,
                     at::ScalarType dtype,
                     int deviceIndex)
{
    TORCH_CHECK(tensor.defined(), name, " must be defined");
    TORCH_CHECK(tensor.is_cuda(), name, " must be a CUDA tensor");
    TORCH_CHECK(tensor.scalar_type() == dtype, name, " must have dtype ", dtype,
                ", got ", tensor.scalar_type());
    TORCH_CHECK(tensor.is_contiguous(), name, " must be contiguous");
    TORCH_CHECK(tensor.get_device() == deviceIndex, name, " must be on cuda:",
                deviceIndex, ", got ", tensor.device());
}

class TorchOptixRangeTracer
{
public:
    TorchOptixRangeTracer(int width, int height,
                          float thetaMin, float thetaMax,
                          float phiMin, float phiMax,
                          float minRange)
        : m_deviceIndex(c10::cuda::current_device())
    {
        c10::cuda::CUDAGuard guard(m_deviceIndex);
        GeometryConfig geometry;
        geometry.width = width;
        geometry.height = height;
        geometry.thetaMinDegrees = thetaMin;
        geometry.thetaMaxDegrees = thetaMax;
        geometry.phiMinDegrees = phiMin;
        geometry.phiMaxDegrees = phiMax;
        geometry.minRangeMeters = minRange;
        m_tracer = std::make_unique<OptixRangeTracer>(geometry);
    }

    ~TorchOptixRangeTracer()
    {
        c10::cuda::CUDAGuard guard(m_deviceIndex);
        m_tracer.reset();
    }

    void setMesh(const torch::Tensor& vertices, const torch::Tensor& triangles)
    {
        checkCudaTensor(vertices, "vertices", at::kFloat, m_deviceIndex);
        checkCudaTensor(triangles, "triangles", at::kInt, m_deviceIndex);
        TORCH_CHECK(vertices.dim() == 2 && vertices.size(1) == 3,
                    "vertices must have shape [N, 3], got ", vertices.sizes());
        TORCH_CHECK(triangles.dim() == 2 && triangles.size(1) == 3,
                    "triangles must have shape [F, 3], got ", triangles.sizes());
        TORCH_CHECK(vertices.size(0) >= 3, "vertices must contain at least 3 rows");
        TORCH_CHECK(triangles.size(0) > 0, "triangles must contain at least one row");
        TORCH_CHECK(vertices.size(0) <= std::numeric_limits<unsigned int>::max() &&
                    triangles.size(0) <= std::numeric_limits<unsigned int>::max(),
                    "mesh exceeds OptiX 32-bit element counts");

        c10::cuda::CUDAGuard guard(m_deviceIndex);
        const int32_t minimum = triangles.min().item<int32_t>();
        const int32_t maximum = triangles.max().item<int32_t>();
        TORCH_CHECK(minimum >= 0, "triangles contains a negative vertex index");
        TORCH_CHECK(static_cast<int64_t>(maximum) < vertices.size(0),
                    "triangles contains vertex index ", maximum,
                    " but vertices has only ", vertices.size(0), " rows");

        const cudaStream_t stream = c10::cuda::getCurrentCUDAStream(m_deviceIndex).stream();
        m_tracer->setMeshDevice(
            reinterpret_cast<const float3*>(vertices.data_ptr<float>()),
            static_cast<std::size_t>(vertices.size(0)),
            reinterpret_cast<const uint3*>(triangles.data_ptr<int32_t>()),
            static_cast<std::size_t>(triangles.size(0)), stream);
        m_meshSet = true;
    }

    void setScan(const torch::Tensor& odomTransforms,
                 const torch::Tensor& radarAzimuths)
    {
        checkCudaTensor(odomTransforms, "odom_transforms", at::kFloat, m_deviceIndex);
        checkCudaTensor(radarAzimuths, "radar_azimuths", at::kFloat, m_deviceIndex);
        TORCH_CHECK(odomTransforms.dim() == 3 && odomTransforms.size(1) == 4 &&
                    odomTransforms.size(2) == 4,
                    "odom_transforms must have shape [B, 4, 4], got ",
                    odomTransforms.sizes());
        TORCH_CHECK(radarAzimuths.dim() == 1,
                    "radar_azimuths must have shape [B], got ", radarAzimuths.sizes());
        TORCH_CHECK(odomTransforms.size(0) == radarAzimuths.size(0),
                    "odom_transforms and radar_azimuths pose counts must match, got ",
                    odomTransforms.size(0), " and ", radarAzimuths.size(0));
        TORCH_CHECK(odomTransforms.size(0) > 0, "scan pose count must be positive");
        TORCH_CHECK(odomTransforms.size(0) <= std::numeric_limits<unsigned int>::max(),
                    "scan pose count exceeds the OptiX launch dimension type");

        const uint64_t totalRays = static_cast<uint64_t>(odomTransforms.size(0)) *
                                   m_tracer->height() * m_tracer->width();
        TORCH_CHECK(totalRays <= (uint64_t{1} << 30),
                    "OptiX launch contains ", totalRays,
                    " rays; the width*height*depth limit is 2^30");

        c10::cuda::CUDAGuard guard(m_deviceIndex);
        const cudaStream_t stream = c10::cuda::getCurrentCUDAStream(m_deviceIndex).stream();
        m_tracer->setScanDevice(odomTransforms.data_ptr<float>(),
                                radarAzimuths.data_ptr<float>(),
                                static_cast<std::size_t>(odomTransforms.size(0)), stream);
        m_scanSet = true;
    }

    torch::Tensor trace(const torch::Tensor& currentTransform)
    {
        TORCH_CHECK(m_meshSet, "set_mesh() must be called before trace()");
        TORCH_CHECK(m_scanSet, "set_scan() must be called before trace()");
        checkCudaTensor(currentTransform, "T_current", at::kFloat, m_deviceIndex);
        TORCH_CHECK(currentTransform.dim() == 2 && currentTransform.size(0) == 4 &&
                    currentTransform.size(1) == 4,
                    "T_current must have shape [4, 4], got ", currentTransform.sizes());

        c10::cuda::CUDAGuard guard(m_deviceIndex);
        auto output = torch::empty(
            {static_cast<int64_t>(m_tracer->poseCount()),
             static_cast<int64_t>(m_tracer->height()),
             static_cast<int64_t>(m_tracer->width())},
            torch::TensorOptions().device(torch::Device(torch::kCUDA, m_deviceIndex))
                                  .dtype(torch::kFloat32));
        const cudaStream_t stream = c10::cuda::getCurrentCUDAStream(m_deviceIndex).stream();
        m_tracer->traceDevice(currentTransform.data_ptr<float>(),
                              output.data_ptr<float>(), stream);
        return output;
    }

private:
    int m_deviceIndex;
    std::unique_ptr<OptixRangeTracer> m_tracer;
    bool m_meshSet = false;
    bool m_scanSet = false;
};
}

PYBIND11_MODULE(optix_range_tracer, module)
{
    module.doc() = "PyTorch binding for the OptiX batched range tracer";
    py::class_<TorchOptixRangeTracer>(module, "OptixRangeTracer")
        .def(py::init<int, int, float, float, float, float, float>(),
             py::arg("width") = 61,
             py::arg("height") = 61,
             py::arg("theta_min") = -3.0f,
             py::arg("theta_max") = 3.0f,
             py::arg("phi_min") = -3.0f,
             py::arg("phi_max") = 3.0f,
             py::arg("min_range") = 0.0f,
             "Angles are specified in degrees; the tracer is tied to the current CUDA device.")
        .def("set_mesh", &TorchOptixRangeTracer::setMesh,
             "Copy CUDA vertices and int32 triangle indices into tracer-owned buffers.")
        .def("set_scan", &TorchOptixRangeTracer::setScan,
             "Copy CUDA row-major transforms and azimuths into tracer-owned buffers.")
        .def("trace", &TorchOptixRangeTracer::trace,
             "Trace on PyTorch's current CUDA stream and return [B, H, W].");
}
