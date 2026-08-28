#pragma once

#include <cuda_runtime.h>
#include <optix.h>

#include <cstddef>

enum RayType
{
    RAY_TYPE_RANGE = 0,
    RAY_TYPE_COUNT
};

struct GeometryConfig
{
    int width = 61;
    int height = 61;
    float thetaMinDegrees = -3.0f;
    float thetaMaxDegrees = 3.0f;
    float phiMinDegrees = -3.0f;
    float phiMaxDegrees = 3.0f;
    float minRangeMeters = 0.0f;
};

// Row-major rotation for p_destination = R * p_source + translation.
struct RigidTransform
{
    float3 row0;
    float3 row1;
    float3 row2;
    float3 translation;
};

struct Params
{
    float* ranges;
    const float3* localDirections;
    const float* odomMatrices;
    const float* radarAzimuths;
    const float* currentMatrix;
    RigidTransform currentTransform;
    unsigned int patchWidth;
    unsigned int patchHeight;
    unsigned int poseCount;
    float minRangeMeters;
    OptixTraversableHandle handle;
};

struct RayGenData {};
struct MissData {};
struct HitGroupData {};

class OptixRangeTracer
{
public:
    explicit OptixRangeTracer(const GeometryConfig& geometry);
    ~OptixRangeTracer();

    OptixRangeTracer(const OptixRangeTracer&) = delete;
    OptixRangeTracer& operator=(const OptixRangeTracer&) = delete;

    // Input arrays are host arrays and are copied into tracer-owned CUDA buffers.
    void setMesh(const float3* vertices,
                 std::size_t vertexCount,
                 const uint3* triangles,
                 std::size_t triangleCount);

    // CUDA inputs are copied device-to-device into tracer-owned buffers.
    void setMeshDevice(const float3* vertices,
                       std::size_t vertexCount,
                       const uint3* triangles,
                       std::size_t triangleCount,
                       cudaStream_t stream);

    // Input arrays are host arrays and are copied into tracer-owned CUDA buffers.
    void setScan(const RigidTransform* odomTransforms,
                 const float* radarAzimuths,
                 std::size_t poseCount);

    // Matrices are contiguous row-major [poseCount, 4, 4] CUDA floats.
    void setScanDevice(const float* odomMatrices,
                       const float* radarAzimuths,
                       std::size_t poseCount,
                       cudaStream_t stream);

    // outputRanges must point to width()*height()*poseCount() CUDA floats.
    // The caller owns stream synchronization and the output buffer lifetime.
    void trace(const RigidTransform& currentTransform,
               float* outputRanges,
               cudaStream_t stream = nullptr);

    // currentMatrix is a contiguous row-major [4, 4] CUDA float matrix.
    void traceDevice(const float* currentMatrix,
                     float* outputRanges,
                     cudaStream_t stream);

    std::size_t poseCount() const { return m_poseCount; }
    int width() const { return m_geometry.width; }
    int height() const { return m_geometry.height; }

private:
    void createContextAndPipeline();
    void createSbt();
    void uploadLocalDirections();
    void setMeshCopy(const float3* vertices,
                     std::size_t vertexCount,
                     const uint3* triangles,
                     std::size_t triangleCount,
                     cudaMemcpyKind copyKind,
                     cudaStream_t stream);
    void setScanCopy(const float* odomMatrices,
                     const float* radarAzimuths,
                     std::size_t poseCount,
                     cudaMemcpyKind copyKind,
                     cudaStream_t stream);
    void launch(const RigidTransform& currentTransform,
                const float* currentMatrix,
                float* outputRanges,
                cudaStream_t stream);
    void releaseMesh() noexcept;
    void releaseScan() noexcept;
    void destroy() noexcept;

    GeometryConfig m_geometry;
    std::size_t m_poseCount = 0;

    OptixDeviceContext m_context = nullptr;
    OptixModule m_module = nullptr;
    OptixProgramGroup m_raygenProgramGroup = nullptr;
    OptixProgramGroup m_missProgramGroup = nullptr;
    OptixProgramGroup m_hitgroupProgramGroup = nullptr;
    OptixPipeline m_pipeline = nullptr;
    OptixShaderBindingTable m_sbt = {};

    CUdeviceptr m_dLocalDirections = 0;
    CUdeviceptr m_dOdomMatrices = 0;
    CUdeviceptr m_dRadarAzimuths = 0;
    CUdeviceptr m_dCurrentMatrix = 0;
    CUdeviceptr m_dVertices = 0;
    CUdeviceptr m_dTriangles = 0;
    CUdeviceptr m_dGasOutput = 0;
    CUdeviceptr m_dParams = 0;
    OptixTraversableHandle m_gasHandle = 0;
};
