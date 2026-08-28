#include "OptixRangeTracer.h"

#include <optix_function_table_definition.h>
#include <optix_stack_size.h>
#include <optix_stubs.h>

#include <sampleConfig.h>
#include <sutil/Exception.h>
#include <sutil/sutil.h>

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{
template <typename T>
struct SbtRecord
{
    __align__(OPTIX_SBT_RECORD_ALIGNMENT)
    char header[OPTIX_SBT_RECORD_HEADER_SIZE];
    T data;
};

using RayGenSbtRecord = SbtRecord<RayGenData>;
using MissSbtRecord = SbtRecord<MissData>;
using HitGroupSbtRecord = SbtRecord<HitGroupData>;

void contextLogCallback(unsigned int level, const char* tag,
                        const char* message, void*)
{
    std::cerr << '[' << std::setw(2) << level << "][" << tag << "]: "
              << message << '\n';
}

template <typename T>
void freeCuda(T& pointer) noexcept
{
    if (pointer)
    {
        cudaFree(reinterpret_cast<void*>(pointer));
        pointer = 0;
    }
}
}

OptixRangeTracer::OptixRangeTracer(const GeometryConfig& geometry)
    : m_geometry(geometry)
{
    if (geometry.width < 2 || geometry.height < 2)
        throw std::invalid_argument("Patch width and height must be at least 2");
    if (!(geometry.thetaMinDegrees < geometry.thetaMaxDegrees) ||
        !(geometry.phiMinDegrees < geometry.phiMaxDegrees))
        throw std::invalid_argument("Patch angle minima must be below maxima");
    if (!std::isfinite(geometry.minRangeMeters) || geometry.minRangeMeters < 0.0f)
        throw std::invalid_argument("Minimum range must be finite and non-negative");

    try
    {
        createContextAndPipeline();
        createSbt();
        uploadLocalDirections();
        CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&m_dParams), sizeof(Params)));
        CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&m_dCurrentMatrix), 16 * sizeof(float)));
    }
    catch (...)
    {
        destroy();
        throw;
    }
}

OptixRangeTracer::~OptixRangeTracer()
{
    destroy();
}

void OptixRangeTracer::createContextAndPipeline()
{
    CUDA_CHECK(cudaFree(nullptr));
    OPTIX_CHECK(optixInit());

    OptixDeviceContextOptions contextOptions = {};
    contextOptions.logCallbackFunction = contextLogCallback;
    contextOptions.logCallbackLevel = 4;
    OPTIX_CHECK(optixDeviceContextCreate(nullptr, &contextOptions, &m_context));

    OptixModuleCompileOptions moduleOptions = {};
#if OPTIX_DEBUG_DEVICE_CODE
    moduleOptions.optLevel = OPTIX_COMPILE_OPTIMIZATION_LEVEL_0;
    moduleOptions.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_FULL;
#else
    moduleOptions.optLevel = OPTIX_COMPILE_OPTIMIZATION_DEFAULT;
    moduleOptions.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_MINIMAL;
#endif

    OptixPipelineCompileOptions pipelineOptions = {};
    pipelineOptions.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_GAS;
    pipelineOptions.numPayloadValues = 1;
    pipelineOptions.numAttributeValues = 2;
    pipelineOptions.pipelineLaunchParamsVariableName = "params";
    pipelineOptions.usesPrimitiveTypeFlags = OPTIX_PRIMITIVE_TYPE_FLAGS_TRIANGLE;

    size_t inputSize = 0;
    const char* input = sutil::getInputData(
        OPTIX_SAMPLE_NAME, OPTIX_SAMPLE_DIR, "optixTriangle.cu", inputSize);
    OPTIX_CHECK_LOG(optixModuleCreate(
        m_context, &moduleOptions, &pipelineOptions, input, inputSize,
        LOG, &LOG_SIZE, &m_module));

    OptixProgramGroupOptions groupOptions = {};
    OptixProgramGroupDesc description = {};
    description.kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
    description.raygen.module = m_module;
    description.raygen.entryFunctionName = "__raygen__rg";
    OPTIX_CHECK_LOG(optixProgramGroupCreate(
        m_context, &description, 1, &groupOptions, LOG, &LOG_SIZE,
        &m_raygenProgramGroup));

    description = {};
    description.kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
    description.miss.module = m_module;
    description.miss.entryFunctionName = "__miss__ms";
    OPTIX_CHECK_LOG(optixProgramGroupCreate(
        m_context, &description, 1, &groupOptions, LOG, &LOG_SIZE,
        &m_missProgramGroup));

    description = {};
    description.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    description.hitgroup.moduleCH = m_module;
    description.hitgroup.entryFunctionNameCH = "__closesthit__ch";
    OPTIX_CHECK_LOG(optixProgramGroupCreate(
        m_context, &description, 1, &groupOptions, LOG, &LOG_SIZE,
        &m_hitgroupProgramGroup));

    OptixProgramGroup groups[] = {
        m_raygenProgramGroup, m_missProgramGroup, m_hitgroupProgramGroup};
    OptixPipelineLinkOptions linkOptions = {};
    linkOptions.maxTraceDepth = 1;
    OPTIX_CHECK_LOG(optixPipelineCreate(
        m_context, &pipelineOptions, &linkOptions, groups, 3,
        LOG, &LOG_SIZE, &m_pipeline));

    OptixStackSizes stackSizes = {};
    for (OptixProgramGroup group : groups)
        OPTIX_CHECK(optixUtilAccumulateStackSizes(group, &stackSizes, m_pipeline));

    uint32_t directTraversal = 0;
    uint32_t directState = 0;
    uint32_t continuation = 0;
    OPTIX_CHECK(optixUtilComputeStackSizes(
        &stackSizes, 1, 0, 0, &directTraversal, &directState, &continuation));
    OPTIX_CHECK(optixPipelineSetStackSize(
        m_pipeline, directTraversal, directState, continuation, 1));
}

void OptixRangeTracer::createSbt()
{
    RayGenSbtRecord raygen = {};
    MissSbtRecord miss = {};
    HitGroupSbtRecord hit = {};
    OPTIX_CHECK(optixSbtRecordPackHeader(m_raygenProgramGroup, &raygen));
    OPTIX_CHECK(optixSbtRecordPackHeader(m_missProgramGroup, &miss));
    OPTIX_CHECK(optixSbtRecordPackHeader(m_hitgroupProgramGroup, &hit));

    CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&m_sbt.raygenRecord), sizeof(raygen)));
    CUDA_CHECK(cudaMemcpy(reinterpret_cast<void*>(m_sbt.raygenRecord),
                          &raygen, sizeof(raygen), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&m_sbt.missRecordBase), sizeof(miss)));
    CUDA_CHECK(cudaMemcpy(reinterpret_cast<void*>(m_sbt.missRecordBase),
                          &miss, sizeof(miss), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&m_sbt.hitgroupRecordBase), sizeof(hit)));
    CUDA_CHECK(cudaMemcpy(reinterpret_cast<void*>(m_sbt.hitgroupRecordBase),
                          &hit, sizeof(hit), cudaMemcpyHostToDevice));

    m_sbt.missRecordStrideInBytes = sizeof(miss);
    m_sbt.missRecordCount = 1;
    m_sbt.hitgroupRecordStrideInBytes = sizeof(hit);
    m_sbt.hitgroupRecordCount = 1;
}

void OptixRangeTracer::uploadLocalDirections()
{
    const std::size_t count = static_cast<std::size_t>(m_geometry.width) *
                              static_cast<std::size_t>(m_geometry.height);
    std::vector<float3> directions(count);
    constexpr float radians = 3.14159265358979323846f / 180.0f;

    for (int row = 0; row < m_geometry.height; ++row)
    {
        const float rowFraction = static_cast<float>(row) / (m_geometry.height - 1);
        // Pixel convention: row 0 is phi_max, last row is phi_min.
        const float phi = (m_geometry.phiMaxDegrees +
            (m_geometry.phiMinDegrees - m_geometry.phiMaxDegrees) * rowFraction) * radians;
        for (int column = 0; column < m_geometry.width; ++column)
        {
            const float columnFraction = static_cast<float>(column) / (m_geometry.width - 1);
            const float theta = (m_geometry.thetaMinDegrees +
                (m_geometry.thetaMaxDegrees - m_geometry.thetaMinDegrees) * columnFraction) * radians;
            directions[static_cast<std::size_t>(row) * m_geometry.width + column] =
                make_float3(std::cos(phi) * std::cos(theta),
                            std::cos(phi) * std::sin(theta),
                            std::sin(phi));
        }
    }

    CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&m_dLocalDirections),
                          directions.size() * sizeof(float3)));
    CUDA_CHECK(cudaMemcpy(reinterpret_cast<void*>(m_dLocalDirections),
                          directions.data(), directions.size() * sizeof(float3),
                          cudaMemcpyHostToDevice));
}

void OptixRangeTracer::setMesh(const float3* vertices, std::size_t vertexCount,
                               const uint3* triangles, std::size_t triangleCount)
{
    if (!vertices || !triangles || vertexCount < 3 || triangleCount == 0)
        throw std::invalid_argument("Mesh requires non-null arrays, at least 3 vertices, and a triangle");
    if (vertexCount > std::numeric_limits<unsigned int>::max() ||
        triangleCount > std::numeric_limits<unsigned int>::max())
        throw std::invalid_argument("Mesh exceeds OptiX 32-bit element counts");
    for (std::size_t i = 0; i < triangleCount; ++i)
        if (triangles[i].x >= vertexCount || triangles[i].y >= vertexCount ||
            triangles[i].z >= vertexCount)
            throw std::invalid_argument("Triangle index is outside the vertex array");

    setMeshCopy(vertices, vertexCount, triangles, triangleCount,
                cudaMemcpyHostToDevice, nullptr);
}

void OptixRangeTracer::setMeshDevice(const float3* vertices, std::size_t vertexCount,
                                     const uint3* triangles, std::size_t triangleCount,
                                     cudaStream_t stream)
{
    if (!vertices || !triangles || vertexCount < 3 || triangleCount == 0)
        throw std::invalid_argument("Mesh requires non-null arrays, at least 3 vertices, and a triangle");
    if (vertexCount > std::numeric_limits<unsigned int>::max() ||
        triangleCount > std::numeric_limits<unsigned int>::max())
        throw std::invalid_argument("Mesh exceeds OptiX 32-bit element counts");

    setMeshCopy(vertices, vertexCount, triangles, triangleCount,
                cudaMemcpyDeviceToDevice, stream);
}

void OptixRangeTracer::setMeshCopy(const float3* vertices, std::size_t vertexCount,
                                   const uint3* triangles, std::size_t triangleCount,
                                   cudaMemcpyKind copyKind, cudaStream_t stream)
{

    CUdeviceptr newVertices = 0;
    CUdeviceptr newTriangles = 0;
    CUdeviceptr temporary = 0;
    CUdeviceptr uncompacted = 0;
    CUdeviceptr compacted = 0;
    CUdeviceptr dCompactedSize = 0;
    OptixTraversableHandle newHandle = 0;

    try
    {
        CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&newVertices), vertexCount * sizeof(float3)));
        CUDA_CHECK(cudaMemcpyAsync(reinterpret_cast<void*>(newVertices), vertices,
                                   vertexCount * sizeof(float3), copyKind, stream));
        CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&newTriangles), triangleCount * sizeof(uint3)));
        CUDA_CHECK(cudaMemcpyAsync(reinterpret_cast<void*>(newTriangles), triangles,
                                   triangleCount * sizeof(uint3), copyKind, stream));

        unsigned int flags[] = {OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT};
        OptixBuildInput input = {};
        input.type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;
        input.triangleArray.vertexBuffers = &newVertices;
        input.triangleArray.numVertices = static_cast<unsigned int>(vertexCount);
        input.triangleArray.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
        input.triangleArray.vertexStrideInBytes = sizeof(float3);
        input.triangleArray.indexBuffer = newTriangles;
        input.triangleArray.numIndexTriplets = static_cast<unsigned int>(triangleCount);
        input.triangleArray.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
        input.triangleArray.indexStrideInBytes = sizeof(uint3);
        input.triangleArray.flags = flags;
        input.triangleArray.numSbtRecords = 1;

        OptixAccelBuildOptions options = {};
        options.buildFlags = OPTIX_BUILD_FLAG_ALLOW_COMPACTION | OPTIX_BUILD_FLAG_PREFER_FAST_TRACE;
        options.operation = OPTIX_BUILD_OPERATION_BUILD;
        OptixAccelBufferSizes sizes = {};
        OPTIX_CHECK(optixAccelComputeMemoryUsage(m_context, &options, &input, 1, &sizes));
        CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&temporary), sizes.tempSizeInBytes));
        CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&uncompacted), sizes.outputSizeInBytes));

        CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&dCompactedSize), sizeof(std::size_t)));
        OptixAccelEmitDesc emitted = {};
        emitted.type = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE;
        emitted.result = dCompactedSize;
        OPTIX_CHECK(optixAccelBuild(
            m_context, stream, &options, &input, 1, temporary, sizes.tempSizeInBytes,
            uncompacted, sizes.outputSizeInBytes, &newHandle, &emitted, 1));

        std::size_t compactedSize = 0;
        CUDA_CHECK(cudaMemcpyAsync(&compactedSize, reinterpret_cast<void*>(dCompactedSize),
                                   sizeof(compactedSize), cudaMemcpyDeviceToHost, stream));
        CUDA_CHECK(cudaStreamSynchronize(stream));
        freeCuda(dCompactedSize);
        freeCuda(temporary);

        if (compactedSize < sizes.outputSizeInBytes)
        {
            CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&compacted), compactedSize));
            OPTIX_CHECK(optixAccelCompact(m_context, stream, newHandle,
                                          compacted, compactedSize, &newHandle));
            CUDA_CHECK(cudaStreamSynchronize(stream));
            freeCuda(uncompacted);
        }
        else
        {
            compacted = uncompacted;
            uncompacted = 0;
        }
    }
    catch (...)
    {
        freeCuda(temporary);
        freeCuda(dCompactedSize);
        freeCuda(uncompacted);
        freeCuda(compacted);
        freeCuda(newTriangles);
        freeCuda(newVertices);
        throw;
    }

    releaseMesh();
    m_dVertices = newVertices;
    m_dTriangles = newTriangles;
    m_dGasOutput = compacted;
    m_gasHandle = newHandle;
}

void OptixRangeTracer::setScan(const RigidTransform* odomTransforms,
                               const float* radarAzimuths, std::size_t poseCount)
{
    if (!odomTransforms || !radarAzimuths || poseCount == 0 ||
        poseCount > std::numeric_limits<unsigned int>::max())
        throw std::invalid_argument("Scan requires non-null arrays and a valid nonzero pose count");

    std::vector<float> matrices(poseCount * 16, 0.0f);
    for (std::size_t i = 0; i < poseCount; ++i)
    {
        const RigidTransform& transform = odomTransforms[i];
        float* matrix = matrices.data() + i * 16;
        matrix[0] = transform.row0.x; matrix[1] = transform.row0.y; matrix[2] = transform.row0.z; matrix[3] = transform.translation.x;
        matrix[4] = transform.row1.x; matrix[5] = transform.row1.y; matrix[6] = transform.row1.z; matrix[7] = transform.translation.y;
        matrix[8] = transform.row2.x; matrix[9] = transform.row2.y; matrix[10] = transform.row2.z; matrix[11] = transform.translation.z;
        matrix[15] = 1.0f;
    }
    setScanCopy(matrices.data(), radarAzimuths, poseCount,
                cudaMemcpyHostToDevice, nullptr);
}

void OptixRangeTracer::setScanDevice(const float* odomMatrices,
                                     const float* radarAzimuths,
                                     std::size_t poseCount,
                                     cudaStream_t stream)
{
    if (!odomMatrices || !radarAzimuths || poseCount == 0 ||
        poseCount > std::numeric_limits<unsigned int>::max())
        throw std::invalid_argument("Scan requires non-null arrays and a valid nonzero pose count");
    setScanCopy(odomMatrices, radarAzimuths, poseCount,
                cudaMemcpyDeviceToDevice, stream);
}

void OptixRangeTracer::setScanCopy(const float* odomMatrices,
                                   const float* radarAzimuths,
                                   std::size_t poseCount,
                                   cudaMemcpyKind copyKind,
                                   cudaStream_t stream)
{
    CUdeviceptr newMatrices = 0;
    CUdeviceptr newAzimuths = 0;
    try
    {
        CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&newMatrices),
                              poseCount * 16 * sizeof(float)));
        CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&newAzimuths), poseCount * sizeof(float)));
        if (copyKind == cudaMemcpyHostToDevice)
        {
            CUDA_CHECK(cudaMemcpy(reinterpret_cast<void*>(newMatrices), odomMatrices,
                                  poseCount * 16 * sizeof(float), copyKind));
            CUDA_CHECK(cudaMemcpy(reinterpret_cast<void*>(newAzimuths), radarAzimuths,
                                  poseCount * sizeof(float), copyKind));
        }
        else
        {
            CUDA_CHECK(cudaMemcpyAsync(reinterpret_cast<void*>(newMatrices), odomMatrices,
                                       poseCount * 16 * sizeof(float), copyKind, stream));
            CUDA_CHECK(cudaMemcpyAsync(reinterpret_cast<void*>(newAzimuths), radarAzimuths,
                                       poseCount * sizeof(float), copyKind, stream));
        }
    }
    catch (...)
    {
        freeCuda(newAzimuths);
        freeCuda(newMatrices);
        throw;
    }

    releaseScan();
    m_dOdomMatrices = newMatrices;
    m_dRadarAzimuths = newAzimuths;
    m_poseCount = poseCount;
}

void OptixRangeTracer::trace(const RigidTransform& currentTransform,
                             float* outputRanges, cudaStream_t stream)
{
    launch(currentTransform, nullptr, outputRanges, stream);
}

void OptixRangeTracer::traceDevice(const float* currentMatrix,
                                   float* outputRanges,
                                   cudaStream_t stream)
{
    if (!currentMatrix)
        throw std::invalid_argument("Current transform CUDA pointer must not be null");
    CUDA_CHECK(cudaMemcpyAsync(reinterpret_cast<void*>(m_dCurrentMatrix), currentMatrix,
                               16 * sizeof(float), cudaMemcpyDeviceToDevice, stream));
    launch(RigidTransform{}, reinterpret_cast<const float*>(m_dCurrentMatrix),
           outputRanges, stream);
}

void OptixRangeTracer::launch(const RigidTransform& currentTransform,
                              const float* currentMatrix,
                              float* outputRanges,
                              cudaStream_t stream)
{
    if (!m_gasHandle)
        throw std::logic_error("setMesh must be called before trace");
    if (!m_poseCount)
        throw std::logic_error("setScan must be called before trace");
    if (!outputRanges)
        throw std::invalid_argument("Output CUDA pointer must not be null");

    Params params = {};
    params.ranges = outputRanges;
    params.localDirections = reinterpret_cast<const float3*>(m_dLocalDirections);
    params.odomMatrices = reinterpret_cast<const float*>(m_dOdomMatrices);
    params.radarAzimuths = reinterpret_cast<const float*>(m_dRadarAzimuths);
    params.currentMatrix = currentMatrix;
    params.currentTransform = currentTransform;
    params.patchWidth = static_cast<unsigned int>(m_geometry.width);
    params.patchHeight = static_cast<unsigned int>(m_geometry.height);
    params.poseCount = static_cast<unsigned int>(m_poseCount);
    params.minRangeMeters = m_geometry.minRangeMeters;
    params.handle = m_gasHandle;

    CUDA_CHECK(cudaMemcpyAsync(reinterpret_cast<void*>(m_dParams), &params,
                               sizeof(params), cudaMemcpyHostToDevice, stream));
    OPTIX_CHECK(optixLaunch(
        m_pipeline, stream, m_dParams, sizeof(Params), &m_sbt,
        params.patchWidth, params.patchHeight, params.poseCount));
}

void OptixRangeTracer::releaseMesh() noexcept
{
    freeCuda(m_dGasOutput);
    freeCuda(m_dTriangles);
    freeCuda(m_dVertices);
    m_gasHandle = 0;
}

void OptixRangeTracer::releaseScan() noexcept
{
    freeCuda(m_dRadarAzimuths);
    freeCuda(m_dOdomMatrices);
    m_poseCount = 0;
}

void OptixRangeTracer::destroy() noexcept
{
    releaseMesh();
    releaseScan();
    freeCuda(m_dParams);
    freeCuda(m_dCurrentMatrix);
    freeCuda(m_dLocalDirections);
    freeCuda(m_sbt.hitgroupRecordBase);
    freeCuda(m_sbt.missRecordBase);
    freeCuda(m_sbt.raygenRecord);

    if (m_pipeline) optixPipelineDestroy(m_pipeline);
    if (m_hitgroupProgramGroup) optixProgramGroupDestroy(m_hitgroupProgramGroup);
    if (m_missProgramGroup) optixProgramGroupDestroy(m_missProgramGroup);
    if (m_raygenProgramGroup) optixProgramGroupDestroy(m_raygenProgramGroup);
    if (m_module) optixModuleDestroy(m_module);
    if (m_context) optixDeviceContextDestroy(m_context);
    m_pipeline = nullptr;
    m_hitgroupProgramGroup = nullptr;
    m_missProgramGroup = nullptr;
    m_raygenProgramGroup = nullptr;
    m_module = nullptr;
    m_context = nullptr;
}
