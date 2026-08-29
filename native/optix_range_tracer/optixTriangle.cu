#include <optix.h>

#include "OptixRangeTracer.h"

#include <sutil/vec_math.h>

extern "C" { __constant__ Params params; }

static __forceinline__ __device__ float3 multiplyRows(
    const RigidTransform& transform, const float3& value)
{
    return make_float3(dot(transform.row0, value),
                       dot(transform.row1, value),
                       dot(transform.row2, value));
}

static __forceinline__ __device__ float3 multiplyTranspose(
    const RigidTransform& transform, const float3& value)
{
    return make_float3(transform.row0.x * value.x + transform.row1.x * value.y + transform.row2.x * value.z,
                       transform.row0.y * value.x + transform.row1.y * value.y + transform.row2.y * value.z,
                       transform.row0.z * value.x + transform.row1.z * value.y + transform.row2.z * value.z);
}

static __forceinline__ __device__ RigidTransform loadTransform(const float* matrix)
{
    RigidTransform transform;
    transform.row0 = make_float3(matrix[0], matrix[1], matrix[2]);
    transform.row1 = make_float3(matrix[4], matrix[5], matrix[6]);
    transform.row2 = make_float3(matrix[8], matrix[9], matrix[10]);
    transform.translation = make_float3(matrix[3], matrix[7], matrix[11]);
    return transform;
}

extern "C" __global__ void __raygen__rg()
{
    const uint3 idx = optixGetLaunchIndex();
    const float3 local = params.localDirections[idx.y * params.patchWidth + idx.x];
    const RigidTransform current = params.currentMatrix
        ? loadTransform(params.currentMatrix)
        : params.currentTransform;
    const RigidTransform odom = loadTransform(params.odomMatrices + idx.z * 16);

    // Compose T_azimuth_from_enu = T_odom * T_current.
    RigidTransform azimuthFromEnu;
    azimuthFromEnu.row0 = multiplyTranspose(current, odom.row0);
    azimuthFromEnu.row1 = multiplyTranspose(current, odom.row1);
    azimuthFromEnu.row2 = multiplyTranspose(current, odom.row2);
    azimuthFromEnu.translation =
        multiplyRows(odom, current.translation) + odom.translation;

    // Invert the Python point-coordinate azimuth rotation for ray directions.
    const float azimuth = params.radarAzimuths[idx.z];
    float sine;
    float cosine;
    __sincosf(azimuth, &sine, &cosine);
    const float3 directionReference = make_float3(
        cosine * local.x - sine * local.y,
        sine * local.x + cosine * local.y,
        local.z);

    // p_azimuth = R p_enu + t, hence o_enu = -R^T t and d_enu = R^T d_azimuth.
    const float3 rayOrigin = -multiplyTranspose(azimuthFromEnu, azimuthFromEnu.translation);
    const float3 rayDirection = normalize(multiplyTranspose(azimuthFromEnu, directionReference));

    unsigned int payload;
    optixTrace(params.handle, rayOrigin, rayDirection,
               params.minRangeMeters, 1.0e16f, 0.0f, OptixVisibilityMask(255),
               OPTIX_RAY_FLAG_NONE, 0, RAY_TYPE_COUNT, RAY_TYPE_RANGE, payload);

    const unsigned long long output =
        (static_cast<unsigned long long>(idx.z) * params.patchHeight + idx.y) *
        params.patchWidth + idx.x;
    params.ranges[output] = __uint_as_float(payload);
}

extern "C" __global__ void __miss__ms()
{
    optixSetPayload_0(__float_as_uint(0.0f));
}

extern "C" __global__ void __closesthit__ch()
{
    optixSetPayload_0(__float_as_uint(optixGetRayTmax()));
}
