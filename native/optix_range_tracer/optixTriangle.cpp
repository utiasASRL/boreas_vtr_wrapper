#include "OptixRangeTracer.h"

#include <sutil/Exception.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
RigidTransform identityTransform()
{
    return {make_float3(1.0f, 0.0f, 0.0f),
            make_float3(0.0f, 1.0f, 0.0f),
            make_float3(0.0f, 0.0f, 1.0f),
            make_float3(0.0f, 0.0f, 0.0f)};
}

void savePgm(const std::string& path,
             const std::vector<unsigned char>& pixels,
             int width,
             int height)
{
    if (width <= 0 || height <= 0 ||
        pixels.size() != static_cast<std::size_t>(width) * height)
        throw std::invalid_argument("Invalid PGM dimensions or pixel count");

    std::ofstream output(path, std::ios::binary);
    if (!output)
        throw std::runtime_error("Could not open debug image: " + path);
    output << "P5\n" << width << ' ' << height << "\n255\n";
    output.write(reinterpret_cast<const char*>(pixels.data()), pixels.size());
    if (!output)
        throw std::runtime_error("Could not write debug image: " + path);
}

void usage(const char* executable)
{
    std::cout << "Usage: " << executable
              << " [--dim=WIDTHxHEIGHT] [--poses=COUNT] [--save-debug-images]\n";
}
}

int main(int argc, char** argv)
try
{
    GeometryConfig geometry;
    std::size_t poseCount = 400;
    bool saveDebugImages = false;
    for (int i = 1; i < argc; ++i)
    {
        const std::string argument(argv[i]);
        if (argument == "--help" || argument == "-h")
        {
            usage(argv[0]);
            return 0;
        }
        if (argument == "--save-debug-images")
            saveDebugImages = true;
        else if (argument.rfind("--dim=", 0) == 0)
        {
            const std::string value = argument.substr(6);
            const std::size_t separator = value.find('x');
            if (separator == std::string::npos)
                throw std::invalid_argument("Dimensions must use WIDTHxHEIGHT");
            geometry.width = std::stoi(value.substr(0, separator));
            geometry.height = std::stoi(value.substr(separator + 1));
        }
        else if (argument.rfind("--poses=", 0) == 0)
            poseCount = std::stoul(argument.substr(8));
        else
            throw std::invalid_argument("Unknown option: " + argument);
    }

    // A 0.6 m square centered ten metres along ENU +x. With identity transforms,
    // the center ray must hit at exactly 10 m while outer patch rays miss.
    const float3 vertices[] = {
        make_float3(10.0f, -0.3f, -0.3f), make_float3(10.0f, 0.3f, -0.3f),
        make_float3(10.0f, 0.3f, 0.3f), make_float3(10.0f, -0.3f, 0.3f)};
    const uint3 triangles[] = {make_uint3(0, 1, 2), make_uint3(0, 2, 3)};

    OptixRangeTracer tracer(geometry);
    tracer.setMesh(vertices, 4, triangles, 2);
    std::vector<RigidTransform> odometry(poseCount, identityTransform());
    std::vector<float> azimuths(poseCount, 0.0f);
    tracer.setScan(odometry.data(), azimuths.data(), poseCount);

    const std::size_t raysPerPose = static_cast<std::size_t>(tracer.width()) * tracer.height();
    const std::size_t totalRays = raysPerPose * tracer.poseCount();
    float* deviceRanges = nullptr;
    CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&deviceRanges), totalRays * sizeof(float)));
    tracer.trace(identityTransform(), deviceRanges);

    std::vector<float> firstPose(raysPerPose);
    CUDA_CHECK(cudaMemcpy(firstPose.data(), deviceRanges, raysPerPose * sizeof(float),
                          cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaFree(deviceRanges));

    const std::size_t center = static_cast<std::size_t>(tracer.height() / 2) *
                               tracer.width() + tracer.width() / 2;
    std::size_t hitCount = 0;
    float minimum = std::numeric_limits<float>::max();
    float maximum = 0.0f;
    for (float range : firstPose)
        if (range > 0.0f)
        {
            ++hitCount;
            minimum = std::min(minimum, range);
            maximum = std::max(maximum, range);
        }

    std::cout << "Launch dimensions: " << tracer.width() << " x " << tracer.height()
              << " x " << tracer.poseCount() << " (" << totalRays << " rays)\n"
              << "Pose 0 center range: " << firstPose[center] << "\n"
              << "Pose 0 hit count: " << hitCount << " / " << raysPerPose << "\n";
    if (hitCount)
        std::cout << "Pose 0 minimum hit range: " << minimum << "\n"
                  << "Pose 0 maximum hit range: " << maximum << "\n";

    if (saveDebugImages)
    {
        std::vector<unsigned char> hitMask(raysPerPose, 0);
        std::vector<unsigned char> normalizedRanges(raysPerPose, 0);
        const bool constantHitRange = hitCount && maximum == minimum;
        for (std::size_t i = 0; i < raysPerPose; ++i)
        {
            if (firstPose[i] <= 0.0f)
                continue;
            hitMask[i] = 255;
            normalizedRanges[i] = constantHitRange
                ? 255
                : static_cast<unsigned char>(1.0f +
                    254.0f * (firstPose[i] - minimum) / (maximum - minimum));
        }

        const std::string hitMaskPath = "pose0_hit_mask.pgm";
        const std::string rangePath = "pose0_range.pgm";
        savePgm(hitMaskPath, hitMask, tracer.width(), tracer.height());
        savePgm(rangePath, normalizedRanges, tracer.width(), tracer.height());
        std::cout << "Wrote debug image: " << hitMaskPath << "\n"
                  << "Wrote debug image: " << rangePath << "\n";
    }

    std::cout << "Pose 0 coarse hit/miss patch:\n";
    const int rowStep = std::max(1, tracer.height() / 10);
    const int columnStep = std::max(1, tracer.width() / 10);
    for (int row = 0; row < tracer.height(); row += rowStep)
    {
        for (int column = 0; column < tracer.width(); column += columnStep)
            std::cout << (firstPose[static_cast<std::size_t>(row) * tracer.width() + column] > 0.0f ? "X " : ". ");
        std::cout << '\n';
    }

    if (std::fabs(firstPose[center] - 10.0f) > 1.0e-3f || hitCount == 0 || hitCount == raysPerPose)
        throw std::runtime_error("Standalone geometry check failed");
    return 0;
}
catch (const std::exception& exception)
{
    std::cerr << "Caught exception: " << exception.what() << '\n';
    return 1;
}
