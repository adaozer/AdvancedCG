#pragma once

#include "Core.h"
#include <random>
#include <algorithm>

class Sampler
{
public:
	virtual float next() = 0;
};

class MTRandom : public Sampler
{
public:
	std::mt19937 generator;
	std::uniform_real_distribution<float> dist;
	MTRandom(unsigned int seed = 1) : dist(0.0f, 1.0f)
	{
		generator.seed(seed);
	}
	float next()
	{
		return dist(generator);
	}
};

// Note all of these distributions assume z-up coordinate system
class SamplingDistributions
{
public:
	static Vec3 uniformSampleHemisphere(float r1, float r2)
	{
		float a1 = std::acos(r1);
		float a2 = 2.f * M_PI * r2;
		return SphericalCoordinates::sphericalToWorld(a1, a2);

	}
	static float uniformHemispherePDF(const Vec3 wi)
	{
		return (1.f / (2.f * M_PI));
	}
	static Vec3 cosineSampleHemisphere(float r1, float r2)
	{
		float a1 = std::acos(sqrt(r1));
		float a2 = 2.f * M_PI * r2;
		return SphericalCoordinates::sphericalToWorld(a1, a2);
	}
	static float cosineHemispherePDF(const Vec3 wi)
	{
		return std::cos(SphericalCoordinates::sphericalTheta(wi)) / M_PI;
	}
	static Vec3 uniformSampleSphere(float r1, float r2)
	{
		float a1 = std::acos(1.f - 2.f * r1);
		float a2 = 2.f * M_PI * r2;
		return SphericalCoordinates::sphericalToWorld(a1, a2);
	}
	static float uniformSpherePDF(const Vec3& wi)
	{
		return (1.f / (4.f * M_PI));
	}
};