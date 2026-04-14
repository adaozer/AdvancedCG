#pragma once

#include "Core.h"
#include "Geometry.h"
#include "Materials.h"
#include "Sampling.h"

#pragma warning( disable : 4244)

class SceneBounds
{
public:
	Vec3 sceneCentre;
	float sceneRadius;
};

class Light
{
public:
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf) = 0;
	virtual Colour evaluate(const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isArea() = 0;
	virtual Vec3 normal(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float totalIntegratedPower() = 0;
	virtual Vec3 samplePositionFromLight(Sampler* sampler, float& pdf) = 0;
	virtual Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf) = 0;
};

class AreaLight : public Light
{
public:
	Triangle* triangle = NULL;
	Colour emission;
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf)
	{
		emittedColour = emission;
		return triangle->sample(sampler, pdf);
	}
	Colour evaluate(const Vec3& wi)
	{
		if (Dot(wi, triangle->gNormal()) < 0)
		{
			return emission;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 1.0f / triangle->area;
	}
	bool isArea()
	{
		return true;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return triangle->gNormal();
	}
	float totalIntegratedPower()
	{
		return (triangle->area * emission.Lum());
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		return triangle->sample(sampler, pdf);
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Add code to sample a direction from the light
		Vec3 wi = Vec3(0, 0, 1);
		pdf = 1.0f;
		Frame frame;
		frame.fromVector(triangle->gNormal());
		return frame.toWorld(wi);
	}
};

class BackgroundColour : public Light
{
public:
	Colour emission;
	BackgroundColour(Colour _emission)
	{
		emission = _emission;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = emission;
		return wi;
	}
	Colour evaluate(const Vec3& wi)
	{
		return emission;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return SamplingDistributions::uniformSpherePDF(wi);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		return emission.Lum() * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 4 * M_PI * use<SceneBounds>().sceneRadius * use<SceneBounds>().sceneRadius;
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
	}
};

class EnvironmentMap : public Light
{
public:
	Texture* env;
	std::vector<float> marginalCDF;      // size H+1
	std::vector<float> conditionalCDF;   // size H*(W+1), row-major
	std::vector<float> rowWeights;       // size H, unnormalised row sums * sinTheta
	float totalWeight;

	EnvironmentMap(Texture* _env)
	{
		env = _env;
		build();
	}

	void build()
	{
		int W = env->width, H = env->height;
		conditionalCDF.resize(H * (W + 1));
		marginalCDF.resize(H + 1);
		rowWeights.resize(H);

		for (int j = 0; j < H; j++)
		{
			float sinTheta = sinf(M_PI * (j + 0.5f) / H);
			float rowSum = 0.0f;
			conditionalCDF[j * (W + 1)] = 0.0f;
			for (int i = 0; i < W; i++)
			{
				rowSum += env->texels[j * W + i].Lum();
				conditionalCDF[j * (W + 1) + i + 1] = rowSum;
			}
			// Normalise conditional CDF for this row (sinTheta cancels within a row)
			if (rowSum > 0.0f)
				for (int i = 1; i <= W; i++)
					conditionalCDF[j * (W + 1) + i] /= rowSum;
			else
				for (int i = 1; i <= W; i++)
					conditionalCDF[j * (W + 1) + i] = (float)i / W;

			rowWeights[j] = rowSum * sinTheta;
		}

		// Build marginal CDF over rows (weighted by sinTheta for spherical Jacobian)
		marginalCDF[0] = 0.0f;
		for (int j = 0; j < H; j++)
			marginalCDF[j + 1] = marginalCDF[j] + rowWeights[j];
		totalWeight = marginalCDF[H];
		if (totalWeight > 0.0f)
			for (int j = 1; j <= H; j++)
				marginalCDF[j] /= totalWeight;
		else
			for (int j = 1; j <= H; j++)
				marginalCDF[j] = (float)j / H;
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		int W = env->width, H = env->height;
		float r1 = sampler->next();
		float r2 = sampler->next();

		// Invert marginal CDF to pick a row
		int j = (int)(std::lower_bound(marginalCDF.begin(), marginalCDF.end(), r1) - marginalCDF.begin()) - 1;
		j = std::max(0, std::min(H - 1, j));

		// Invert conditional CDF to pick a column within that row
		float* rowCDF = conditionalCDF.data() + j * (W + 1);
		int i = (int)(std::lower_bound(rowCDF, rowCDF + W + 1, r2) - rowCDF) - 1;
		i = std::max(0, std::min(W - 1, i));

		// Convert pixel centre to spherical direction
		float u = (i + 0.5f) / W;
		float v = (j + 0.5f) / H;
		float phi = 2.0f * M_PI * u;
		float theta = M_PI * v;
		float sinTheta = sinf(theta);

		Vec3 wi;
		wi.x = sinTheta * cosf(phi);
		wi.y = cosf(theta);
		wi.z = sinTheta * sinf(phi);

		reflectedColour = evaluate(wi);
		pdf = PDF(shadingData, wi);
		return wi;
	}

	Colour evaluate(const Vec3& wi)
	{
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(std::max(-1.0f, std::min(1.0f, wi.y))) / M_PI;
		return env->sample(u, v);
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		if (totalWeight <= 0.0f) return SamplingDistributions::uniformSpherePDF(wi);

		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(std::max(-1.0f, std::min(1.0f, wi.y))) / M_PI;

		float sinTheta = sinf(M_PI * v);
		if (sinTheta < 1e-6f) return 0.0f;

		int i = std::max(0, std::min(env->width - 1, (int)(u * env->width)));
		int j = std::max(0, std::min(env->height - 1, (int)(v * env->height)));
		float lum = env->texels[j * env->width + i].Lum();

		// p(w) = L(u,v) * W * H / (totalWeight * 2*pi^2)
		// Derived from: p(u,v) = L * sinTheta * W * H / totalWeight
		//               p(w)   = p(u,v) / (2*pi^2 * sinTheta)  [Jacobian sphere->UV]
		return (lum * env->width * env->height) / (totalWeight * 2.0f * M_PI * M_PI);
	}

	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		float total = 0;
		for (int i = 0; i < env->height; i++)
		{
			float st = sinf(((float)i / (float)env->height) * M_PI);
			for (int n = 0; n < env->width; n++)
			{
				total += (env->texels[(i * env->width) + n].Lum() * st);
			}
		}
		total = total / (float)(env->width * env->height);
		return total * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		// Samples a point on the bounding sphere of the scene. Feel free to improve this.
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 1.0f / (4 * M_PI * SQ(use<SceneBounds>().sceneRadius));
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		ShadingData dummy;
		Colour col;
		return sample(dummy, sampler, col, pdf);
	}
};