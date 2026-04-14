#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom *samplers;
	std::thread **threads;
	int numProcs;
	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread*[numProcs];
		samplers = new MTRandom[numProcs];
		clear();
	}
	void clear()
	{
		film->clear();
	}
	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		float pmf;
		Light* lightSample = scene->sampleLight(sampler, pmf);
		float pdf;
		Colour emmittedColour;
		Vec3 samp = lightSample->sample(shadingData, sampler, emmittedColour, pdf);

		if (lightSample->isArea()) {
			Vec3 wi = samp - shadingData.x;
			wi = wi.normalize();
			float geometryTerm = Dot(wi, shadingData.sNormal);
			if (geometryTerm <= 0.f) {
				return Colour(0.f, 0.f, 0.f);
			}
			if (scene->visible(shadingData.x, samp)) {
			Colour bsdf = shadingData.bsdf->evaluate(shadingData, wi);
			return bsdf * emmittedColour * (geometryTerm / (pdf * pmf));
			}
			return Colour(0.0f, 0.0f, 0.0f);
		}

		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		Vec3 wi = samp;
		float cosTheta = Dot(wi, shadingData.sNormal);
		if (cosTheta <= 0.f) {
			return Colour(0.f, 0.f, 0.f);
		}
		if (scene->visible(shadingData.x, shadingData.x + wi * 10000000.f)) {
			Colour bsdf = shadingData.bsdf->evaluate(shadingData, wi);
			return bsdf * emmittedColour * (cosTheta / (pdf * pmf));		
		}
		return Colour(0.f, 0.f, 0.f);
		
	}
	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		if (shadingData.t == FLT_MAX)
			return pathThroughput * scene->background->evaluate(r.dir);

		if (shadingData.bsdf->isLight())
		{
			if (depth == 0)
				return pathThroughput * shadingData.bsdf->emit(shadingData, shadingData.wo);
			return Colour(0.0f, 0.0f, 0.0f);
		}

		Colour L = pathThroughput * computeDirect(shadingData, sampler);

		if (depth > 3)
		{
			float q = std::max({ pathThroughput.r, pathThroughput.g, pathThroughput.b });
			q = std::min(q, 0.95f);
			if (sampler->next() > q)
				return L;
			pathThroughput = pathThroughput / q;
		}

		float pdf;
		Colour bsdfColour;
		Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdfColour, pdf);

		if (pdf <= 0.0f)
			return L;

		float cosTheta = fabsf(Dot(wi, shadingData.sNormal));
		pathThroughput = pathThroughput * bsdfColour * (cosTheta / pdf);

		Ray nextRay(shadingData.x + wi * EPSILON, wi);
		return L + pathTrace(nextRay, pathThroughput, depth + 1, sampler);
	}
	Colour direct(Ray& r, Sampler* sampler)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return computeDirect(shadingData, sampler);
		}
		return scene->background->evaluate(r.dir);
	}

	Colour albedo(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return scene->background->evaluate(r.dir);
	}
	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	void render()
	{
		film->incrementSPP();
		unsigned int rowsPerThread = film->height / numProcs;
		for (unsigned int i = 0; i < numProcs; i++) {
			unsigned int startY = i * rowsPerThread;
			unsigned int endY = (i == numProcs - 1) ? film->height: startY + rowsPerThread;
			threads[i] = new std::thread(&RayTracer::render2, this, startY, endY);
		}
		for (unsigned int i = 0; i < numProcs; i++) {
			threads[i]->join();
			delete threads[i];
		}
	}
	void render2(unsigned int startY = 0, unsigned int endY = 0)
	{
		if (endY == 0) endY = film->height;

		for (unsigned int y = startY; y < endY; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				float px = x + 0.5f;
				float py = y + 0.5f;
				Ray ray = scene->camera.generateRay(px, py);
				Colour throughput(1.0f, 1.0f, 1.0f);
				Colour col = pathTrace(ray, throughput, 0, &samplers[0]);
				//Colour col = viewNormals(ray);
				//Colour col = albedo(ray);
				film->splat(px, py, col);
				unsigned char r = (unsigned char)(col.r * 255);
				unsigned char g = (unsigned char)(col.g * 255);
				unsigned char b = (unsigned char)(col.b * 255);
				film->tonemap(x, y, r, g, b);
				canvas->draw(x, y, r, g, b);
			}
		}
	}
	int getSPP()
	{
		return film->SPP;
	}
	void saveHDR(std::string filename)
	{
		film->save(filename);
	}
	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}
};