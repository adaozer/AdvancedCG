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
#include <atomic>

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom *samplers;
	std::thread **threads;
	int numProcs;
	std::atomic<int> tiles;
	int tileSize = 16;

	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new GaussianFilter(1.0f, 1));
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
			float r2 = (samp - shadingData.x).lengthSq();
			Vec3 lightNormal = lightSample->normal(shadingData, wi);
			float cosLight = fabsf(Dot(-wi, lightNormal));
			return bsdf * emmittedColour * (geometryTerm * cosLight) / (r2 * pdf * pmf);
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

Colour pathTraceRecursive(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler) {
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX) {
			if (shadingData.bsdf->isLight()) {
				return (depth == 0) ? shadingData.bsdf->emit(shadingData, shadingData.wo) : Colour(0.f, 0.f, 0.f);

			}
			Colour direct = pathThroughput * computeDirect(shadingData, sampler);
			if (depth > 10) return direct;
			Colour indirect;
			float pdf;	
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, indirect, pdf);
			r.init(shadingData.x + (wi * EPSILON), wi);
			float cosine = std::max(Dot(wi, shadingData.sNormal), 0.f);
			pathThroughput = pathThroughput * indirect * cosine / pdf;

			if (depth > 3) {
				float rrp = std::min(pathThroughput.Lum(), 1.f);
				if (sampler->next() < rrp) {
					pathThroughput = pathThroughput / rrp;
					return direct + pathTraceRecursive(r, pathThroughput, depth + 1, sampler);
				}
				else return direct;
			}
			return direct + pathTraceRecursive(r, pathThroughput, depth + 1, sampler);
		}
		return scene->background->evaluate(r.dir) * pathThroughput;
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

	void renderTile(int threadID)
	{
		int totalTilesX = (film->width + tileSize - 1) / tileSize;
		int totalTilesY = (film->height + tileSize - 1) / tileSize;
		int totalTiles = totalTilesX * totalTilesY;

		while (true)
		{
			int tileID = tiles.fetch_add(1);
			if (tileID >= totalTiles) break;

			unsigned int tileX = tileID % totalTilesX;
			unsigned int tileY = tileID / totalTilesX;

			unsigned int startX = tileX * tileSize;
			unsigned int startY = tileY * tileSize;
			unsigned int endX = std::min(startX + (unsigned int)tileSize, film->width);
			unsigned int endY = std::min(startY + (unsigned int)tileSize, film->height);

			render2(startY, endY, startX, endX, threadID);
		}
	}

	void render()
	{
		film->incrementSPP();
		tiles.store(0);
		for (unsigned int i = 0; i < numProcs; i++) {
			threads[i] = new std::thread(&RayTracer::renderTile, this, i);
		}
		for (unsigned int i = 0; i < numProcs; i++) {
			threads[i]->join();
			delete threads[i];
		}

		for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				unsigned char r, g, b;
				film->tonemap(x, y, r, g, b);
				canvas->draw(x, y, r, g, b);
			}
		}
	}
	void render2(unsigned int startY = 0, unsigned int endY = 0, 
		unsigned int startX = 0, unsigned int endX = 0, unsigned int threadID = 0)
	{
		if (endY == 0) endY = film->height;
		if (endX == 0) endX = film->width;
		for (unsigned int y = startY; y < endY; y++)
		{
			for (unsigned int x = startX; x < endX; x++)
			{
				float px = x + samplers[threadID].next();
				float py = y + samplers[threadID].next();
				Ray ray = scene->camera.generateRay(px, py);
				Colour throughput(1.0f, 1.0f, 1.0f);
				Colour col = pathTraceRecursive(ray, throughput, 0, &samplers[threadID]);
				//Colour col = viewNormals(ray);
				//Colour col = albedo(ray);
				float lum = col.Lum();
				if (lum > 10.0f) col = col * (10.0f / lum);
				film->splat(px, py, col);
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