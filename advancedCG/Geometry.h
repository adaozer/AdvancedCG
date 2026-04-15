#pragma once

#include "Core.h"
#include "Sampling.h"

class Ray
{
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	Ray()
	{
	}
	Ray(Vec3 _o, Vec3 _d)
	{
		init(_o, _d);
	}
	void init(Vec3 _o, Vec3 _d)
	{
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}
};

class Plane
{
public:
	Vec3 n;
	float d;
	void init(Vec3& _n, float _d)
	{
		n = _n;
		d = _d;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		t = -(Dot(n, r.o) + d) / (Dot(n, r.dir));
		if (t >= 0) return true;
		return false;
	}
};

#define EPSILON 0.000001f

class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal
	float area; // Triangle area
	float d; // For ray triangle if needed
	unsigned int materialIndex;
	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e1 = vertices[2].p - vertices[1].p;
		e2 = vertices[0].p - vertices[2].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p);
	}
	Vec3 centre() const
	{
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}
	// Add code here
	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
		float denom = Dot(n, r.dir);
		if (denom == 0) { return false; }
		t = (d - Dot(n, r.o)) / denom;
		if (t < 0) { return false; }
		Vec3 p = r.at(t);
		float invArea = 1.0f / Dot(e1.cross(e2), n);
		u = Dot(e1.cross(p - vertices[1].p), n) * invArea;
		if (u < 0 || u > 1.0f) { return false; }
		v = Dot(e2.cross(p - vertices[2].p), n) * invArea;
		if (v < 0 || (u + v) > 1.0f) { return false; }
		return true;
	}

	bool mollerTrumbore(const Ray& r, float& t) const
	{
		Vec3 E1 = vertices[1].p - vertices[0].p;
		Vec3 E2 = vertices[2].p - vertices[0].p;
		Vec3 P = r.dir.cross(E2);
		float det = E1.dot(P);
		if (fabsf(det) < EPSILON) return false;
		float invDet = 1.0f / det;
		Vec3 T = r.o - vertices[0].p;
		float u = T.dot(P) * invDet;
		if (u < 0.0f || u > 1.0f) return false;
		Vec3 Q = T.cross(E1);
		float v = r.dir.dot(Q) * invDet;
		if (v < 0.0f || (u + v) > 1.0f) return false;
		t = E2.dot(Q) * invDet;
		return t > 0.0f;
	}

	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const
	{
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}
	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		pdf = 1.f / area;
		float r1 = sampler->next();
		float r2 = sampler->next();
		float sq = sqrtf(r1);
		float alpha = 1.f - sq;
		float beta = r2 * sq;
		float gamma = 1.f - (alpha + beta);
		return vertices[0].p * alpha + vertices[1].p * beta + vertices[2].p * gamma;
	}

	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

class AABB
{
public:
	Vec3 max;
	Vec3 min;
	AABB()
	{
		reset();
	}
	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}
	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
	}
	// Add code here
	bool rayAABB(const Ray& r, float& t)
	{
		Vec3 Tmin = (min - r.o) * r.invDir;
		Vec3 Tmax = (max - r.o) * r.invDir;
		Vec3 Tentry = Min(Tmin, Tmax);
		Vec3 Texit = Max(Tmin, Tmax);
		float tentry = std::max(Tentry.x, std::max(Tentry.y, Tentry.z));
		float texit = std::min(Texit.x, std::min(Texit.y, Texit.z));
		t = std::min(tentry, texit);
		return (tentry <= texit && texit > 0);
	}

	// Add code here
	bool rayAABB(const Ray& r)
	{
		Vec3 Tmin = (min - r.o) * r.invDir;
		Vec3 Tmax = (max - r.o) * r.invDir;
		Vec3 Tentry = Min(Tmin, Tmax);
		Vec3 Texit = Max(Tmin, Tmax);
		float tentry = std::max(Tentry.x, std::max(Tentry.y, Tentry.z));
		float texit = std::min(Texit.x, std::min(Texit.y, Texit.z));
		return (tentry <= texit && texit > 0);
	}

	// Add code here
	float area()
	{
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}
};

class Sphere
{
public:
	Vec3 centre;
	float radius;
	void init(Vec3& _centre, float _radius)
	{
		centre = _centre;
		radius = _radius;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		return false;
	}
};

struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32

class BVHNode
{
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	unsigned int offset;
	unsigned char num;

	BVHNode()
	{
		r = NULL;
		l = NULL;
		offset = 0;
		num = 0;
	}

	bool isLeaf() { return num > 0; }

	void buildRecursive(std::vector<Triangle>& triangles, std::vector<unsigned int>& indices, int start, int end)
	{
		bounds.reset();
		for (int i = start; i < end; i++)
			for (int j = 0; j < 3; j++)
				bounds.extend(triangles[indices[i]].vertices[j].p);

		int count = end - start;

		if (count <= MAXNODE_TRIANGLES)
		{
			offset = (unsigned int)start;
			num = (unsigned char)count;
			return;
		}

		AABB centroidBounds;
		centroidBounds.reset();
		for (int i = start; i < end; i++)
			centroidBounds.extend(triangles[indices[i]].centre());

		int bestAxis = -1;
		int bestBin = -1;
		float bestCost = FLT_MAX;
		float parentArea = bounds.area();

		for (int axis = 0; axis < 3; axis++)
		{
			float axisMin = centroidBounds.min.coords[axis];
			float axisMax = centroidBounds.max.coords[axis];
			if (axisMax - axisMin < EPSILON) continue;

			struct Bin { AABB bounds; int count; };
			Bin bins[BUILD_BINS];
			for (int b = 0; b < BUILD_BINS; b++) { bins[b].count = 0; bins[b].bounds.reset(); }

			for (int i = start; i < end; i++)
			{
				float cv = triangles[indices[i]].centre().coords[axis];
				int b = (int)(BUILD_BINS * (cv - axisMin) / (axisMax - axisMin));
				b = std::min(b, BUILD_BINS - 1);
				bins[b].count++;
				for (int j = 0; j < 3; j++)
					bins[b].bounds.extend(triangles[indices[i]].vertices[j].p);
			}

			for (int split = 1; split < BUILD_BINS; split++)
			{
				AABB leftBounds, rightBounds;
				leftBounds.reset(); rightBounds.reset();
				int leftN = 0, rightN = 0;

				for (int b = 0; b < split; b++)
					if (bins[b].count > 0)
					{
						leftBounds.extend(bins[b].bounds.min);
						leftBounds.extend(bins[b].bounds.max);
						leftN += bins[b].count;
					}
				for (int b = split; b < BUILD_BINS; b++)
					if (bins[b].count > 0)
					{
						rightBounds.extend(bins[b].bounds.min);
						rightBounds.extend(bins[b].bounds.max);
						rightN += bins[b].count;
					}

				if (leftN == 0 || rightN == 0) continue;

				float cost = TRAVERSE_COST + ((float)leftN * leftBounds.area() + (float)rightN * rightBounds.area()) / parentArea * TRIANGLE_COST;
				if (cost < bestCost)
				{
					bestCost = cost;
					bestAxis = axis;
					bestBin = split;
				}
			}
		}

		if (bestAxis == -1 || bestCost >= (float)count * TRIANGLE_COST)
		{
			offset = (unsigned int)start;
			num = (unsigned char)count;
			return;
		}

		float axisMin = centroidBounds.min.coords[bestAxis];
		float axisMax = centroidBounds.max.coords[bestAxis];
		auto pivot = std::partition(indices.begin() + start, indices.begin() + end,
			[&](unsigned int idx)
			{
				float cv = triangles[idx].centre().coords[bestAxis];
				int b = (int)(BUILD_BINS * (cv - axisMin) / (axisMax - axisMin));
				b = std::min(b, BUILD_BINS - 1);
				return b < bestBin;
			});

		int mid = (int)(pivot - indices.begin());
		if (mid == start || mid == end)
		{
			offset = (unsigned int)start;
			num = (unsigned char)count;
			return;
		}

		l = new BVHNode();
		r = new BVHNode();
		l->buildRecursive(triangles, indices, start, mid);
		r->buildRecursive(triangles, indices, mid, end);
		num = 0;
	}

	void build(std::vector<Triangle>& inputTriangles)
	{
		if (inputTriangles.empty()) return;

		std::vector<unsigned int> indices((unsigned int)inputTriangles.size());
		for (unsigned int i = 0; i < (unsigned int)inputTriangles.size(); i++) indices[i] = i;

		buildRecursive(inputTriangles, indices, 0, (int)inputTriangles.size());

		std::vector<Triangle> reordered(inputTriangles.size());
		for (unsigned int i = 0; i < (unsigned int)inputTriangles.size(); i++)
			reordered[i] = inputTriangles[indices[i]];
		inputTriangles = reordered;
	}

	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		if (!bounds.rayAABB(ray)) return;

		if (isLeaf())
		{
			for (unsigned int i = offset; i < offset + num; i++)
			{
				float t, u, v;
				if (triangles[i].rayIntersect(ray, t, u, v))
				{
					if (t < intersection.t)
					{
						intersection.t = t;
						intersection.ID = i;
						intersection.alpha = u;
						intersection.beta = v;
						intersection.gamma = 1.0f - (u + v);
					}
				}
			}
		}
		else
		{
			l->traverse(ray, triangles, intersection);
			r->traverse(ray, triangles, intersection);
		}
	}

	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, intersection);
		return intersection;
	}

	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT)
	{
		if (!bounds.rayAABB(ray)) return true;

		if (isLeaf())
		{
			for (unsigned int i = offset; i < offset + num; i++)
			{
				float t;
				if (triangles[i].mollerTrumbore(ray, t))
					if (t < maxT) return false;
			}
			return true;
		}
		else
		{
			if (!l->traverseVisible(ray, triangles, maxT)) return false;
			return r->traverseVisible(ray, triangles, maxT);
		}
	}
};
