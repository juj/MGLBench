#include <MathGeoLib.h>
#include <Algorithm/GJK.h>
#include <Algorithm/SAT.h>
#include <stdio.h>

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#endif

int nextTest = 0;

double totalTime = 0.f;

void AABBTest()
{
	LCG rng(123);
	const int N = 10000;
	AABB aabb[N];
	for(int i = 0; i < N; ++i)
	{
		vec x0 = vec::RandomBox(rng, -10.f, 10.f, -10.f, 10.f, -10.f, 10.f);
		vec x1 = vec::RandomBox(rng, -10.f, 10.f, -10.f, 10.f, -10.f, 10.f);
		vec m0 = x0.Min(x1);
		vec m1 = x0.Max(x1);
		aabb[i] = AABB(m0, m1);
	}
	int numCollisions = 0;

	tick_t t0 = Clock::Tick();
	for(int i = 1; i < N; ++i)
		for(int j = 0; j < i; ++j)
		{
			if (aabb[i].Intersects(aabb[j]))
				++numCollisions;
		}
	tick_t t1 = Clock::Tick();
	totalTime += Clock::TimespanToMillisecondsD(t0, t1);

	const int expectedNumCollisions = 14712812;
	if (numCollisions != expectedNumCollisions)
		printf("AABB-AABB test failed! Got result: %d colliding pairs, expected %d colliding pairs!\n", numCollisions, expectedNumCollisions);

	printf("AABB-AABB intersection: %f msecs (%d collisions)\n", Clock::TimespanToMillisecondsD(t0, t1), numCollisions);
}

void OBBTest()
{
	LCG rng(123);
	const int N = 5000;
	OBB obb[N];
	for(int i = 0; i < N; ++i)
	{
		vec x0 = vec::RandomBox(rng, -10.f, 10.f, -10.f, 10.f, -10.f, 10.f);
		vec x1 = vec::RandomBox(rng, -10.f, 10.f, -10.f, 10.f, -10.f, 10.f);
		vec m0 = x0.Min(x1);
		vec m1 = x0.Max(x1);
		AABB a(m0, m1);
		Quat q = Quat::RandomRotation(rng);
		obb[i] = a.Transform(q);
	}
	int numCollisions = 0;

	tick_t t0 = Clock::Tick();
	for(int i = 1; i < N; ++i)
		for(int j = 0; j < i; ++j)
		{
			if (obb[i].Intersects(obb[j]))
				++numCollisions;
		}
	tick_t t1 = Clock::Tick();
	totalTime += Clock::TimespanToMillisecondsD(t0, t1);

	const int expectedNumCollisions = 4879382;
	if (numCollisions != expectedNumCollisions)
		printf("OBB-OBB test failed! Got result: %d colliding pairs, expected %d colliding pairs!\n", numCollisions, expectedNumCollisions);

	printf("OBB-OBB intersection: %f msecs (%d collisions)\n", Clock::TimespanToMillisecondsD(t0, t1), numCollisions);
}

void SphereTest()
{
	LCG rng(123);
	const int N = 10000;
	Sphere s[N];
	for(int i = 0; i < N; ++i)
		s[i] = Sphere(vec::RandomBox(rng, -10.f, 10.f, -10.f, 10.f, -10.f, 10.f), rng.Float(0.1f, 10.f));
	int numCollisions = 0;

	tick_t t0 = Clock::Tick();
	for(int i = 1; i < N; ++i)
		for(int j = 0; j < i; ++j)
		{
			if (s[i].Intersects(s[j]))
				++numCollisions;
		}
	tick_t t1 = Clock::Tick();
	totalTime += Clock::TimespanToMillisecondsD(t0, t1);

	const int expectedNumCollisions = 15821386;
	if (numCollisions != expectedNumCollisions)
		printf("Sphere-Sphere test failed! Got result: %d colliding pairs, expected %d colliding pairs!\n", numCollisions, expectedNumCollisions);

	printf("Sphere-Sphere intersection: %f msecs (%d collisions)\n", Clock::TimespanToMillisecondsD(t0, t1), numCollisions);
}

void CapsuleTest()
{
	LCG rng(123);
	const int N = 3000;
	Capsule c[N];
	for(int i = 0; i < N; ++i)
		c[i] = Capsule(vec::RandomBox(rng, -10.f, 10.f, -10.f, 10.f, -10.f, 10.f), vec::RandomBox(rng, -10.f, 10.f, -10.f, 10.f, -10.f, 10.f), rng.Float(0.1f, 10.f));
	int numCollisions = 0;

	tick_t t0 = Clock::Tick();
	for(int i = 1; i < N; ++i)
		for(int j = 0; j < i; ++j)
		{
			if (c[i].Intersects(c[j]))
				++numCollisions;
		}
	tick_t t1 = Clock::Tick();
	totalTime += Clock::TimespanToMillisecondsD(t0, t1);

	const int expectedNumCollisions = 3358668;
	if (numCollisions != expectedNumCollisions)
		printf("Capsule-Capsule test failed! Got result: %d colliding pairs, expected %d colliding pairs!\n", numCollisions, expectedNumCollisions);

	printf("Capsule-Capsule intersection: %f msecs (%d collisions)\n", Clock::TimespanToMillisecondsD(t0, t1), numCollisions);
}

void PolyhedronTest()
{
	LCG rng(123);
	const int N = 250;
	Polyhedron p[N];
	for(int i = 0; i < N; ++i)
	{
		const int k = 5;
		vec verts[k];
		for(int j = 0; j < k; ++j)
			verts[j] = vec::RandomBox(rng, -10.f, 10.f, -10.f, 10.f, -10.f, 10.f);
		p[i] = Polyhedron::ConvexHull(verts, k);
		if (p[i].NumVertices() < 4 || p[i].NumFaces() < 4 || !p[i].EulerFormulaHolds() || !p[i].FaceIndicesValid() || p[i].IsNull() || !p[i].IsClosed() || !p[i].IsConvex())
			--i;
	}
	int numCollisions = 0;

	tick_t t0 = Clock::Tick();
	for(int i = 1; i < N; ++i)
		for(int j = 0; j < i; ++j)
		{
			if (p[i].Intersects(p[j]))
				++numCollisions;
			if (GJKIntersect(p[i], p[j]))
				++numCollisions;
//			if (SATIntersect(p[i], p[j]))
//				++numCollisions;
		}
	tick_t t1 = Clock::Tick();
	totalTime += Clock::TimespanToMillisecondsD(t0, t1);

	const int expectedNumCollisions = 51728;
	if (numCollisions != expectedNumCollisions)
		printf("Polyhedron-Polyhedron test failed! Got result: %d colliding pairs, expected %d colliding pairs!\n", numCollisions, expectedNumCollisions);

	printf("Polyhedron-Polyhedron intersection: %f msecs (%d collisions)\n", Clock::TimespanToMillisecondsD(t0, t1), numCollisions);
}

void TriangleTriangleTest()
{
	LCG rng(123);
	const int N = 2000;
	Triangle t[N];
	for(int i = 0; i < N; ++i)
	{
		t[i].a = vec::RandomBox(rng, -10.f, 10.f, -10.f, 10.f, -10.f, 10.f);
		t[i].b = vec::RandomBox(rng, -10.f, 10.f, -10.f, 10.f, -10.f, 10.f);
		t[i].c = vec::RandomBox(rng, -10.f, 10.f, -10.f, 10.f, -10.f, 10.f);
	}
	int numCollisions = 0;

	tick_t t0 = Clock::Tick();
	for(int i = 1; i < N; ++i)
		for(int j = 0; j < i; ++j)
		{
			if (t[i].Intersects(t[j]))
				++numCollisions;
		}
	tick_t t1 = Clock::Tick();
	totalTime += Clock::TimespanToMillisecondsD(t0, t1);

	const int expectedNumCollisions = 568430;
	if (numCollisions != expectedNumCollisions)
		printf("Triangle-Triangle test failed! Got result: %d colliding pairs, expected %d colliding pairs!\n", numCollisions, expectedNumCollisions);

	printf("Triangle-Triangle intersection: %f msecs (%d collisions)\n", Clock::TimespanToMillisecondsD(t0, t1), numCollisions);
}

void RayTriangleTest()
{
	LCG rng(123);
	const int N = 5000;
	const int M = 3000;
	Triangle t[N];
	for(int i = 0; i < N; ++i)
	{
		t[i].a = vec::RandomBox(rng, -10.f, 10.f, -10.f, 10.f, -10.f, 10.f);
		t[i].b = vec::RandomBox(rng, -10.f, 10.f, -10.f, 10.f, -10.f, 10.f);
		t[i].c = vec::RandomBox(rng, -10.f, 10.f, -10.f, 10.f, -10.f, 10.f);
	}
	Ray r[M];
	for(int i = 0; i < M; ++i)
	{
		r[i].pos = vec::RandomBox(rng, -10.f, 10.f, -10.f, 10.f, -10.f, 10.f);
		r[i].dir = vec::RandomDir(rng);
	}
	int numCollisions = 0;

	tick_t t0 = Clock::Tick();
	for(int i = 1; i < N; ++i)
		for(int j = 0; j < M; ++j)
		{
			if (r[i].Intersects(t[j]))
				++numCollisions;
		}
	tick_t t1 = Clock::Tick();
	totalTime += Clock::TimespanToMillisecondsD(t0, t1);

	const int expectedNumCollisions = 596509;
	if (numCollisions != expectedNumCollisions)
		printf("Ray-Triangle test failed! Got result: %d colliding pairs, expected %d colliding pairs!\n", numCollisions, expectedNumCollisions);

	printf("Ray-Triangle intersection: %f msecs (%d collisions)\n", Clock::TimespanToMillisecondsD(t0, t1), numCollisions);
}

void AABBTransformTest()
{
	LCG rng(123);
	const int N = 10000;
	AABB aabb[N];
	for(int i = 0; i < N; ++i)
	{
		vec x0 = vec::RandomBox(rng, -10.f, 10.f, -10.f, 10.f, -10.f, 10.f);
		vec x1 = vec::RandomBox(rng, -10.f, 10.f, -10.f, 10.f, -10.f, 10.f);
		vec m0 = x0.Min(x1);
		vec m1 = x0.Max(x1);
		aabb[i] = AABB(m0, m1);
	}

	int numDegenerate = 0;

	const int K = 1000;
	tick_t t0 = Clock::Tick();
	for(int i = 0; i < N; ++i)
	{
		Quat q = Quat::RandomRotation(rng);
		for(int j = 0; j < K; ++j)
		{
			aabb[i].TransformAsAABB(q);
		}
		numDegenerate += aabb[i].IsDegenerate();
	}
	tick_t t1 = Clock::Tick();
	totalTime += Clock::TimespanToMillisecondsD(t0, t1);

	const int expectedNumDegenerate = 0;
	if (numDegenerate != expectedNumDegenerate)
		printf("AABB transform test failed! Got result: %d degenerate, expected %d degenerate!\n", numDegenerate, expectedNumDegenerate);

	printf("AABB transform: %f msecs (%d degenerate)\n", Clock::TimespanToMillisecondsD(t0, t1), numDegenerate);
}

void OBBTransformTest()
{
	LCG rng(123);
	const int N = 2500;
	OBB obb[N];
	for(int i = 0; i < N; ++i)
	{
		vec x0 = vec::RandomBox(rng, -10.f, 10.f, -10.f, 10.f, -10.f, 10.f);
		vec x1 = vec::RandomBox(rng, -10.f, 10.f, -10.f, 10.f, -10.f, 10.f);
		vec m0 = x0.Min(x1);
		vec m1 = x0.Max(x1);
		obb[i] = AABB(m0, m1).ToOBB();
	}

	int numDegenerate = 0;

	const int K = 1000;
	tick_t t0 = Clock::Tick();
	for(int i = 0; i < N; ++i)
	{
		Quat q = Quat::RandomRotation(rng);
		for(int j = 0; j < K; ++j)
		{
			obb[i].Transform(q);
		}
		numDegenerate += obb[i].IsDegenerate();
	}
	tick_t t1 = Clock::Tick();
	totalTime += Clock::TimespanToMillisecondsD(t0, t1);

	const int expectedNumDegenerate = 0;
	if (numDegenerate != expectedNumDegenerate)
		printf("OBB transform test failed! Got result: %d degenerate, expected %d degenerate!\n", numDegenerate, expectedNumDegenerate);

	printf("OBB transform: %f msecs (%d degenerate)\n", Clock::TimespanToMillisecondsD(t0, t1), numDegenerate);
}

void CapsuleTransformTest()
{
	LCG rng(123);
	const int N = 10000;
	Capsule c[N];
	for(int i = 0; i < N; ++i)
		c[i] = Capsule(vec::RandomBox(rng, -10.f, 10.f, -10.f, 10.f, -10.f, 10.f), vec::RandomBox(rng, -10.f, 10.f, -10.f, 10.f, -10.f, 10.f), rng.Float(0.1f, 10.f));

	int numDegenerate = 0;

	const int K = 1000;
	tick_t t0 = Clock::Tick();
	for(int i = 0; i < N; ++i)
	{
		float3x4 m = Quat::RandomRotation(rng).ToFloat3x4();
		m.SetTranslatePart(rng.Float(-10.f, 10.f), rng.Float(-10.f, 10.f), rng.Float(-10.f, 10.f));
		for(int j = 0; j < K; ++j)
		{
			c[i].Transform(m);
		}
		numDegenerate += c[i].IsDegenerate();
	}
	tick_t t1 = Clock::Tick();
	totalTime += Clock::TimespanToMillisecondsD(t0, t1);

	const int expectedNumDegenerate = 0;
	if (numDegenerate != expectedNumDegenerate)
		printf("Capsule transform test failed! Got result: %d degenerate, expected %d degenerate!\n", numDegenerate, expectedNumDegenerate);

	printf("Capsule transform: %f msecs (%d degenerate)\n", Clock::TimespanToMillisecondsD(t0, t1), numDegenerate);
}

void PolyhedronTransformTest()
{
	LCG rng(123);
	const int N = 1000;
	Polyhedron p[N];
	for(int i = 0; i < N; ++i)
	{
		const int k = 300;
		vec verts[k];
		for(int j = 0; j < k; ++j)
			verts[j] = vec::RandomSphere(rng, POINT_VEC(0, 0, 0), 10.f);
		p[i] = Polyhedron::ConvexHull(verts, k);
		if (p[i].NumVertices() < 4 || p[i].NumFaces() < 4 || !p[i].EulerFormulaHolds() || !p[i].FaceIndicesValid() || p[i].IsNull() || !p[i].IsClosed())
			--i;
	}

	int numDegenerate = 0;

	const int K = 1000;
	tick_t t0 = Clock::Tick();
	for(int i = 0; i < N; ++i)
	{
		float3x4 m = Quat::RandomRotation(rng).ToFloat3x4();
		m.SetTranslatePart(rng.Float(-10.f, 10.f), rng.Float(-10.f, 10.f), rng.Float(-10.f, 10.f));
		for(int j = 0; j < K; ++j)
		{
			p[i].Transform(m);
		}
		if (p[i].NumVertices() < 4 || p[i].NumFaces() < 4 || !p[i].EulerFormulaHolds() || !p[i].FaceIndicesValid() || p[i].IsNull() || !p[i].IsClosed())
			++numDegenerate;
	}
	tick_t t1 = Clock::Tick();
	totalTime += Clock::TimespanToMillisecondsD(t0, t1);

	const int expectedNumDegenerate = 0;
	if (numDegenerate != expectedNumDegenerate)
		printf("Polyhedron transform test failed! Got result: %d degenerate, expected %d degenerate!\n", numDegenerate, expectedNumDegenerate);

	printf("Polyhedron transform: %f msecs (%d degenerate)\n", Clock::TimespanToMillisecondsD(t0, t1), numDegenerate);
}

void BoundingAABBTest()
{
	LCG rng(123);
	const int N = 100;
	float totalVolume = 0.f;
	tick_t t0 = Clock::Tick();
	for(int i = 0; i < N; ++i)
	{
		const int k = 3000;
		vec verts[k];
		for(int j = 0; j < k; ++j)
			verts[j] = vec::RandomSphere(rng, POINT_VEC(0, 0, 0), 10.f);
		AABB aabb = AABB::MinimalEnclosingAABB(verts, k);
		totalVolume += aabb.Volume();
	}
	tick_t t1 = Clock::Tick();
	totalTime += Clock::TimespanToMillisecondsD(t0, t1);
	printf("Bounding AABB: %f msecs (volume: %f)\n", Clock::TimespanToMillisecondsD(t0, t1), totalVolume);
}

void BoundingOBBTest()
{
	LCG rng(123);
	const int N = 20;
	float totalVolume = 0.f;
	tick_t t0 = Clock::Tick();
	for(int i = 0; i < N; ++i)
	{
		const int k = 1000;
		vec verts[k];
		for(int j = 0; j < k; ++j)
			verts[j] = vec::RandomSphere(rng, POINT_VEC(0, 0, 0), 10.f);
		OBB obb = OBB::OptimalEnclosingOBB(verts, k);
		totalVolume += obb.Volume();
	}
	tick_t t1 = Clock::Tick();
	totalTime += Clock::TimespanToMillisecondsD(t0, t1);
	printf("Bounding OBB: %f msecs (volume: %f)\n", Clock::TimespanToMillisecondsD(t0, t1), totalVolume);
}

void BoundingSphereTest()
{
	LCG rng(123);
	const int N = 100;
	float totalVolume = 0.f;
	tick_t t0 = Clock::Tick();
	for(int i = 0; i < N; ++i)
	{
		const int k = 3000;
		vec verts[k];
		for(int j = 0; j < k; ++j)
			verts[j] = vec::RandomSphere(rng, POINT_VEC(0, 0, 0), 10.f);
		Sphere s = Sphere::OptimalEnclosingSphere(verts, k);
		totalVolume += s.Volume();
	}
	tick_t t1 = Clock::Tick();
	totalTime += Clock::TimespanToMillisecondsD(t0, t1);
	printf("Bounding Sphere: %f msecs (volume: %f)\n", Clock::TimespanToMillisecondsD(t0, t1), totalVolume);
}

void ConvexHullTest()
{
	LCG rng(123);
	const int N = 50;
	float totalVolume = 0.f;
	tick_t t0 = Clock::Tick();
	for(int i = 0; i < N; ++i)
	{
		const int k = 3000;
		vec verts[k];
		for(int j = 0; j < k; ++j)
			verts[j] = vec::RandomSphere(rng, POINT_VEC(0, 0, 0), 10.f);
		Polyhedron p = Polyhedron::ConvexHull(verts, k);
		totalVolume += p.Volume();
	}
	tick_t t1 = Clock::Tick();
	totalTime += Clock::TimespanToMillisecondsD(t0, t1);
	printf("Convex hull: %f msecs (volume: %f)\n", Clock::TimespanToMillisecondsD(t0, t1), totalVolume);
}

void tick()
{
	switch(nextTest)
	{
	case 0: AABBTest(); break;
	case 1: OBBTest(); break;
	case 2: SphereTest(); break;
	case 3: CapsuleTest(); break;
	case 4: PolyhedronTest(); break;
	case 5: TriangleTriangleTest(); break;
	case 6: RayTriangleTest(); break;
	case 7: AABBTransformTest(); break;
	case 8: OBBTransformTest(); break;
	case 9: CapsuleTransformTest(); break;
	case 10: PolyhedronTransformTest(); break;
	case 11: BoundingAABBTest(); break;
	case 12: BoundingOBBTest(); break;
	case 13: BoundingSphereTest(); break;
	case 14: ConvexHullTest(); break;
	default:
		nextTest = -1;
		printf("\nAll tests done! Total accumulated time (lower is better): %f msecs\n", totalTime);
#ifdef __EMSCRIPTEN__
		emscripten_cancel_main_loop();
		exit(0);
#endif
		return;
	}

	++nextTest;
}

int main()
{
	printf("Running benchmarks (with resulting times, lower is better):\n\n");
#ifdef __EMSCRIPTEN__
	emscripten_set_main_loop(tick, 0, 0);
#else
	while(nextTest != -1)
		tick();
#endif
}
