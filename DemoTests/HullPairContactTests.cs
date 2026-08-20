using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuUtilities.Memory;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using Xunit;

namespace DemoTests
{
    /// <summary>
    /// Shows that the hull versus hull collision tester can produce contact manifolds whose depths
    /// are NaN for pairs of identical extruded regular polygons at overlapping poses, while the
    /// normal and offsets stay finite. The failing manifolds also have 2 contacts where nearby
    /// working poses produce 4, both on the same extrusion cap.
    ///
    /// The collision batcher is driven directly, so no simulation, solver, inertia or integration
    /// is involved; the NaN is present in the tester's own output. An analytic Box pair given the
    /// identical placements never fails, which is what the control test demonstrates.
    ///
    /// These tests are expected to FAIL until the tester is fixed; the Box control passes.
    /// </summary>
    public static class HullPairContactTests
    {
        struct Callbacks : ICollisionCallbacks
        {
            public Buffer<ConvexContactManifold> Manifold;

            public bool AllowCollisionTesting(int pairId, int childA, int childB) => true;

            public void OnChildPairCompleted(int pairId, int childA, int childB, ref ConvexContactManifold manifold) { }

            public void OnPairCompleted<TManifold>(int pairId, ref TManifold manifold) where TManifold : unmanaged, IContactManifold<TManifold>
            {
                Manifold[0] = Unsafe.As<TManifold, ConvexContactManifold>(ref manifold);
            }
        }

        /// <summary>Builds the hull of a regular polygon with the given circumradius, extruded along Z.</summary>
        static ConvexHull CreateExtrudedPolygon(int sides, float radius, float depth, BufferPool pool)
        {
            pool.Take<Vector3>(sides * 2, out var points);
            var angleStep = MathF.Tau / sides;
            for (int i = 0; i < sides; ++i)
            {
                var angle = i * angleStep - MathF.PI / 2;
                var x = MathF.Cos(angle) * radius;
                var y = MathF.Sin(angle) * radius;
                points[i] = new Vector3(x, y, depth / 2);
                points[i + sides] = new Vector3(x, y, -depth / 2);
            }
            ConvexHullHelper.CreateShape(points.Slice(0, sides * 2), pool, out _, out var hull);
            pool.Return(ref points);
            return hull;
        }

        /// <summary>Runs one pair through the collision batcher and returns the manifold produced.</summary>
        static ConvexContactManifold ComputeManifold(Shapes shapes, CollisionTaskRegistry registry, BufferPool pool, TypedIndex shape, in RigidPose poseA, in RigidPose poseB)
        {
            pool.Take<ConvexContactManifold>(1, out var manifoldSlot);
            manifoldSlot[0] = default;
            var callbacks = new Callbacks { Manifold = manifoldSlot };
            var batcher = new CollisionBatcher<Callbacks>(pool, shapes, registry, 1 / 60f, callbacks);
            batcher.Add(shape, shape, poseB.Position - poseA.Position, poseA.Orientation, poseB.Orientation, 0.1f, new PairContinuation(0));
            batcher.Flush();
            var manifold = manifoldSlot[0];
            pool.Return(ref manifoldSlot);
            return manifold;
        }

        static bool AllDepthsFinite(in ConvexContactManifold manifold)
        {
            for (int i = 0; i < manifold.Count; ++i)
            {
                if (!float.IsFinite(Unsafe.Add(ref Unsafe.AsRef(in manifold.Contact0), i).Depth))
                    return false;
            }
            return true;
        }

        /// <summary>
        /// The minimal deterministic case: two identical extruded hexagons, axis-aligned, offset in
        /// both X and Y. Produces a 2-contact manifold with depth = NaN under a finite normal.
        /// With the Y offset removed, or with 5 or 7 sides at this same offset, the manifold is clean,
        /// so the failure is knife-edge sensitive to the pose; the sweep test below measures how common
        /// failing poses actually are.
        /// </summary>
        [Fact]
        public static void ExtrudedHexagonPairAtKnownOffsetHasFiniteContactDepths()
        {
            var pool = new BufferPool();
            var registry = DefaultTypes.CreateDefaultCollisionTaskRegistry();
            var shapes = new Shapes(pool, 8);

            var hull = shapes.Add(CreateExtrudedPolygon(6, 0.5f, 1f, pool));
            var manifold = ComputeManifold(shapes, registry, pool, hull, RigidPose.Identity, new Vector3(0.3445f, 0.076f, 0));

            Assert.True(AllDepthsFinite(manifold),
                $"Hexagon pair manifold has a non-finite depth. Count={manifold.Count}, normal={manifold.Normal}, " +
                $"depth0={(manifold.Count > 0 ? manifold.Contact0.Depth : 0)}");

            shapes.Dispose();
            pool.Clear();
        }

        /// <summary>
        /// Measures the size of the failure surface: seeded random overlapping placements per side
        /// count, offsets within one circumradius on each axis, each body given a random rotation
        /// about Z. On current main, every side count from 3 up fails for 15-40% of placements.
        /// </summary>
        [Fact]
        public static void ExtrudedPolygonPairsAtRandomOverlapsHaveFiniteContactDepths()
        {
            const float radius = 1f;
            const float depth = 4f;
            const int trialsPerSideCount = 100;

            var pool = new BufferPool();
            var registry = DefaultTypes.CreateDefaultCollisionTaskRegistry();
            var report = "";

            foreach (var sides in (int[])[3, 4, 5, 6, 7, 8, 10])
            {
                var shapes = new Shapes(pool, 8);
                var hull = shapes.Add(CreateExtrudedPolygon(sides, radius, depth, pool));
                var random = new Random(1);
                int failures = 0;

                for (int trial = 0; trial < trialsPerSideCount; ++trial)
                {
                    var poseA = new RigidPose(default, Quaternion.CreateFromAxisAngle(Vector3.UnitZ, random.NextSingle() * MathF.Tau));
                    var poseB = new RigidPose(
                        new Vector3((random.NextSingle() * 2f - 1f) * radius, (random.NextSingle() * 2f - 1f) * radius, 0),
                        Quaternion.CreateFromAxisAngle(Vector3.UnitZ, random.NextSingle() * MathF.Tau));

                    if (!AllDepthsFinite(ComputeManifold(shapes, registry, pool, hull, poseA, poseB)))
                        ++failures;
                }

                if (failures > 0)
                    report += $"sides={sides}: {failures}/{trialsPerSideCount} placements produced a non-finite depth. ";

                shapes.Dispose();
            }

            Assert.True(report.Length == 0, report);
            pool.Clear();
        }

        /// <summary>
        /// The control: an analytic Box pair given the identical placements (same seed, same offsets,
        /// same rotations) never produces a non-finite depth, so deep overlap by itself is handled
        /// fine and the hull tester owns the failures above.
        /// </summary>
        [Fact]
        public static void BoxPairsAtTheSameRandomOverlapsHaveFiniteContactDepths()
        {
            const float radius = 1f;
            const int trials = 100;

            var pool = new BufferPool();
            var registry = DefaultTypes.CreateDefaultCollisionTaskRegistry();
            var shapes = new Shapes(pool, 8);

            var side = radius * MathF.Sqrt(2f);
            var box = shapes.Add(new Box(side, side, 4f));
            var random = new Random(1);
            int failures = 0;

            for (int trial = 0; trial < trials; ++trial)
            {
                var poseA = new RigidPose(default, Quaternion.CreateFromAxisAngle(Vector3.UnitZ, random.NextSingle() * MathF.Tau));
                var poseB = new RigidPose(
                    new Vector3((random.NextSingle() * 2f - 1f) * radius, (random.NextSingle() * 2f - 1f) * radius, 0),
                    Quaternion.CreateFromAxisAngle(Vector3.UnitZ, random.NextSingle() * MathF.Tau));

                if (!AllDepthsFinite(ComputeManifold(shapes, registry, pool, box, poseA, poseB)))
                    ++failures;
            }

            Assert.True(failures == 0, $"{failures}/{trials} box placements produced a non-finite depth.");

            shapes.Dispose();
            pool.Clear();
        }
    }
}
