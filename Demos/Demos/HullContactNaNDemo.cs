using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities.Memory;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Numerics;

namespace Demos.Demos;

/// <summary>
/// Reproduces a hull versus hull contact generation failure: two identical convex hulls, each a regular polygon extruded along Z,
/// overlapping with an offset in both X and Y, produce a contact manifold whose depths are NaN while the normal stays finite.
/// </summary>
/// <remarks>
/// <para>
/// Three pairs are spawned in the same overlapping relative pose, differing only in the polygon's side count. On the first timestep,
/// the 6-sided pair's manifold comes back with NaN depths and the NaN propagates into the body poses; the 5- and 7-sided pairs
/// resolve their overlap and settle normally. Which side counts fail is extremely sensitive to the exact vertex coordinates and the
/// relative pose - with random overlaps and rotations about Z, every side count from 3 up fails for a substantial fraction of poses.
/// </para>
/// <para>
/// In Release the demo keeps running and reports the NaN poses in the UI text. In Debug, CHECKMATH catches the value as the manifold
/// flows into constraint creation on the very first timestep, so running under a debugger breaks with the offending manifold in view.
/// </para>
/// </remarks>
public class HullContactNaNDemo : Demo
{
    (int Sides, BodyHandle A, BodyHandle B)[] pairs;

    static ConvexHull CreateExtrudedPolygon(int sides, float radius, float depth, BufferPool pool)
    {
        pool.Take<Vector3>(sides * 2, out var points);
        for (int i = 0; i < sides; ++i)
        {
            var angle = i * (MathF.Tau / sides) - MathF.PI / 2;
            var x = MathF.Cos(angle) * radius;
            var y = MathF.Sin(angle) * radius;
            points[i] = new Vector3(x, y, depth / 2);
            points[i + sides] = new Vector3(x, y, -depth / 2);
        }
        ConvexHullHelper.CreateShape(points.Slice(0, sides * 2), pool, out _, out var hull);
        pool.Return(ref points);
        return hull;
    }

    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Position = new Vector3(0, 3, 10);
        camera.Yaw = 0;
        camera.Pitch = 0;

        Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));

        Simulation.Statics.Add(new StaticDescription(new Vector3(0, -0.5f, 0), Simulation.Shapes.Add(new Box(30, 1, 30))));

        //The pair's relative pose is what matters: an offset in both X and Y over an identical, axis-aligned partner.
        //With offset.Y = 0, all three side counts are clean.
        var offset = new Vector3(0.3445f, 0.076f, 0);
        pairs = new (int, BodyHandle, BodyHandle)[3];

        foreach (var (index, sides) in new[] { (0, 5), (1, 6), (2, 7) })
        {
            var hull = CreateExtrudedPolygon(sides, 0.5f, 1f, BufferPool);
            var shape = Simulation.Shapes.Add(hull);
            var inertia = hull.ComputeInertia(1);

            var position = new Vector3((index - 1) * 4, 2, 0);
            var a = Simulation.Bodies.Add(BodyDescription.CreateDynamic(position, inertia, shape, 1e-2f));
            var b = Simulation.Bodies.Add(BodyDescription.CreateDynamic(position + offset, inertia, shape, 1e-2f));
            pairs[index] = (sides, a, b);
        }
    }

    static bool IsFinite(Vector3 v) => float.IsFinite(v.X) && float.IsFinite(v.Y) && float.IsFinite(v.Z);

    public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
    {
        var resolution = renderer.Surface.Resolution;
        renderer.TextBatcher.Write(text.Clear().Append("Three pairs of identical extruded polygon hulls spawned overlapping at the same relative pose; only the side count differs."), new Vector2(16, resolution.Y - 80), 16, Vector3.One, font);
        renderer.TextBatcher.Write(text.Clear().Append("The 6-sided pair's manifold has NaN depths on the first timestep, and the NaN propagates into the body poses."), new Vector2(16, resolution.Y - 64), 16, Vector3.One, font);
        renderer.TextBatcher.Write(text.Clear().Append("In Debug, CHECKMATH catches it in constraint creation on the first timestep instead."), new Vector2(16, resolution.Y - 48), 16, Vector3.One, font);
        for (int i = 0; i < pairs.Length; ++i)
        {
            var (sides, a, b) = pairs[i];
            var finite = IsFinite(Simulation.Bodies[a].Pose.Position) && IsFinite(Simulation.Bodies[b].Pose.Position);
            renderer.TextBatcher.Write(
                text.Clear().Append("sides=").Append(sides).Append(finite ? ": poses finite" : ": POSES ARE NaN"),
                new Vector2(16 + i * 200, resolution.Y - 16), 16, finite ? new Vector3(0.5f, 1, 0.5f) : new Vector3(1, 0.3f, 0.3f), font);
        }
        base.Render(renderer, camera, input, text, font);
    }
}
