using UnityEngine;

namespace PhysicsCharacterController
{
    public readonly struct SwimmingCapsuleGeometry
    {
        public Vector3 PointA { get; }
        public Vector3 PointB { get; }
        public float RadiusMeters { get; }

        public SwimmingCapsuleGeometry(Vector3 pointA, Vector3 pointB, float radiusMeters)
        {
            PointA = pointA;
            PointB = pointB;
            RadiusMeters = radiusMeters;
        }
    }

    public sealed class SwimmingCapsuleGeometryCalculator
    {
        public SwimmingCapsuleGeometry Calculate(
            Vector3 colliderPosition,
            Quaternion colliderRotation,
            Vector3 colliderScale,
            Vector3 localCenter,
            float heightMeters,
            float radiusMeters,
            int directionAxis)
        {
            Vector3 localAxis = ResolveLocalAxis(directionAxis);
            float axisScale = ResolveAxisScale(colliderScale, directionAxis);
            float perpendicularScale = ResolvePerpendicularScale(colliderScale, directionAxis);
            float worldRadiusMeters = radiusMeters * perpendicularScale;
            float worldHeightMeters = Mathf.Max(heightMeters * axisScale, worldRadiusMeters * 2f);
            float halfSegmentLengthMeters = worldHeightMeters * 0.5f - worldRadiusMeters;
            Vector3 worldCenter = colliderPosition + colliderRotation * Vector3.Scale(localCenter, colliderScale);
            Vector3 worldAxis = colliderRotation * localAxis;

            return new SwimmingCapsuleGeometry(
                worldCenter + worldAxis * halfSegmentLengthMeters,
                worldCenter - worldAxis * halfSegmentLengthMeters,
                worldRadiusMeters);
        }

        private static Vector3 ResolveLocalAxis(int directionAxis)
        {
            return directionAxis switch
            {
                0 => Vector3.right,
                1 => Vector3.up,
                2 => Vector3.forward,
                _ => Vector3.up
            };
        }

        private static float ResolveAxisScale(Vector3 colliderScale, int directionAxis)
        {
            return directionAxis switch
            {
                0 => Mathf.Abs(colliderScale.x),
                1 => Mathf.Abs(colliderScale.y),
                2 => Mathf.Abs(colliderScale.z),
                _ => Mathf.Abs(colliderScale.y)
            };
        }

        private static float ResolvePerpendicularScale(Vector3 colliderScale, int directionAxis)
        {
            return directionAxis switch
            {
                0 => Mathf.Max(Mathf.Abs(colliderScale.y), Mathf.Abs(colliderScale.z)),
                1 => Mathf.Max(Mathf.Abs(colliderScale.x), Mathf.Abs(colliderScale.z)),
                2 => Mathf.Max(Mathf.Abs(colliderScale.x), Mathf.Abs(colliderScale.y)),
                _ => Mathf.Max(Mathf.Abs(colliderScale.x), Mathf.Abs(colliderScale.z))
            };
        }
    }
}
