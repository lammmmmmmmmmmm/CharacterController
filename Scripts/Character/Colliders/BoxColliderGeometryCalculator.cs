using UnityEngine;

namespace PhysicsCharacterController
{
    public readonly struct BoxColliderGeometry
    {
        public BoxColliderGeometry(Vector3 center, Vector3 halfExtentsMeters, Quaternion rotation)
        {
            Center = center;
            HalfExtentsMeters = halfExtentsMeters;
            Rotation = rotation;
        }

        public Vector3 Center { get; }
        public Vector3 HalfExtentsMeters { get; }
        public Quaternion Rotation { get; }
    }

    public sealed class BoxColliderGeometryCalculator
    {
        public BoxColliderGeometry Calculate(
            Vector3 worldPosition,
            Quaternion worldRotation,
            Vector3 lossyScale,
            Vector3 localCenter,
            Vector3 sizeMeters)
        {
            Vector3 scaledLocalCenter = Vector3.Scale(localCenter, lossyScale);
            Vector3 worldCenter = worldPosition + worldRotation * scaledLocalCenter;
            Vector3 absoluteScale = new(
                Mathf.Abs(lossyScale.x),
                Mathf.Abs(lossyScale.y),
                Mathf.Abs(lossyScale.z));
            Vector3 halfExtentsMeters = Vector3.Scale(sizeMeters * 0.5f, absoluteScale);
            return new BoxColliderGeometry(worldCenter, halfExtentsMeters, worldRotation);
        }
    }
}
