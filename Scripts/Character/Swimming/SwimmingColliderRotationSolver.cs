using UnityEngine;

namespace PhysicsCharacterController
{
    public sealed class SwimmingColliderRotationSolver
    {
        public Quaternion CalculateTargetRotation(Vector3 worldDirection, Vector3 fallbackUpDirection)
        {
            Vector3 normalizedDirection = worldDirection.normalized;
            Vector3 stableUpDirection = Vector3.ProjectOnPlane(Vector3.up, normalizedDirection);

            if (stableUpDirection.sqrMagnitude <= Mathf.Epsilon)
            {
                stableUpDirection = Vector3.ProjectOnPlane(fallbackUpDirection, normalizedDirection);
            }

            if (stableUpDirection.sqrMagnitude <= Mathf.Epsilon)
            {
                stableUpDirection = Vector3.ProjectOnPlane(Vector3.forward, normalizedDirection);
            }

            return Quaternion.LookRotation(normalizedDirection, stableUpDirection.normalized);
        }
    }
}
