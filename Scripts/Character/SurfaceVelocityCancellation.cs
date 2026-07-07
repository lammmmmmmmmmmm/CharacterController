using UnityEngine;

namespace PhysicsCharacterController
{
    /// <summary>
    /// Computes the velocity needed to cancel movement pushing into an impassable surface
    /// (walls, unclimbable slopes), so the character slides along it instead of ramming it.
    /// Ramming feeds the physics solver a constant penetration, which reads as shaking, and on
    /// inclined surfaces the solver converts the push into an unintended climb.
    /// </summary>
    public static class SurfaceVelocityCancellation
    {
        private const float MIN_HORIZONTAL_NORMAL_SQR_MAGNITUDE = 0.001f;

        public static Vector3 CalculateCancellation(Vector3 desiredMovement, Vector3 surfaceNormal)
        {
            // Inclined surfaces have a vertical normal component; flatten it so the
            // cancellation never injects vertical velocity.
            Vector3 horizontalNormal = new(surfaceNormal.x, 0f, surfaceNormal.z);
            if (horizontalNormal.sqrMagnitude < MIN_HORIZONTAL_NORMAL_SQR_MAGNITUDE)
            {
                return Vector3.zero;
            }

            horizontalNormal.Normalize();

            float speedIntoSurface = Vector3.Dot(desiredMovement, horizontalNormal);
            if (speedIntoSurface >= 0f)
            {
                return Vector3.zero;
            }

            return -speedIntoSurface * horizontalNormal;
        }
    }
}
