using UnityEngine;

namespace PhysicsCharacterController
{
    /// <summary>
    /// Calculates eye height relative to the character collider. It interpolates the configured
    /// standing and crouched offsets from the collider top, keeping crouch-specific camera math
    /// independent from the component that samples ground and moves the camera target.
    /// </summary>
    public sealed class ColliderRelativeEyeHeightCalculator
    {
        private readonly float _standingColliderHeightMeters;
        private readonly float _crouchedColliderHeightMeters;
        private readonly float _standingEyeOffsetFromColliderTopMeters;
        private readonly float _crouchedEyeOffsetFromColliderTopMeters;

        #region Public Methods

        public ColliderRelativeEyeHeightCalculator(
            float standingColliderHeightMeters,
            float crouchedColliderHeightMeters,
            float standingEyeOffsetFromColliderTopMeters,
            float crouchedEyeOffsetFromColliderTopMeters)
        {
            _standingColliderHeightMeters = standingColliderHeightMeters;
            _crouchedColliderHeightMeters = crouchedColliderHeightMeters;
            _standingEyeOffsetFromColliderTopMeters = standingEyeOffsetFromColliderTopMeters;
            _crouchedEyeOffsetFromColliderTopMeters = crouchedEyeOffsetFromColliderTopMeters;
        }

        public float CalculateEyeHeightMeters(float colliderTopMeters, float currentColliderHeightMeters)
        {
            float colliderHeightRangeMeters = _standingColliderHeightMeters - _crouchedColliderHeightMeters;
            float crouchProgress01 = colliderHeightRangeMeters <= Mathf.Epsilon
                ? 0f
                : Mathf.Clamp01((_standingColliderHeightMeters - currentColliderHeightMeters) / colliderHeightRangeMeters);
            float eyeOffsetFromColliderTopMeters = Mathf.Lerp(_standingEyeOffsetFromColliderTopMeters, _crouchedEyeOffsetFromColliderTopMeters, crouchProgress01);

            return colliderTopMeters + eyeOffsetFromColliderTopMeters;
        }

        #endregion
    }
}
