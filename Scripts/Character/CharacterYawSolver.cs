using UnityEngine;

namespace PhysicsCharacterController
{
    public sealed class CharacterYawSolver
    {
        private float _smoothVelocityDegreesPerSecond;

        #region Public Methods

        public float ResolveTargetYawDegrees(float movementYawDegrees, float cameraYawDegrees, bool isLockedToCamera, bool isCameraFacingOverrideEnabled)
        {
            return isLockedToCamera || isCameraFacingOverrideEnabled ? cameraYawDegrees : movementYawDegrees;
        }

        public float SmoothYawDegrees(float currentYawDegrees, float targetYawDegrees, float smoothingDurationSeconds, float deltaSeconds)
        {
            return Mathf.SmoothDampAngle(currentYawDegrees, targetYawDegrees, ref _smoothVelocityDegreesPerSecond, smoothingDurationSeconds, Mathf.Infinity, deltaSeconds);
        }

        #endregion
    }
}
