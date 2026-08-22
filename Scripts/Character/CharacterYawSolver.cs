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

        public Quaternion ResolveHorizontalFacingRotation(Vector3 worldDirection, Quaternion currentRotation)
        {
            Vector3 horizontalDirection = Vector3.ProjectOnPlane(worldDirection, Vector3.up);
            if (horizontalDirection.sqrMagnitude <= Mathf.Epsilon)
            {
                return Quaternion.Euler(0f, currentRotation.eulerAngles.y, 0f);
            }

            return Quaternion.LookRotation(horizontalDirection.normalized, Vector3.up);
        }

        public void ResetSmoothingVelocity()
        {
            _smoothVelocityDegreesPerSecond = 0f;
        }

        #endregion
    }
}
