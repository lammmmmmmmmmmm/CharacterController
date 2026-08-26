using UnityEngine;

namespace PhysicsCharacterController
{
    public sealed class CharacterYawSolver
    {
        private const float MINIMUM_HORIZONTAL_DIRECTION_SQR_MAGNITUDE = 0.0001f;

        #region Public Methods

        public bool TryResolveHorizontalFacingRotation(
            Vector3 worldDirection,
            Quaternion currentRotation,
            out Quaternion targetRotation)
        {
            Vector3 horizontalDirection = Vector3.ProjectOnPlane(worldDirection, Vector3.up);
            if (horizontalDirection.sqrMagnitude <= MINIMUM_HORIZONTAL_DIRECTION_SQR_MAGNITUDE)
            {
                targetRotation = currentRotation;
                return false;
            }

            targetRotation = Quaternion.LookRotation(horizontalDirection.normalized, Vector3.up);
            return true;
        }

        public Quaternion RotateTowards(
            Quaternion currentRotation,
            Quaternion targetRotation,
            float maximumTurnDegrees)
        {
            return Quaternion.RotateTowards(currentRotation, targetRotation, maximumTurnDegrees);
        }

        #endregion
    }
}
