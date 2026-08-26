using UnityEngine;

namespace PhysicsCharacterController
{
    /// <summary>
    /// Applies requested horizontal character rotations through the character Rigidbody.
    /// Rotation policies decide when to call this component and which direction to request.
    /// </summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Rigidbody))]
    public sealed class CharacterRotation : MonoBehaviour
    {
        [Header("Settings")]
        [Min(0f)]
        [SerializeField] private float _turnSpeedDegreesPerSecond = 540f;

        private readonly CharacterYawSolver _yawSolver = new();
        private Rigidbody _characterRigidbody;

        #region Unity Lifecycle

        private void Awake()
        {
            _characterRigidbody = GetComponent<Rigidbody>();
        }

        #endregion

        #region Public Methods

        public Vector3 CurrentFacingDirectionNormalized => _characterRigidbody.rotation * Vector3.forward;

        public void RotateTowardsDirection(Vector3 worldDirection, float deltaSeconds)
        {
            if (!_yawSolver.TryResolveHorizontalFacingRotation(worldDirection, _characterRigidbody.rotation, out Quaternion targetRotation))
            {
                return;
            }

            float maximumTurnDegrees = _turnSpeedDegreesPerSecond * deltaSeconds;
            Quaternion nextRotation = _yawSolver.RotateTowards(_characterRigidbody.rotation, targetRotation, maximumTurnDegrees);
            _characterRigidbody.MoveRotation(nextRotation);
        }

        public void SetFacingDirectionImmediately(Vector3 worldDirection)
        {
            if (!_yawSolver.TryResolveHorizontalFacingRotation(worldDirection, _characterRigidbody.rotation, out Quaternion targetRotation))
            {
                return;
            }

            _characterRigidbody.rotation = targetRotation;
        }

        #endregion
    }
}
