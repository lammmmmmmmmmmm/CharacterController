using UnityEngine;

namespace PhysicsCharacterController
{
    /// <summary>
    /// Samples Unity character inputs and applies yaw solved by a testable policy object through the Rigidbody.
    /// </summary>
    public sealed class CharacterRotation : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private BaseCharacterInput _input;
        [SerializeField] private GameObject _characterCamera;
        [SerializeField] private GameObject _characterParent;

        [Header("Settings")]
        [SerializeField] private float _rotationSmoothingDurationSeconds = 0.1f;

        private readonly CharacterYawSolver _yawSolver = new();
        private Rigidbody _characterRigidbody;
        private bool _isLockedToCamera;
        private bool _isCameraFacingOverrideEnabled;
        private bool _isAutomaticRotationEnabled = true;

        public Transform CameraTransform => _characterCamera.transform;

        #region Unity Lifecycle

        private void Awake()
        {
            _characterRigidbody = _characterParent.GetComponent<Rigidbody>();
        }

        private void FixedUpdate()
        {
            RotateCharacter();
        }

        #endregion

        #region Public Methods

        public void SetLockedToCamera(bool isLockedToCamera)
        {
            _isLockedToCamera = isLockedToCamera;
        }

        public void SetCameraFacingOverride(bool isEnabled)
        {
            _isCameraFacingOverrideEnabled = isEnabled;
        }

        public void SetAutomaticRotationEnabled(bool isEnabled)
        {
            if (_isAutomaticRotationEnabled == isEnabled)
            {
                return;
            }

            _isAutomaticRotationEnabled = isEnabled;
            if (isEnabled)
            {
                _yawSolver.ResetSmoothingVelocity();
            }
        }

        public void SetFacingDirectionImmediately(Vector3 worldDirection)
        {
            _yawSolver.ResetSmoothingVelocity();
            _characterRigidbody.rotation = _yawSolver.ResolveHorizontalFacingRotation(
                worldDirection,
                _characterRigidbody.rotation);
        }

        #endregion

        #region Private Methods

        private void RotateCharacter()
        {
            if (!_input.AreNormalActionsEnabled || !_isAutomaticRotationEnabled)
            {
                return;
            }

            float targetYawDegrees = _yawSolver.ResolveTargetYawDegrees(
                _input.GetMoveAngle(),
                _characterCamera.transform.rotation.eulerAngles.y,
                _isLockedToCamera,
                _isCameraFacingOverrideEnabled);
            float smoothedYawDegrees = _yawSolver.SmoothYawDegrees(
                _characterRigidbody.rotation.eulerAngles.y,
                targetYawDegrees,
                _rotationSmoothingDurationSeconds,
                Time.fixedDeltaTime);
            _characterRigidbody.MoveRotation(Quaternion.Euler(0f, smoothedYawDegrees, 0f));
        }

        #endregion
    }
}
