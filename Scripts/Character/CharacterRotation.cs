using UnityEngine;

namespace PhysicsCharacterController
{
    /// <summary>
    /// Rotates the character toward the movement direction, or locks it to the camera yaw in
    /// first person. Rotates through the Rigidbody, never the Transform: writing the transform of
    /// a simulated rigidbody every physics tick forces a sync that breaks position interpolation,
    /// which downstream consumers (camera target, mesh smoother) rely on for per-frame sampling.
    /// </summary>
    public class CharacterRotation : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private BaseCharacterInput _input;
        [SerializeField] private GameObject _characterCamera;
        [SerializeField] private GameObject _characterParent;

        [Header("Settings")]
        [Tooltip("Character rotation speed when the forward direction is changed")]
        [SerializeField] private float _rotationSmooth = 0.1f;

        private Rigidbody _characterRigidbody;
        private float _turnSmoothVelocity;
        private bool _isLockedToCamera;

        private void Awake()
        {
            _characterRigidbody = _characterParent.GetComponent<Rigidbody>();
        }

        private void FixedUpdate()
        {
            if (_isLockedToCamera)
            {
                RotateToCamera();
            }
            else
            {
                RotateToMovementDirection();
            }
        }

        private void RotateToMovementDirection()
        {
            float targetAngle = _input.GetMoveAngle();
            float smoothedAngle = Mathf.SmoothDampAngle(
                _characterRigidbody.rotation.eulerAngles.y,
                targetAngle,
                ref _turnSmoothVelocity,
                _rotationSmooth);

            _characterRigidbody.MoveRotation(Quaternion.Euler(0f, smoothedAngle, 0f));
        }

        private void RotateToCamera()
        {
            float cameraYRotation = _characterCamera.transform.rotation.eulerAngles.y;
            _characterRigidbody.MoveRotation(Quaternion.Euler(0f, cameraYRotation, 0f));
        }

        // Used for First Person characters
        public void SetLockedToCamera(bool isLocked)
        {
            _isLockedToCamera = isLocked;
        }
    }
}
