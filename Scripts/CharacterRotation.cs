using UnityEngine;

namespace PhysicsCharacterController
{
    public class CharacterRotation : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private BaseCharacterInput _input;
        [SerializeField] private GameObject _characterCamera;
        [SerializeField] private GameObject _characterParent;

        [Header("Settings")]
        [Tooltip("Character rotation speed when the forward direction is changed")]
        [SerializeField] private float _rotationSmooth = 0.1f;

        private float _turnSmoothVelocity;
        private bool _isLockedToCamera;

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
                _characterParent.transform.eulerAngles.y,
                targetAngle,
                ref _turnSmoothVelocity,
                _rotationSmooth);

            _characterParent.transform.rotation = Quaternion.Euler(0f, smoothedAngle, 0f);
        }

        private void RotateToCamera()
        {
            float cameraYRotation = _characterCamera.transform.rotation.eulerAngles.y;
            _characterParent.transform.rotation = Quaternion.Euler(0f, cameraYRotation, 0f);
        }

        // Used for First Person characters
        public void SetLockedToCamera(bool isLocked)
        {
            _isLockedToCamera = isLocked;
        }
    }
}
