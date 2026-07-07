using UnityEngine;

namespace PhysicsCharacterController
{
    /// <summary>
    /// Tracking target for the third person camera that filters out terrain bumps without lagging jumps.
    /// Follows the head point exactly on X/Z; on Y it smooths small displacements (steps, uneven ground)
    /// but clamps the smoothing offset, so large fast motion like jumping passes through 1:1.
    /// While airborne it converges to exact 1:1 tracking instead — bumps only exist on the ground.
    /// The target's position is continuous state, so changing convergence speed can never snap.
    /// </summary>
    public class ThirdPersonCameraTarget : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private Transform _headPoint;
        [SerializeField] private GroundChecker _groundChecker;

        [Header("Settings")]
        [SerializeField] private AirborneSmoothingSwitch _ySmoothing = new(groundedSmoothTimeSeconds: 0.35f, airborneSmoothTimeSeconds: 0.05f);
        [Tooltip("Max vertical distance the smoothed target may trail the head point. Beyond it the target moves with the head 1:1")]
        [SerializeField] private float _maxYOffsetMeters = 0.3f;

        private float _smoothedY;
        private float _ySmoothVelocity;

        #region Unity Lifecycle

        private void OnEnable()
        {
            _smoothedY = _headPoint.position.y;
            _ySmoothVelocity = 0f;
            transform.position = _headPoint.position;
        }

        private void LateUpdate()
        {
            FollowHeadPoint();
        }

        #endregion

        private void FollowHeadPoint()
        {
            Vector3 headPosition = _headPoint.position;
            float smoothTimeSeconds = _ySmoothing.SelectSmoothTimeSeconds(_groundChecker.IsGrounded, Time.deltaTime);

            _smoothedY = Mathf.SmoothDamp(_smoothedY, headPosition.y, ref _ySmoothVelocity, smoothTimeSeconds);
            _smoothedY = Mathf.Clamp(
                _smoothedY,
                headPosition.y - _maxYOffsetMeters,
                headPosition.y + _maxYOffsetMeters);

            transform.position = new Vector3(headPosition.x, _smoothedY, headPosition.z);
        }
    }
}
