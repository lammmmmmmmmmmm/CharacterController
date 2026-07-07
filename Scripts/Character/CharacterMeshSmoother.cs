using UnityEngine;

namespace PhysicsCharacterController
{
    /// <summary>
    /// Visually decouples the character mesh from physics jitter, gated by surface roughness.
    /// On smooth ground (flat or gradual slopes) the mesh locks to the body 1:1, so it can never
    /// clip or float. Rough terrain is detected by how fast the ground contact normal varies;
    /// there, vertical smoothing fades in to absorb the bumps. Horizontal rattle is always
    /// filtered tightly so stopping never reads as sliding. Attach to the mesh child of the
    /// physics root.
    /// </summary>
    public class CharacterMeshSmoother : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private GroundChecker _groundChecker;

        [Header("Vertical Smoothing")]
        [SerializeField] private AirborneSmoothingSwitch _verticalSmoothing = new(groundedSmoothTimeSeconds: 0.15f, airborneSmoothTimeSeconds: 0.05f);
        [Tooltip("Max vertical distance the mesh may stray from its authored position on the collider")]
        [SerializeField] private float _maxVerticalOffsetMeters = 0.25f;

        [Header("Roughness Detection")]
        [Tooltip("Ground normal rotation rate at which the surface counts as fully rough. Gradual slopes stay near zero; bumps and rocks spike far above")]
        [SerializeField] private float _fullRoughnessNormalRateDegreesPerSecond = 60f;
        [Tooltip("Time for smoothing to fade back out after the last detected bump, so it holds steady across a bumpy patch")]
        [SerializeField] private float _roughnessReleaseSeconds = 0.4f;

        [Header("Horizontal Smoothing")]
        [Tooltip("Keep tight: only filters collision rattle. Higher values make the mesh slide when the character stops")]
        [SerializeField] private float _horizontalSmoothTimeSeconds = 0.06f;
        [Tooltip("Max horizontal distance the mesh may stray from its authored position on the collider")]
        [SerializeField] private float _maxHorizontalOffsetMeters = 0.1f;

        private Vector3 _authoredLocalPosition;
        private Vector3 _smoothedWorldPosition;
        private Vector3 _smoothVelocity;
        private Vector3 _previousGroundNormal;
        private float _roughness01;

        #region Unity Lifecycle

        private void Awake()
        {
            _authoredLocalPosition = transform.localPosition;
        }

        private void OnEnable()
        {
            _smoothedWorldPosition = GetTargetWorldPosition();
            _smoothVelocity = Vector3.zero;
            _previousGroundNormal = Vector3.zero;
            _roughness01 = 0f;
        }

        private void LateUpdate()
        {
            FollowPhysicsBody();
        }

        #endregion

        private void FollowPhysicsBody()
        {
            if (Time.deltaTime <= 0f)
            {
                return;
            }

            bool isGrounded = _groundChecker.IsGrounded;
            UpdateRoughness(isGrounded);

            Vector3 targetWorldPosition = GetTargetWorldPosition();

            float verticalSmoothTimeSeconds = _verticalSmoothing.SelectSmoothTimeSeconds(isGrounded, Time.deltaTime);
            if (isGrounded)
            {
                verticalSmoothTimeSeconds *= _roughness01;
            }

            float smoothedY = Mathf.SmoothDamp(_smoothedWorldPosition.y, targetWorldPosition.y, ref _smoothVelocity.y, verticalSmoothTimeSeconds);
            smoothedY = Mathf.Clamp(
                smoothedY,
                targetWorldPosition.y - _maxVerticalOffsetMeters,
                targetWorldPosition.y + _maxVerticalOffsetMeters);

            float smoothedX = Mathf.SmoothDamp(_smoothedWorldPosition.x, targetWorldPosition.x, ref _smoothVelocity.x, _horizontalSmoothTimeSeconds);
            float smoothedZ = Mathf.SmoothDamp(_smoothedWorldPosition.z, targetWorldPosition.z, ref _smoothVelocity.z, _horizontalSmoothTimeSeconds);
            Vector2 horizontalOffset = new(smoothedX - targetWorldPosition.x, smoothedZ - targetWorldPosition.z);
            if (horizontalOffset.sqrMagnitude > _maxHorizontalOffsetMeters * _maxHorizontalOffsetMeters)
            {
                horizontalOffset = horizontalOffset.normalized * _maxHorizontalOffsetMeters;
            }

            _smoothedWorldPosition = new Vector3(
                targetWorldPosition.x + horizontalOffset.x,
                smoothedY,
                targetWorldPosition.z + horizontalOffset.y);

            transform.position = _smoothedWorldPosition;
        }

        private void UpdateRoughness(bool isGrounded)
        {
            if (isGrounded)
            {
                Vector3 groundNormal = _groundChecker.GroundHit.normal;
                bool hasNewNormalSample = _previousGroundNormal != Vector3.zero && groundNormal != _previousGroundNormal;

                if (hasNewNormalSample)
                {
                    // Normal samples change only on physics ticks, so rate against fixed delta.
                    float normalRateDegreesPerSecond = Vector3.Angle(_previousGroundNormal, groundNormal) / Time.fixedDeltaTime;
                    float roughnessImpulse01 = Mathf.Clamp01(normalRateDegreesPerSecond / _fullRoughnessNormalRateDegreesPerSecond);
                    _roughness01 = Mathf.Max(_roughness01, roughnessImpulse01);
                }

                _previousGroundNormal = groundNormal;
            }
            else
            {
                _previousGroundNormal = Vector3.zero;
            }

            _roughness01 = Mathf.MoveTowards(_roughness01, 0f, Time.deltaTime / _roughnessReleaseSeconds);
        }

        private Vector3 GetTargetWorldPosition()
        {
            return transform.parent.TransformPoint(_authoredLocalPosition);
        }
    }
}
