using DG.Tweening;
using UnityEngine;

namespace PhysicsCharacterController
{
    public class CharacterCrouch : MonoBehaviour, IMovementModifier
    {
        [Header("Crouch Settings")]
        [Range(0f, 1f)]
        [SerializeField] private float _crouchSpeedMultiplier = 0.248f;
        [Tooltip("Multiplier applied to the collider when player is crouching")]
        [SerializeField] private float _crouchHeightMultiplier = 0.5f;
        [Tooltip("Duration of the crouch/stand transition in seconds")]
        [SerializeField] private float _transitionDurationSeconds = 0.2f;

        [Header("Head Position")]
        [Tooltip("FP camera head height")]
        [SerializeField] private Vector3 _povNormalHeadHeight = new(0f, 0.5f, -0.1f);
        [Tooltip("FP camera head height when crouching")]
        [SerializeField] private Vector3 _povCrouchHeadHeight = new(0f, -0.1f, -0.1f);

        [Header("Obstacle Detection")]
        [Tooltip("Mask for obstacles above the character")]
        [SerializeField] private LayerMask _obstacleMask;
        [Tooltip("Radius of the sphere check for obstacles")]
        [SerializeField] private float _obstacleCheckRadius = 0.2f;

        [Header("References")]
        [Tooltip("Head reference")]
        [SerializeField] private Transform _headPoint;
        [SerializeField] private GroundChecker _groundChecker;
        [SerializeField] private BaseCharacterInput _input;

        private CapsuleCollider _collider;
        private float _originalColliderHeight;
        private bool _isCrouching;
        private bool _wantsToCrouch;
        private float _transitionProgress;
        private Tweener _transitionTween;

        public bool IsCrouching => _isCrouching;
        public bool WantsToCrouch => _wantsToCrouch;
        public float CrouchHeightMultiplier => _crouchHeightMultiplier;
        public float CrouchSpeedMultiplier => _crouchSpeedMultiplier;

        private void Awake()
        {
            _collider = GetComponent<CapsuleCollider>();
            _originalColliderHeight = _collider.height;
        }

        private void OnEnable()
        {
            _input.OnCrouch += OnCrouchInput;
        }

        private void OnDisable()
        {
            _input.OnCrouch -= OnCrouchInput;
        }

        private void OnDestroy()
        {
            _transitionTween?.Kill();
        }

        private void OnCrouchInput(bool wantsToCrouch)
        {
            _wantsToCrouch = wantsToCrouch;
        }

        public bool ShouldStayCrouched()
        {
            bool shouldCrouchInput = _wantsToCrouch && _groundChecker.IsGrounded;
            bool isBlockedAbove = _isCrouching && IsObstacleAbove();

            return shouldCrouchInput || isBlockedAbove;
        }

        private void OnDrawGizmosSelected()
        {
            if (!_collider)
            {
                _collider = GetComponent<CapsuleCollider>();
                _originalColliderHeight = _collider.height;
            }

            float currentTop = _collider.center.y + _collider.height * 0.5f;
            float targetTop = _originalColliderHeight * 0.5f;
            float distance = targetTop - currentTop;

            if (distance <= 0)
            {
                return;
            }

            Vector3 origin = transform.position + Vector3.up * currentTop;
            Vector3 end = origin + Vector3.up * distance;

            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(origin, _obstacleCheckRadius);
            Gizmos.DrawLine(origin, end);

            bool isBlocked = IsObstacleAbove();
            Gizmos.color = isBlocked ? Color.red : Color.green;
            Gizmos.DrawWireSphere(end, _obstacleCheckRadius);
        }

        private bool IsObstacleAbove()
        {
            float currentTop = _collider.center.y + _collider.height * 0.5f;
            float targetTop = _originalColliderHeight * 0.5f;
            float distance = targetTop - currentTop;

            if (distance <= 0)
            {
                return false;
            }

            Vector3 origin = transform.position + Vector3.up * currentTop;
            return Physics.SphereCast(origin, _obstacleCheckRadius, Vector3.up, out _, distance, _obstacleMask, QueryTriggerInteraction.Ignore);
        }

        public void ApplyCrouchState()
        {
            _isCrouching = true;
            TransitionTo(targetProgress: 1f);
        }

        public void ApplyStandState()
        {
            _isCrouching = false;
            TransitionTo(targetProgress: 0f);
        }

        private void TransitionTo(float targetProgress)
        {
            _transitionTween?.Kill();

            float remainingRatio = Mathf.Abs(targetProgress - _transitionProgress);
            float scaledDurationSeconds = _transitionDurationSeconds * remainingRatio;

            _transitionTween = DOTween
                .To(() => _transitionProgress, value => _transitionProgress = value, targetProgress, scaledDurationSeconds)
                .SetEase(Ease.OutQuad)
                .OnUpdate(ApplyTransitionProgress)
                .SetLink(gameObject);

            ApplyTransitionProgress();
        }

        private void ApplyTransitionProgress()
        {
            float crouchedHeight = _originalColliderHeight * _crouchHeightMultiplier;

            float height = Mathf.Lerp(_originalColliderHeight, crouchedHeight, _transitionProgress);
            float centerY = Mathf.Lerp(0f, -crouchedHeight * _crouchHeightMultiplier, _transitionProgress);

            _collider.height = height;
            _collider.center = new Vector3(0f, centerY, 0f);

            Vector3 headOffset = Vector3.Lerp(_povNormalHeadHeight, _povCrouchHeadHeight, _transitionProgress);
            _headPoint.position = transform.position + headOffset;
        }

        public float GetSpeedMultiplier(Vector3 intendedDirection)
        {
            return _isCrouching ? _crouchSpeedMultiplier : 1f;
        }
    }
}