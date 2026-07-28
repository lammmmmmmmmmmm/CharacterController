using DG.Tweening;
using UnityEngine;

namespace PhysicsCharacterController
{
    /// <summary>
    /// Applies crouch input by resizing the configured CharacterColliderShape and moving the
    /// first-person head point. The collider abstraction preserves the character's feet position,
    /// so crouching behaves consistently with capsule and box colliders.
    /// </summary>
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

        private CharacterColliderShape _characterColliderShape;
        private bool _isCrouching;
        private bool _wantsToCrouch;
        private float _transitionProgress;
        private Tweener _transitionTween;

        public bool IsCrouching => _isCrouching;
        public bool WantsToCrouch => _wantsToCrouch;
        public float CrouchHeightMultiplier => _crouchHeightMultiplier;
        public float CrouchSpeedMultiplier => _crouchSpeedMultiplier;

        #region Unity Lifecycle

        private void Awake()
        {
            _characterColliderShape = GetComponent<CharacterColliderShape>();
        }

        private void OnEnable()
        {
            _input.OnCrouch += UpdateCrouchIntent;
        }

        private void OnDisable()
        {
            _input.OnCrouch -= UpdateCrouchIntent;
        }

        private void OnDestroy()
        {
            _transitionTween?.Kill();
        }

        private void OnDrawGizmosSelected()
        {
            CharacterColliderShape characterColliderShape = GetComponent<CharacterColliderShape>();
            characterColliderShape.RefreshColliderCache();
            float currentTopOffsetMeters = characterColliderShape.CurrentTopOffsetMeters;
            float standingTopOffsetMeters = Application.isPlaying
                ? characterColliderShape.OriginalTopOffsetMeters
                : currentTopOffsetMeters;
            float clearanceDistanceMeters = standingTopOffsetMeters - currentTopOffsetMeters;

            if (clearanceDistanceMeters <= 0f)
            {
                return;
            }

            Vector3 origin = transform.position + Vector3.up * currentTopOffsetMeters;
            Vector3 end = origin + Vector3.up * clearanceDistanceMeters;

            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(origin, _obstacleCheckRadius);
            Gizmos.DrawLine(origin, end);

            bool isBlocked = IsObstacleAbove();
            Gizmos.color = isBlocked ? Color.red : Color.green;
            Gizmos.DrawWireSphere(end, _obstacleCheckRadius);
        }

        #endregion

        #region Public Methods

        public bool ShouldStayCrouched()
        {
            bool shouldCrouchFromInput = _wantsToCrouch && _groundChecker.IsGrounded;
            bool isBlockedAbove = _isCrouching && IsObstacleAbove();

            return shouldCrouchFromInput || isBlockedAbove;
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

        public float GetSpeedMultiplier(Vector3 intendedDirection)
        {
            return _isCrouching ? _crouchSpeedMultiplier : 1f;
        }

        #endregion

        #region Private Methods

        private void UpdateCrouchIntent(bool wantsToCrouch)
        {
            _wantsToCrouch = wantsToCrouch;
        }

        private bool IsObstacleAbove()
        {
            float currentTopOffsetMeters = _characterColliderShape.CurrentTopOffsetMeters;
            float standingTopOffsetMeters = _characterColliderShape.OriginalTopOffsetMeters;
            float clearanceDistanceMeters = standingTopOffsetMeters - currentTopOffsetMeters;

            if (clearanceDistanceMeters <= 0f)
            {
                return false;
            }

            Vector3 origin = transform.position + Vector3.up * currentTopOffsetMeters;
            return Physics.SphereCast(
                origin,
                _obstacleCheckRadius,
                Vector3.up,
                out _,
                clearanceDistanceMeters,
                _obstacleMask,
                QueryTriggerInteraction.Ignore);
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
            float crouchedHeightMeters = _characterColliderShape.OriginalHeightMeters * _crouchHeightMultiplier;
            float heightMeters = Mathf.Lerp(
                _characterColliderShape.OriginalHeightMeters,
                crouchedHeightMeters,
                _transitionProgress);

            _characterColliderShape.SetHeightPreservingBottom(heightMeters);

            Vector3 headOffset = Vector3.Lerp(_povNormalHeadHeight, _povCrouchHeadHeight, _transitionProgress);
            // Local space keeps the camera anchor attached when character yaw changes.
            _headPoint.localPosition = headOffset;
        }

        #endregion
    }
}
