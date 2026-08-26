using UnityEngine;

namespace PhysicsCharacterController
{
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Rigidbody))]
    public sealed class UnderwaterSwimmingCollider : MonoBehaviour
    {
        private const int MAX_OVERLAP_COUNT = 32;

        [Header("Colliders")]
        [SerializeField] private CharacterColliderShape _uprightCollider;
        [SerializeField] private Transform _underwaterColliderPivot;
        [SerializeField] private CharacterColliderShape _underwaterCollider;
        [SerializeField] private PhysicsMaterial _swimmingPhysicsMaterial;

        [Header("Collision")]
        [SerializeField] private LayerMask _solidCollisionMask = ~0;

        [Header("Dependencies")]
        [SerializeField] private Rigidbody _rigidbody;

        private readonly Collider[] _overlapResults = new Collider[MAX_OVERLAP_COUNT];
        private readonly SwimmingColliderRotationSolver _rotationSolver = new();

        public bool IsActive => _underwaterCollider.IsPhysicsEnabled;
        public Vector3 AcceptedDirection => _underwaterColliderPivot.forward;
        public Quaternion AcceptedRotation => _underwaterColliderPivot.rotation;

        #region Unity Lifecycle

        private void Awake()
        {
            _underwaterCollider.PhysicsCollider.sharedMaterial = _swimmingPhysicsMaterial;
            _underwaterCollider.SetPhysicsEnabled(false);
            _uprightCollider.SetPhysicsEnabled(true);
        }

        #endregion

        #region Public Methods

        public bool TryActivate(Vector3 worldDirection)
        {
            Vector3 activationDirection = worldDirection.sqrMagnitude > Mathf.Epsilon
                ? worldDirection.normalized
                : transform.forward;
            Quaternion candidateRotation = _rotationSolver.CalculateTargetRotation(activationDirection, transform.forward);

            if (!IsColliderPoseClear(_underwaterCollider, candidateRotation))
            {
                return false;
            }

            _underwaterColliderPivot.rotation = candidateRotation;
            _uprightCollider.SetPhysicsEnabled(false);
            _underwaterCollider.SetPhysicsEnabled(true);
            return true;
        }

        public bool TryAlign(Vector3 worldDirection, float fixedDeltaTime, float rotationSpeedDegreesPerSecond)
        {
            if (!IsActive || worldDirection.sqrMagnitude <= Mathf.Epsilon)
            {
                return false;
            }

            Quaternion targetRotation = _rotationSolver.CalculateTargetRotation(worldDirection, _underwaterColliderPivot.up);
            Quaternion candidateRotation = Quaternion.RotateTowards(
                _underwaterColliderPivot.rotation,
                targetRotation,
                rotationSpeedDegreesPerSecond * fixedDeltaTime);

            if (!IsColliderPoseClear(_underwaterCollider, candidateRotation))
            {
                return false;
            }

            _underwaterColliderPivot.rotation = candidateRotation;
            return true;
        }

        public bool TryDeactivate()
        {
            if (!IsActive)
            {
                return true;
            }

            if (!IsColliderPoseClear(_uprightCollider))
            {
                return false;
            }

            _underwaterCollider.SetPhysicsEnabled(false);
            _uprightCollider.SetPhysicsEnabled(true);
            return true;
        }

        #endregion

        #region Private Methods

        private bool IsColliderPoseClear(CharacterColliderShape colliderShape)
        {
            int overlapCount = colliderShape.OverlapNonAlloc(_overlapResults, _solidCollisionMask, QueryTriggerInteraction.Ignore);
            return AreOverlapsClear(overlapCount);
        }

        private bool IsColliderPoseClear(CharacterColliderShape colliderShape, Quaternion candidateRotation)
        {
            int overlapCount = colliderShape.OverlapAtPoseNonAlloc(
                colliderShape.PhysicsCollider.transform.position,
                candidateRotation,
                _overlapResults,
                _solidCollisionMask,
                QueryTriggerInteraction.Ignore);

            return AreOverlapsClear(overlapCount);
        }

        private bool AreOverlapsClear(int overlapCount)
        {
            for (int overlapIndex = 0; overlapIndex < overlapCount; overlapIndex++)
            {
                Collider overlap = _overlapResults[overlapIndex];
                if (overlap.attachedRigidbody == _rigidbody)
                {
                    continue;
                }

                return false;
            }

            if (overlapCount == MAX_OVERLAP_COUNT)
            {
                Debug.LogWarning($"Swimming collider clearance for '{name}' filled the overlap buffer; treating the candidate pose as blocked.", this);
                return false;
            }

            return true;
        }

        #endregion
    }
}
