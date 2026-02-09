using System;
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
        [Tooltip("Default character mesh")]
        [SerializeField] private GameObject _meshCharacter;
        [Tooltip("Crouch character mesh")]
        [SerializeField] private GameObject _meshCharacterCrouch;
        [SerializeField] private GroundChecker _groundChecker;
        [SerializeField] private BaseCharacterInput _input;

        private CapsuleCollider _collider;
        private float _originalColliderHeight;
        private bool _isCrouching;

        public bool IsCrouching => _isCrouching;
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

        private void OnCrouchInput(bool wantsToCrouch)
        {
            HandleCrouch(wantsToCrouch);
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

        private void HandleCrouch(bool wantsToCrouch)
        {
            bool shouldCrouchInput = wantsToCrouch && _groundChecker.IsGrounded;
            bool isBlockedAbove = _isCrouching && IsObstacleAbove();

            if (shouldCrouchInput || isBlockedAbove)
            {
                ApplyCrouchState();
            }
            else
            {
                ApplyStandState();
            }
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

        private void ApplyCrouchState()
        {
            _isCrouching = true;

            UpdateMeshVisibility(isCrouching: true);
            UpdateCollider(isCrouching: true);
            UpdateHeadPosition(_povCrouchHeadHeight);
        }

        private void ApplyStandState()
        {
            _isCrouching = false;

            UpdateMeshVisibility(isCrouching: false);
            UpdateCollider(isCrouching: false);
            UpdateHeadPosition(_povNormalHeadHeight);
        }

        private void UpdateMeshVisibility(bool isCrouching)
        {
            //TODO: abstract this
            if (_meshCharacterCrouch && _meshCharacter)
            {
                _meshCharacter.SetActive(!isCrouching);
            }

            if (_meshCharacterCrouch)
            {
                _meshCharacterCrouch.SetActive(isCrouching);
            }
        }

        private void UpdateCollider(bool isCrouching)
        {
            if (isCrouching)
            {
                float newHeight = _originalColliderHeight * _crouchHeightMultiplier;
                _collider.height = newHeight;
                _collider.center = new Vector3(0f, -newHeight * _crouchHeightMultiplier, 0f);
            }
            else
            {
                _collider.height = _originalColliderHeight;
                _collider.center = Vector3.zero;
            }
        }

        private void UpdateHeadPosition(Vector3 headOffset)
        {
            _headPoint.position = new Vector3(
                transform.position.x + headOffset.x,
                transform.position.y + headOffset.y,
                transform.position.z + headOffset.z);
        }

        public float GetSpeedMultiplier(Vector3 intendedDirection)
        {
            return _isCrouching ? _crouchSpeedMultiplier : 1f;
        }
    }
}
