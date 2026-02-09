using System;
using UnityEngine;

namespace PhysicsCharacterController
{
    public class CharacterMove : MonoBehaviour
    {
        [Header("Movement Settings")]
        [SerializeField] private float _movementSpeed = 14f;
        [SerializeField] private float _sprintSpeed = 20f;
        [Range(0.01f, 0.99f)]
        [Tooltip("Minimum input value to trigger movement")]
        [SerializeField] private float _movementThreshold = 0.01f;
        [SerializeField] private float _acceleration = 10f;
        [SerializeField] private float _deceleration = 10f;

        [Header("References")]
        [SerializeField] private BaseCharacterInput _input;

        private Rigidbody _rigidbody;
        private IMovementModifier[] _speedModifiers;
        private IVelocityModifier[] _velocityModifiers;

        private float _currentSpeed;
        private Vector2 _currentInput;
        private bool _isSprinting;

        public float WalkSpeed => _movementSpeed;
        public float SprintSpeed => _sprintSpeed;
        public float CurrentSpeed => _currentSpeed;
        public float MovementThreshold => _movementThreshold;
        public bool IsSprinting => _isSprinting;

        private void Awake()
        {
            _rigidbody = GetComponent<Rigidbody>();
            _speedModifiers = GetComponents<IMovementModifier>();
            _velocityModifiers = GetComponents<IVelocityModifier>();
        }

        private void OnEnable()
        {
            _input.OnSprint += OnSprintInput;
        }

        private void FixedUpdate()
        {
            Move();
        }

        private void OnDisable()
        {
            _input.OnSprint -= OnSprintInput;
        }

        private void OnSprintInput(bool isSprinting) => _isSprinting = isSprinting;

        private void Move()
        {
            _currentInput = _input.GetMoveInput();

            if (_currentInput.magnitude > _movementThreshold)
            {
                MoveWithInput();
            }
            else
            {
                Decelerate();
            }
        }

        private void MoveWithInput()
        {
            float targetSpeed = CalculateTargetSpeed();
            float combinedMultiplier = GetCombinedSpeedMultiplier();
            targetSpeed *= combinedMultiplier;

            _currentSpeed = Mathf.MoveTowards(_currentSpeed, targetSpeed, _acceleration * Time.fixedDeltaTime);

            ApplyMovementVelocity();
        }

        private float CalculateTargetSpeed()
        {
            return _isSprinting ? _sprintSpeed : _movementSpeed;
        }

        private float GetCombinedSpeedMultiplier()
        {
            float combinedMultiplier = 1f;
            foreach (var modifier in _speedModifiers)
            {
                combinedMultiplier *= modifier.GetSpeedMultiplier(_input.HorizontalMoveDirection);
            }

            return combinedMultiplier;
        }

        private void ApplyMovementVelocity()
        {
            Vector3 baseMovementVelocity = CalculateBaseVelocity();
            Vector3 additiveVelocity = CalculateAdditiveVelocity(baseMovementVelocity);

            // Combine: Base Input + Gravity (preserved from RB) + Additive Modifiers
            Vector3 finalVelocity = baseMovementVelocity;
            finalVelocity.x += additiveVelocity.x;
            // This can increase or decrease Y velocity way too much
            finalVelocity.y = _rigidbody.linearVelocity.y + additiveVelocity.y;
            finalVelocity.z += additiveVelocity.z;

            _rigidbody.linearVelocity = finalVelocity;
        }

        private Vector3 CalculateBaseVelocity()
        {
            return _input.HorizontalMoveDirection * _currentSpeed;
        }

        private Vector3 CalculateAdditiveVelocity(Vector3 baseVelocity)
        {
            Vector3 totalAdditive = Vector3.zero;

            foreach (var modifier in _velocityModifiers)
            {
                totalAdditive += modifier.GetVelocityContribution(_rigidbody.linearVelocity, baseVelocity);
            }

            return totalAdditive;
        }

        private void Decelerate()
        {
            _currentSpeed = Mathf.MoveTowards(_currentSpeed, 0f, _deceleration * Time.fixedDeltaTime);
            ApplyDecelerationVelocity();
        }

        private void ApplyDecelerationVelocity()
        {
            Vector3 baseMovementVelocity = CalculateBaseVelocity();

            Vector3 finalVelocity = baseMovementVelocity;
            finalVelocity.y = _rigidbody.linearVelocity.y;

            _rigidbody.linearVelocity = finalVelocity;
        }

        public bool IsMoving()
        {
            return _currentInput.magnitude > _movementThreshold;
        }

        private void OnDrawGizmosSelected()
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(transform.position, transform.position + _input.HorizontalMoveDirection * 2f);
        }
    }
}
