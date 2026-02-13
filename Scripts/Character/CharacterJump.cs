using System;
using UnityEngine;

namespace PhysicsCharacterController
{
    public class CharacterJump : MonoBehaviour
    {
        [Header("Jump Settings")]
        [SerializeField] private float _maxJumpForce = 30f;
        [Tooltip("Time window in seconds to buffer jump input when not grounded")]
        [SerializeField] private float _jumpBufferTime = 0.2f;
        [SerializeField] private float _coyoteTime = 0.2f;

        [Header("References")]
        [SerializeField] private GroundChecker _groundChecker;
        [SerializeField] private SlopeChecker _slopeChecker;
        [SerializeField] private BaseCharacterInput _input;

        public event Action OnJump;

        public float MaxJumpForce
        {
            get => _maxJumpForce;
            set => _maxJumpForce = value;
        }

        public float CurrentJumpForce { get; set; }

        private Rigidbody _rigidbody;

        private float _jumpBufferTimer;
        private bool _hasJumpRequested;
        private bool _hasJumpBuffered;
        private bool _hasCoyoteTime;
        private float _coyoteTimeCounter;

        private bool ShouldStartCoyoteTime =>
            !_groundChecker.IsGrounded &&
            _groundChecker.WasGrounded &&
            _rigidbody.linearVelocity.y < -0.5f;

        private void Awake()
        {
            _rigidbody = GetComponent<Rigidbody>();
            CurrentJumpForce = _maxJumpForce;
        }

        private void OnEnable()
        {
            _input.OnJumpPressed += HandleJumpInput;
        }

        private void OnDisable()
        {
            _input.OnJumpPressed -= HandleJumpInput;
        }

        private void HandleJumpInput()
        {
            if (CanJumpNow())
            {
                _hasJumpRequested = true;
            }
            else if (!_groundChecker.IsGrounded)
            {
                _hasJumpBuffered = true;
                _jumpBufferTimer = 0f;
            }
        }

        public bool TryExecuteJump()
        {
            bool shouldExecuteRequestedJump = _hasJumpRequested && CanJumpNow();
            bool shouldExecuteBufferedJump = _hasJumpBuffered
                                            && _groundChecker.JustLanded
                                            && !_slopeChecker.IsUnclimbableSlope();

            if (!shouldExecuteRequestedJump && !shouldExecuteBufferedJump)
            {
                return false;
            }

            ExecuteJump();
            _hasJumpRequested = false;
            _hasCoyoteTime = false;
            _coyoteTimeCounter = 0f;
            ResetJumpBuffer();
            return true;
        }

        private bool CanJumpNow()
        {
            bool canJumpFromGround = _groundChecker.IsGrounded && !_slopeChecker.IsUnclimbableSlope();
            bool canJumpFromCoyoteWindow = _hasCoyoteTime;
            return canJumpFromGround || canJumpFromCoyoteWindow;
        }

        private void ExecuteJump()
        {
            ResetVerticalVelocity();
            _rigidbody.AddForce(Vector3.up * CurrentJumpForce, ForceMode.VelocityChange);
            OnJump?.Invoke();
        }

        public void HandleJumpBuffer(float deltaTime)
        {
            UpdateJumpBufferTimer(deltaTime);
            ClearExpiredBufferOnLanding();
        }

        private void UpdateJumpBufferTimer(float deltaTime)
        {
            if (!_hasJumpBuffered || _groundChecker.IsGrounded)
            {
                return;
            }

            _jumpBufferTimer += deltaTime;

            if (_jumpBufferTimer > _jumpBufferTime)
            {
                ResetJumpBuffer();
            }
        }

        private void ClearExpiredBufferOnLanding()
        {
            if (!_groundChecker.JustLanded) return;

            if (!_hasJumpBuffered || _jumpBufferTimer > _jumpBufferTime)
            {
                ResetJumpBuffer();
            }
        }

        private void ResetJumpBuffer()
        {
            _hasJumpBuffered = false;
            _jumpBufferTimer = 0f;
        }

        private void ResetVerticalVelocity()
        {
            _rigidbody.linearVelocity = new Vector3(
                _rigidbody.linearVelocity.x,
                0f,
                _rigidbody.linearVelocity.z);
        }

        public void HandleCoyoteTime(float deltaTime)
        {
            if (ShouldStartCoyoteTime)
            {
                _hasCoyoteTime = true;
                _coyoteTimeCounter = _coyoteTime;
            }

            if (!_hasCoyoteTime) return;

            _coyoteTimeCounter -= deltaTime;

            if (_coyoteTimeCounter <= 0f)
            {
                _hasCoyoteTime = false;
                _coyoteTimeCounter = 0f;
            }
        }
    }
}