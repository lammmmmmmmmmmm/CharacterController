using UnityEngine;
using UnityEngine.Events;
using UnityEngine.Serialization;

namespace PhysicsCharacterController
{
    [RequireComponent(typeof(CapsuleCollider))]
    [RequireComponent(typeof(Rigidbody))]
    public class CharacterManager : MonoBehaviour
    {
        [Header("Movement")]
        [SerializeField] private LayerMask groundMask;
        public float movementSpeed = 14f;
        public float sprintSpeed = 20f;
        [Range(0f, 1f)]
        public float crouchSpeedMultiplier = 0.248f;
        [Range(0.01f, 0.99f)]
        [Tooltip("Minimum input value to trigger movement")]
        public float movementThreshold = 0.01f;
        [SerializeField] private float acceleration = 10f;
        [SerializeField] private float deceleration = 10f;

        [Header("Crouch")]
        [Tooltip("Multiplier applied to the collider when player is crouching")]
        public float crouchHeightMultiplier = 0.5f;
        [Tooltip("FP camera head height")]
        public Vector3 povNormalHeadHeight = new(0f, 0.5f, -0.1f);
        [Tooltip("FP camera head height when crouching")]
        public Vector3 povCrouchHeadHeight = new(0f, -0.1f, -0.1f);
        
        [Header("Jump")]
        [SerializeField] private float maxJumpHeight = 2f;
        [SerializeField] private float timeToReachMaxJumpHeightInSeconds = 0.5f;
        [SerializeField] private AnimationCurve jumpCurve;
        [Tooltip("Time window in seconds to buffer jump input when not grounded")]
        [SerializeField] private float jumpBufferTime = 0.2f;

        [Header("Slope and step")]
        [Tooltip("Distance from the player feet used to check if the player is touching the ground")]
        public float groundCheckerThreshold = 0.1f;
        [Tooltip("Distance from the player feet used to check if the player is touching a slope")]
        public float slopeCheckerThreshold = 0.51f;
        [Tooltip("Distance from the player center used to check if the player is touching a step")]
        public float stepCheckerThreshold = 0.6f;
        [Tooltip("If true, the player won't slide down on walkable slopes")]
        public bool lockOnSlope;
        [Range(1f, 89f)]
        [Tooltip("Max climbable slope angle")]
        public float maxClimbableSlopeAngle = 53.6f;
        [Tooltip("Max climbable step height")]
        public float maxStepHeight = 0.74f;
        [Tooltip("Animation curve for slope speed reduction based on angle (0-90 degrees)")]
        public AnimationCurve slopeSpeedCurve = AnimationCurve.Linear(0f, 1f, 90f, 0.3f);

        [Header("Gravity")]
        [Tooltip("Multiplier factor for gravity")]
        public float gravityMultiplier = 6f;
        [Tooltip("Multiplier factor for gravity used on change of normal")]
        public float gravityMultiplierOnSlideChange = 3f;
        [Tooltip("Multiplier factor for gravity used on non climbable slope")]
        public float gravityMultiplierIfUnclimbableSlope = 30f;
        [Space(10)]

        [Header("Wall slide")]
        [Tooltip("Distance from the player head used to check if the player is touching a wall")]
        public float wallCheckerThreshold = 0.8f;
        [Tooltip("Wall checker distance from the player center")]
        public float heightWallCheckerChecker = 0.5f;

        [Header("References")]
        [Tooltip("Character camera")]
        public GameObject characterCamera;
        [Tooltip("Character model")]
        public GameObject characterModel;
        [Tooltip("Character rotation speed when the forward direction is changed")]
        public float characterModelRotationSmooth = 0.1f;
        [Space(10)]
        [Tooltip("Default character mesh")]
        public GameObject meshCharacter;
        [Tooltip("Crouch character mesh")]
        public GameObject meshCharacterCrouch;
        [Tooltip("Head reference")]
        public Transform headPoint;
        [Space(10)]
        [Tooltip("Input reference")]
        public InputReader input;
        [Space(10)]
        public bool debug = true;

        [FormerlySerializedAs("OnJump")]
        [Header("Events")]
        [SerializeField] private UnityEvent onJump;
        public float minimumVerticalSpeedToLandEvent;
        [SerializeField] private UnityEvent onLand;
        public float minimumHorizontalSpeedToFastEvent;
        [SerializeField] private UnityEvent onFast;
        [SerializeField] private UnityEvent onWallSlide;
        [SerializeField] private UnityEvent onSprint;
        [SerializeField] private UnityEvent onCrouch;
        
        private Vector3 _forward;
        private Vector3 _globalForward;
        private Vector3 _reactionForward;
        private Vector3 _down;
        private Vector3 _globalDown;
        private Vector3 _reactionGlobalDown;

        private float _currentSurfaceAngle;
        private bool _currentLockOnSlope;

        private Vector3 _wallNormal;
        private Vector3 _groundNormal;
        private Vector3 _prevGroundNormal;
        private bool _prevIsGrounded;

        private bool _isGrounded;
        private bool _isTouchingSlope;
        private bool _isTouchingStep;
        private bool _isTouchingWall;
        private bool _isCrouch;

        private Vector2 _axisInput;
        private bool _jumpInput;
        private bool _sprintInput;
        private bool _crouchInput;

        [HideInInspector]
        public float targetAngle;
        private Rigidbody _rigidbody;
        private CapsuleCollider _collider;
        private float _originalColliderHeight;

        private float _turnSmoothVelocity;
        private bool _lockRotation;
        private bool _lockToCamera;
        
        private bool _isAccelerating;
        private float _currentSpeed;

        private float _jumpUpTimer;
        private float _yPosBeforeJump;
        private bool _isJumping;
        
        private float _jumpBufferTimer;
        private bool _hasJumpBuffered;

        private void Awake()
        {
            _rigidbody = GetComponent<Rigidbody>();
            _collider = GetComponent<CapsuleCollider>();
            _originalColliderHeight = _collider.height;

            SetNoFriction();
            _currentLockOnSlope = lockOnSlope;
        }

        private void Update()
        {
            _axisInput = input.axisInput;
            _jumpInput = input.jump;
            _sprintInput = input.sprint;
            _crouchInput = input.crouch;
            
            HandleJumpBuffer();
        }

        private void FixedUpdate()
        {
            //local vectors
            CheckGrounded();
            CheckStep();
            // CheckWall();
            CheckSlopeAndDirections();

            //movement
            HandleCrouchModel();
            Move();

            if (!_lockToCamera) Rotation();
            else ForceRotation();

            Jump();

            //gravity
            if (!_isJumping)
            {
                ApplyGravity();
            }

            //events
            UpdateEvents();
        }

        #region Checks
        private void CheckGrounded()
        {
            _prevIsGrounded = _isGrounded;
            _isGrounded = Physics.CheckSphere(transform.position - new Vector3(0, _originalColliderHeight / 2f, 0),
                groundCheckerThreshold, groundMask);
        }

        private void CheckStep()
        {
            bool tmpStep = false;
            Vector3 bottomStepPos = transform.position - new Vector3(0f, _originalColliderHeight / 2f, 0f) +
                                    new Vector3(0f, 0.05f, 0f);

            if (Physics.Raycast(bottomStepPos, _globalForward, out var stepLowerHit, stepCheckerThreshold, groundMask))
            {
                if (RoundValue(stepLowerHit.normal.y) == 0
                && !Physics.Raycast(
                        bottomStepPos + new Vector3(0f, maxStepHeight, 0f), _globalForward, out _,
                        stepCheckerThreshold + 0.05f, groundMask))
                {
                    tmpStep = true;
                }
            }

            if (Physics.Raycast(bottomStepPos, Quaternion.AngleAxis(45, transform.up) * _globalForward,
                    out var stepLowerHit45, stepCheckerThreshold, groundMask))
            {
                if (RoundValue(stepLowerHit45.normal.y) == 0
                && !Physics.Raycast(
                        bottomStepPos + new Vector3(0f, maxStepHeight, 0f),
                        Quaternion.AngleAxis(45, Vector3.up) * _globalForward, out _,
                        stepCheckerThreshold + 0.05f, groundMask))
                {
                    tmpStep = true;
                }
            }

            if (Physics.Raycast(bottomStepPos, Quaternion.AngleAxis(-45, transform.up) * _globalForward,
                    out var stepLowerHitMinus45, stepCheckerThreshold, groundMask))
            {
                if (RoundValue(stepLowerHitMinus45.normal.y) == 0
                && !Physics.Raycast(
                        bottomStepPos + new Vector3(0f, maxStepHeight, 0f),
                        Quaternion.AngleAxis(-45, Vector3.up) * _globalForward, out _,
                        stepCheckerThreshold + 0.05f, groundMask))
                {
                    tmpStep = true;
                }
            }

            _isTouchingStep = tmpStep;
        }

        private void CheckWall()
        {
            bool tmpWall = false;
            Vector3 tmpWallNormal = Vector3.zero;
            Vector3 topWallPos = new Vector3(transform.position.x, transform.position.y + heightWallCheckerChecker,
                transform.position.z);

            if (Physics.Raycast(topWallPos, _globalForward, out var wallHit, wallCheckerThreshold, groundMask))
            {
                tmpWallNormal = wallHit.normal;
                tmpWall = true;
            }
            else if (Physics.Raycast(topWallPos, Quaternion.AngleAxis(45, transform.up) * _globalForward, out wallHit,
                         wallCheckerThreshold, groundMask))
            {
                tmpWallNormal = wallHit.normal;
                tmpWall = true;
            }
            else if (Physics.Raycast(topWallPos, Quaternion.AngleAxis(90, transform.up) * _globalForward, out wallHit,
                         wallCheckerThreshold, groundMask))
            {
                tmpWallNormal = wallHit.normal;
                tmpWall = true;
            }
            else if (Physics.Raycast(topWallPos, Quaternion.AngleAxis(135, transform.up) * _globalForward, out wallHit,
                         wallCheckerThreshold, groundMask))
            {
                tmpWallNormal = wallHit.normal;
                tmpWall = true;
            }
            else if (Physics.Raycast(topWallPos, Quaternion.AngleAxis(180, transform.up) * _globalForward, out wallHit,
                         wallCheckerThreshold, groundMask))
            {
                tmpWallNormal = wallHit.normal;
                tmpWall = true;
            }
            else if (Physics.Raycast(topWallPos, Quaternion.AngleAxis(225, transform.up) * _globalForward, out wallHit,
                         wallCheckerThreshold, groundMask))
            {
                tmpWallNormal = wallHit.normal;
                tmpWall = true;
            }
            else if (Physics.Raycast(topWallPos, Quaternion.AngleAxis(270, transform.up) * _globalForward, out wallHit,
                         wallCheckerThreshold, groundMask))
            {
                tmpWallNormal = wallHit.normal;
                tmpWall = true;
            }
            else if (Physics.Raycast(topWallPos, Quaternion.AngleAxis(315, transform.up) * _globalForward, out wallHit,
                         wallCheckerThreshold, groundMask))
            {
                tmpWallNormal = wallHit.normal;
                tmpWall = true;
            }

            _isTouchingWall = tmpWall;
            _wallNormal = tmpWallNormal;
        }

        private void CheckSlopeAndDirections()
        {
            _prevGroundNormal = _groundNormal;

            if (Physics.SphereCast(transform.position, slopeCheckerThreshold, Vector3.down, out var slopeHit,
                    _originalColliderHeight / 2f + 0.5f, groundMask))
            {
                _groundNormal = slopeHit.normal;

                if (Mathf.Approximately(slopeHit.normal.y, 1))
                {
                    // flat ground
                    _forward = Quaternion.Euler(0f, targetAngle, 0f) * Vector3.forward;
                    _globalForward = _forward;
                    _reactionForward = _forward;

                    _currentLockOnSlope = lockOnSlope;

                    _currentSurfaceAngle = 0f;
                    _isTouchingSlope = false;
                }
                else
                {
                    // slope
                    _currentSurfaceAngle = Vector3.Angle(Vector3.up, slopeHit.normal);
                    _isTouchingSlope = true;
                    
                    _globalForward = transform.forward.normalized;
                    _forward = new Vector3(_globalForward.x,
                        Vector3.ProjectOnPlane(transform.forward.normalized, slopeHit.normal).normalized.y,
                        _globalForward.z);
                    _reactionForward = new Vector3(_forward.x, _globalForward.y - _forward.y, _forward.z);

                    if (_currentSurfaceAngle <= maxClimbableSlopeAngle && !_isTouchingStep)
                    {
                        _currentLockOnSlope = lockOnSlope;
                    }
                    else if (_isTouchingStep)
                    {
                        _currentLockOnSlope = true;
                    }
                    else
                    {
                        _currentLockOnSlope = false;
                    }
                }

                //set down
                _down = Vector3.Project(Vector3.down, slopeHit.normal);
                _globalDown = Vector3.down.normalized;
                _reactionGlobalDown = Vector3.up.normalized;
            }
            else
            {
                _groundNormal = Vector3.zero;

                _forward = Vector3.ProjectOnPlane(transform.forward, slopeHit.normal).normalized;
                _globalForward = _forward;
                _reactionForward = _forward;

                //set down
                _down = Vector3.down.normalized;
                _globalDown = Vector3.down.normalized;
                _reactionGlobalDown = Vector3.up.normalized;

                _currentLockOnSlope = lockOnSlope;
            }
        }
        #endregion

        #region Jump Buffer
        private void HandleJumpBuffer()
        {
            // If jump input is pressed
            if (_jumpInput)
            {
                if (_isGrounded && CanJump())
                {
                    // Normal jump - player is grounded and can jump
                    ExecuteJump();
                }
                else if (!_isGrounded)
                {
                    // Player is not grounded, start jump buffer
                    _hasJumpBuffered = true;
                    _jumpBufferTimer = 0f;
                }
            }
            
            // Update jump buffer timer if buffered
            if (_hasJumpBuffered && !_isGrounded)
            {
                _jumpBufferTimer += Time.deltaTime;
                
                // Clear buffer if time exceeded
                if (_jumpBufferTimer > jumpBufferTime)
                {
                    _hasJumpBuffered = false;
                    _jumpBufferTimer = 0f;
                }
            }
            
            // Check if player just landed and has buffered jump
            if (_hasJumpBuffered && _isGrounded && !_prevIsGrounded && CanJump())
            {
                // Execute buffered jump
                ExecuteJump();
                _hasJumpBuffered = false;
                _jumpBufferTimer = 0f;
            }
            
            // Clear buffer if player lands but buffer expired
            if (_isGrounded && !_prevIsGrounded)
            {
                if (!_hasJumpBuffered || _jumpBufferTimer > jumpBufferTime)
                {
                    _hasJumpBuffered = false;
                    _jumpBufferTimer = 0f;
                }
            }
        }
        
        private bool CanJump()
        {
            return (_isTouchingSlope && _currentSurfaceAngle <= maxClimbableSlopeAngle || 
                    !_isTouchingSlope || _isTouchingStep) && !_isTouchingWall;
        }
        
        private void ExecuteJump()
        {
            if (!_isJumping)
            {
                _isJumping = true;
                _jumpUpTimer = 0f;
                _yPosBeforeJump = _rigidbody.position.y;
            }
        }
        #endregion

        #region Movement
        private void HandleCrouchModel()
        {
            if (_crouchInput && _isGrounded)
            {
                _isCrouch = true;
                if (meshCharacterCrouch && meshCharacter) meshCharacter.SetActive(false);
                if (meshCharacterCrouch) meshCharacterCrouch.SetActive(true);

                float newHeight = _originalColliderHeight * crouchHeightMultiplier;
                _collider.height = newHeight;
                _collider.center = new Vector3(0f, -newHeight * crouchHeightMultiplier, 0f);

                headPoint.position = new Vector3(transform.position.x + povCrouchHeadHeight.x,
                    transform.position.y + povCrouchHeadHeight.y, transform.position.z + povCrouchHeadHeight.z);
            }
            else
            {
                _isCrouch = false;
                if (meshCharacterCrouch && meshCharacter) meshCharacter.SetActive(true);
                if (meshCharacterCrouch) meshCharacterCrouch.SetActive(false);

                _collider.height = _originalColliderHeight;
                _collider.center = Vector3.zero;

                headPoint.position = new Vector3(transform.position.x + povNormalHeadHeight.x,
                    transform.position.y + povNormalHeadHeight.y, transform.position.z + povNormalHeadHeight.z);
            }
        }

        private void Move()
        {
            float crouchMultiplier = 1f;
            if (_isCrouch) crouchMultiplier = crouchSpeedMultiplier;

            if (_axisInput.magnitude > movementThreshold)
            {
                targetAngle = Mathf.Atan2(_axisInput.x, _axisInput.y) * Mathf.Rad2Deg +
                              characterCamera.transform.eulerAngles.y;

                // Calculate target velocity based on sprint state
                float targetSpeed = _sprintInput ? sprintSpeed : movementSpeed;
                
                // Apply slope speed reduction when going up slopes
                float finalSlopeMultiplier = 1f;
                if (_isTouchingSlope && !_isTouchingStep)
                {
                    Vector3 intendedDirection = Quaternion.Euler(0f, targetAngle, 0f) * Vector3.forward;
                    Vector3 horizontalIntendedDirection = Vector3.ProjectOnPlane(intendedDirection, Vector3.up).normalized;
                    Vector3 horizontalSlopeDirection = Vector3.ProjectOnPlane(-_groundNormal, Vector3.up).normalized;
                    float dotProduct = Vector3.Dot(horizontalIntendedDirection, horizontalSlopeDirection);

                    // If moving up the slope (dotProduct > 0), apply speed reduction
                    if (dotProduct > 0.1f)
                    {
                        if (_currentSurfaceAngle <= maxClimbableSlopeAngle)
                        {
                            finalSlopeMultiplier = slopeSpeedCurve.Evaluate(_currentSurfaceAngle);
                        }

                        if (_currentSurfaceAngle > maxClimbableSlopeAngle)
                        {
                            _currentSpeed = 0f;
                            targetSpeed = 0f;
                        }
                    }
                }
                
                targetSpeed *= finalSlopeMultiplier * crouchMultiplier;
                
                if (!_isAccelerating)
                {
                    _isAccelerating = true;
                }

                _currentSpeed = Mathf.MoveTowards(_currentSpeed, targetSpeed, acceleration * Time.fixedDeltaTime);
                Vector3 targetVelocity = _forward * _currentSpeed;

                //TODO: fix step climbing. Apply a constant upward velocity instead of adding each frame
                float upVel = 0f;
                if (_isTouchingStep)
                {
                    upVel = 1f;
                }

                _rigidbody.linearVelocity = new Vector3(targetVelocity.x, _rigidbody.linearVelocity.y + upVel, targetVelocity.z);
            }
            else
            {
                if (_isAccelerating)
                {
                    _isAccelerating = false;
                }

                _currentSpeed = Mathf.MoveTowards(_currentSpeed, 0f, deceleration * Time.fixedDeltaTime);
                Vector3 targetVelocity = _forward * _currentSpeed;
                
                _rigidbody.linearVelocity = new Vector3(targetVelocity.x, _rigidbody.linearVelocity.y, targetVelocity.z);
            }
        }

        private void Rotation()
        {
            float angle = Mathf.SmoothDampAngle(characterModel.transform.eulerAngles.y, targetAngle,
                ref _turnSmoothVelocity, characterModelRotationSmooth);
            transform.rotation = Quaternion.Euler(0f, targetAngle, 0f);

            if (!_lockRotation)
            {
                characterModel.transform.rotation = Quaternion.Euler(0f, angle, 0f);
            }
            else
            {
                var lookPos = -_wallNormal;
                lookPos.y = 0;
                var rotation = Quaternion.LookRotation(lookPos);
                characterModel.transform.rotation = rotation;
            }
        }

        public void ForceRotation()
        {
            characterModel.transform.rotation =
                Quaternion.Euler(0f, characterCamera.transform.rotation.eulerAngles.y, 0f);
        }

        private void Jump()
        {
            //TODO: add ceiling check
            if (_isJumping)
            {
                // Update jump up timer
                _jumpUpTimer += Time.fixedDeltaTime;
                float jumpUpProgress = Mathf.Clamp01(_jumpUpTimer / timeToReachMaxJumpHeightInSeconds);
                float curveValue = jumpCurve?.Evaluate(jumpUpProgress) ?? jumpUpProgress;

                float targetHeight = _yPosBeforeJump + maxJumpHeight * curveValue;
                _rigidbody.position = new Vector3(_rigidbody.position.x, targetHeight, _rigidbody.position.z);
                
                if (curveValue >= 1f)
                {
                    // Jump is complete, start falling
                    _jumpUpTimer = 0f;
                    _isJumping = false;
                }
            }
        }
        #endregion

        #region Gravity
        private void ApplyGravity()
        {
            Vector3 gravity;

            if (_currentLockOnSlope || _isTouchingStep)
            {
                gravity = _down * (gravityMultiplier * -Physics.gravity.y);
            }
            else
            {
                gravity = _globalDown * (gravityMultiplier * -Physics.gravity.y);
            }

            //avoid little jump
            if (!Mathf.Approximately(_groundNormal.y, 1) && _groundNormal.y != 0 && _isTouchingSlope && _prevGroundNormal != _groundNormal)
            {
                //Debug.Log("Added correction jump on slope");
                gravity *= gravityMultiplierOnSlideChange;
            }

            //slide if angle too big
            if (!Mathf.Approximately(_groundNormal.y, 1) && _groundNormal.y != 0 && _currentSurfaceAngle > maxClimbableSlopeAngle && !_isTouchingStep)
            {
                if (_currentSurfaceAngle is > 0f and <= 30f)
                {
                    gravity = _globalDown * (gravityMultiplierIfUnclimbableSlope * -Physics.gravity.y);
                }
                else if (_currentSurfaceAngle is > 30f and <= 89f)
                {
                    gravity = _globalDown * (gravityMultiplierIfUnclimbableSlope * 0.5f * -Physics.gravity.y);
                }
            }

            _rigidbody.AddForce(gravity);
        }
        #endregion

        #region Events
        private void UpdateEvents()
        {
            if ((_jumpInput && _isGrounded && CanJump()) || 
                (_hasJumpBuffered && _isGrounded && !_prevIsGrounded && CanJump()) ||
                (_jumpInput && !_isGrounded && _isTouchingWall)) onJump.Invoke();
            if (_isGrounded && !_prevIsGrounded && _rigidbody.linearVelocity.y > -minimumVerticalSpeedToLandEvent)
                onLand.Invoke();
            if (Mathf.Abs(_rigidbody.linearVelocity.x) + Mathf.Abs(_rigidbody.linearVelocity.z) >
                minimumHorizontalSpeedToFastEvent) onFast.Invoke();
            if (_isTouchingWall && _rigidbody.linearVelocity.y < 0) onWallSlide.Invoke();
            if (_sprintInput) onSprint.Invoke();
            if (_isCrouch) onCrouch.Invoke();
        }
        #endregion

        #region Friction and Round
        private void SetNoFriction()
        {
            _collider.material.dynamicFriction = 0f;
            _collider.material.staticFriction = 0f;

            _collider.material.frictionCombine = PhysicsMaterialCombine.Minimum;
        }

        private float RoundValue(float value)
        {
            float unit = Mathf.Round(value);

            if (value - unit < 0.000001f && value - unit > -0.000001f) return unit;
            return value;
        }
        #endregion

        #region GettersSetters
        public bool GetGrounded()
        {
            return _isGrounded;
        }

        public bool GetTouchingSlope()
        {
            return _isTouchingSlope;
        }

        public bool GetTouchingStep()
        {
            return _isTouchingStep;
        }

        public bool GetTouchingWall()
        {
            return _isTouchingWall;
        }

        public bool GetJumping()
        {
            return _isJumping;
        }

        public bool GetCrouching()
        {
            return _isCrouch;
        }

        public float GetOriginalColliderHeight()
        {
            return _originalColliderHeight;
        }

        public void SetLockRotation(bool @lock)
        {
            _lockRotation = @lock;
        }

        public void SetLockToCamera(bool lockToCamera)
        {
            _lockToCamera = lockToCamera;
            if (!lockToCamera) targetAngle = characterModel.transform.eulerAngles.y;
        }
        #endregion

        #region Gizmos
        private void OnDrawGizmos()
        {
            if (debug)
            {
                _rigidbody = GetComponent<Rigidbody>();
                _collider = GetComponent<CapsuleCollider>();

                Vector3 bottomStepPos = transform.position - new Vector3(0f, _originalColliderHeight / 2f, 0f) +
                                        new Vector3(0f, 0.05f, 0f);
                Vector3 topWallPos = new Vector3(transform.position.x, transform.position.y + heightWallCheckerChecker,
                    transform.position.z);

                //ground and slope
                Gizmos.color = Color.blue;
                Gizmos.DrawWireSphere(transform.position - new Vector3(0, _originalColliderHeight / 2f, 0),
                    groundCheckerThreshold);

                Gizmos.color = Color.green;
                Gizmos.DrawWireSphere(transform.position - new Vector3(0, _originalColliderHeight / 2f, 0),
                    slopeCheckerThreshold);

                //direction
                Gizmos.color = Color.blue;
                Gizmos.DrawLine(transform.position, transform.position + _forward * 2f);

                Gizmos.color = Color.cyan;
                Gizmos.DrawLine(transform.position, transform.position + _globalForward * 2);

                Gizmos.color = Color.cyan;
                Gizmos.DrawLine(transform.position, transform.position + _reactionForward * 2f);

                Gizmos.color = Color.red;
                Gizmos.DrawLine(transform.position, transform.position + _down * 2f);

                Gizmos.color = Color.magenta;
                Gizmos.DrawLine(transform.position, transform.position + _globalDown * 2f);

                Gizmos.color = Color.magenta;
                Gizmos.DrawLine(transform.position, transform.position + _reactionGlobalDown * 2f);

                //step check
                Gizmos.color = Color.black;
                Gizmos.DrawLine(bottomStepPos, bottomStepPos + _globalForward * stepCheckerThreshold);

                Gizmos.color = Color.black;
                Gizmos.DrawLine(bottomStepPos + new Vector3(0f, maxStepHeight, 0f),
                    bottomStepPos + new Vector3(0f, maxStepHeight, 0f) +
                    _globalForward * (stepCheckerThreshold + 0.05f));

                Gizmos.color = Color.black;
                Gizmos.DrawLine(bottomStepPos,
                    bottomStepPos + Quaternion.AngleAxis(45, transform.up) * (_globalForward * stepCheckerThreshold));

                Gizmos.color = Color.black;
                Gizmos.DrawLine(bottomStepPos + new Vector3(0f, maxStepHeight, 0f),
                    bottomStepPos + Quaternion.AngleAxis(45, Vector3.up) * (_globalForward * stepCheckerThreshold) +
                    new Vector3(0f, maxStepHeight, 0f));

                Gizmos.color = Color.black;
                Gizmos.DrawLine(bottomStepPos,
                    bottomStepPos + Quaternion.AngleAxis(-45, transform.up) * (_globalForward * stepCheckerThreshold));

                Gizmos.color = Color.black;
                Gizmos.DrawLine(bottomStepPos + new Vector3(0f, maxStepHeight, 0f),
                    bottomStepPos + Quaternion.AngleAxis(-45, Vector3.up) * (_globalForward * stepCheckerThreshold) +
                    new Vector3(0f, maxStepHeight, 0f));

                //wall check
                // Gizmos.color = Color.black;
                // Gizmos.DrawLine(topWallPos, topWallPos + _globalForward * wallCheckerThreshold);

                // Gizmos.color = Color.black;
                // Gizmos.DrawLine(topWallPos,
                //     topWallPos + Quaternion.AngleAxis(45, transform.up) * (_globalForward * wallCheckerThreshold));

                // Gizmos.color = Color.black;
                // Gizmos.DrawLine(topWallPos,
                //     topWallPos + Quaternion.AngleAxis(90, transform.up) * (_globalForward * wallCheckerThreshold));

                // Gizmos.color = Color.black;
                // Gizmos.DrawLine(topWallPos,
                //     topWallPos + Quaternion.AngleAxis(135, transform.up) * (_globalForward * wallCheckerThreshold));

                // Gizmos.color = Color.black;
                // Gizmos.DrawLine(topWallPos,
                //     topWallPos + Quaternion.AngleAxis(180, transform.up) * (_globalForward * wallCheckerThreshold));

                // Gizmos.color = Color.black;
                // Gizmos.DrawLine(topWallPos,
                //     topWallPos + Quaternion.AngleAxis(225, transform.up) * (_globalForward * wallCheckerThreshold));

                // Gizmos.color = Color.black;
                // Gizmos.DrawLine(topWallPos,
                //     topWallPos + Quaternion.AngleAxis(270, transform.up) * (_globalForward * wallCheckerThreshold));

                // Gizmos.color = Color.black;
                // Gizmos.DrawLine(topWallPos,
                //     topWallPos + Quaternion.AngleAxis(315, transform.up) * (_globalForward * wallCheckerThreshold));
            }
        }
        #endregion
    }
}