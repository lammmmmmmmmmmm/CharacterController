using UnityEngine;
using UnityEngine.Events;
using UnityEngine.Serialization;

namespace PhysicsCharacterController
{
    [RequireComponent(typeof(CapsuleCollider))]
    [RequireComponent(typeof(Rigidbody))]
    public class CharacterManager : MonoBehaviour
    {
        [Header("Movement specifics")]
        [Tooltip("Layers where the player can stand on")]
        [SerializeField] private LayerMask groundMask;
        [Tooltip("Base player speed")]
        public float movementSpeed = 14.4f;
        [Range(0f, 1f)]
        [Tooltip("Minimum input value to trigger movement")]
        public float crouchSpeedMultiplier = 0.248f;
        [Range(0.01f, 0.99f)]
        [Tooltip("Minimum input value to trigger movement")]
        public float movementThreshold = 0.01f;
        [Space(10)]
        [Tooltip("Speed up multiplier")]
        public float dampSpeedUp = 0.2f;
        [Tooltip("Speed down multiplier")]
        public float dampSpeedDown = 0.1f;

        [Header("Jump and gravity specifics")]
        [Tooltip("Jump velocity")]
        public float jumpVelocity = 20f;
        [Tooltip("Multiplier applied to gravity when the player is falling")]
        public float fallMultiplier = 1.5f;
        [Tooltip("Multiplier applied to gravity when the player is holding jump")]
        public float holdJumpMultiplier = 5f;
        [Range(0f, 1f)]
        [Tooltip("Player friction against floor")]
        public float frictionAgainstFloor = 0.3f;
        [Range(0.01f, 0.99f)]
        [Tooltip("Player friction against wall")]
        public float frictionAgainstWall = 0.839f;
        [Tooltip("Player can long jump")]
        public bool canLongJump = true;

        [Header("Slope and step specifics")]
        [Tooltip("Distance from the player feet used to check if the player is touching the ground")]
        public float groundCheckerThreshold = 0.1f;
        [Tooltip("Distance from the player feet used to check if the player is touching a slope")]
        public float slopeCheckerThreshold = 0.51f;
        [Tooltip("Distance from the player center used to check if the player is touching a step")]
        public float stepCheckerThreshold = 0.6f;
        [Space(10)]
        [Range(1f, 89f)]
        [Tooltip("Max climbable slope angle")]
        public float maxClimbableSlopeAngle = 53.6f;
        [Tooltip("Max climbable step height")]
        public float maxStepHeight = 0.74f;
        [Space(10)]
        [Tooltip("Speed multiplier based on slope angle")]
        public AnimationCurve speedMultiplierOnAngle = AnimationCurve.EaseInOut(0f, 0f, 1f, 1f);
        [Range(0.01f, 1f)]
        [Tooltip("Multiplier factor on climbable slope")]
        public float canSlideMultiplierCurve = 0.061f;
        [Range(0.01f, 1f)]
        [Tooltip("Multiplier factor on non climbable slope")]
        public float cantSlideMultiplierCurve = 0.039f;
        [Range(0.01f, 1f)]
        [Tooltip("Multiplier factor on step")]
        public float climbingStairsMultiplierCurve = 0.637f;
        
        [Space(10)]
        [Tooltip("Multiplier factor for gravity")]
        public float gravityMultiplier = 6f;
        [Tooltip("Multiplier factor for gravity used on change of normal")]
        public float gravityMultiplierOnSlideChange = 3f;
        [Tooltip("Multiplier factor for gravity used on non climbable slope")]
        public float gravityMultiplierIfUnclimbableSlope = 30f;
        [Space(10)]
        public bool lockOnSlope;

        [Header("Wall slide specifics")]
        [Tooltip("Distance from the player head used to check if the player is touching a wall")]
        public float wallCheckerThreshold = 0.8f;
        [Tooltip("Wall checker distance from the player center")]
        public float heightWallCheckerChecker = 0.5f;
        [Space(10)]
        [Tooltip("Multiplier used when the player is jumping from a wall")]
        public float jumpFromWallMultiplier = 30f;
        [Tooltip("Factor used to determine the height of the jump")]
        public float multiplierVerticalLeap = 1f;

        [Header("Sprint and crouch specifics")]
        [Tooltip("Sprint speed")]
        public float sprintSpeed = 20f;
        [Tooltip("Multiplier applied to the collider when player is crouching")]
        public float crouchHeightMultiplier = 0.5f;
        [Tooltip("FP camera head height")]
        public Vector3 povNormalHeadHeight = new(0f, 0.5f, -0.1f);
        [Tooltip("FP camera head height when crouching")]
        public Vector3 povCrouchHeadHeight = new(0f, -0.1f, -0.1f);

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

        private float _jumpGravityMultiplier = 1f;

        private bool _isGrounded;
        private bool _isTouchingSlope;
        private bool _isTouchingStep;
        private bool _isTouchingWall;
        private bool _isJumping;
        private bool _isCrouch;

        private Vector2 _axisInput;
        private bool _jump;
        private bool _jumpHold;
        private bool _sprint;
        private bool _crouch;

        [HideInInspector]
        public float targetAngle;
        private Rigidbody _rigidbody;
        private CapsuleCollider _collider;
        private float _originalColliderHeight;

        private Vector3 _currVelocity = Vector3.zero;
        private float _turnSmoothVelocity;
        private bool _lockRotation;
        private bool _lockToCamera;

        private void Awake()
        {
            _rigidbody = GetComponent<Rigidbody>();
            _collider = GetComponent<CapsuleCollider>();
            _originalColliderHeight = _collider.height;

            SetFriction(frictionAgainstFloor, true);
            _currentLockOnSlope = lockOnSlope;
        }

        private void Update()
        {
            _axisInput = input.axisInput;
            _jump = input.jump;
            _jumpHold = input.jumpHold;
            _sprint = input.sprint;
            _crouch = input.crouch;
        }

        private void FixedUpdate()
        {
            //local vectors
            CheckGrounded();
            CheckStep();
            CheckWall();
            CheckSlopeAndDirections();

            //movement
            HandleCrouchModel();
            Walk();

            if (!_lockToCamera) Rotation();
            else ForceRotation();

            Jump();

            //gravity
            ApplyGravity();

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
                if (RoundValue(stepLowerHit.normal.y) == 0 && !Physics.Raycast(
                        bottomStepPos + new Vector3(0f, maxStepHeight, 0f), _globalForward, out _,
                        stepCheckerThreshold + 0.05f, groundMask))
                {
                    //rigidbody.position -= new Vector3(0f, -stepSmooth, 0f);
                    tmpStep = true;
                }
            }

            if (Physics.Raycast(bottomStepPos, Quaternion.AngleAxis(45, transform.up) * _globalForward,
                    out var stepLowerHit45, stepCheckerThreshold, groundMask))
            {
                if (RoundValue(stepLowerHit45.normal.y) == 0 && !Physics.Raycast(
                        bottomStepPos + new Vector3(0f, maxStepHeight, 0f),
                        Quaternion.AngleAxis(45, Vector3.up) * _globalForward, out _,
                        stepCheckerThreshold + 0.05f, groundMask))
                {
                    //rigidbody.position -= new Vector3(0f, -stepSmooth, 0f);
                    tmpStep = true;
                }
            }

            if (Physics.Raycast(bottomStepPos, Quaternion.AngleAxis(-45, transform.up) * _globalForward,
                    out var stepLowerHitMinus45, stepCheckerThreshold, groundMask))
            {
                if (RoundValue(stepLowerHitMinus45.normal.y) == 0 && !Physics.Raycast(
                        bottomStepPos + new Vector3(0f, maxStepHeight, 0f),
                        Quaternion.AngleAxis(-45, Vector3.up) * _globalForward, out _,
                        stepCheckerThreshold + 0.05f, groundMask))
                {
                    //rigidbody.position -= new Vector3(0f, -stepSmooth, 0f);
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

                if (slopeHit.normal.y == 1)
                {
                    _forward = Quaternion.Euler(0f, targetAngle, 0f) * Vector3.forward;
                    _globalForward = _forward;
                    _reactionForward = _forward;

                    SetFriction(frictionAgainstFloor, true);
                    _currentLockOnSlope = lockOnSlope;

                    _currentSurfaceAngle = 0f;
                    _isTouchingSlope = false;
                }
                else
                {
                    //set forward
                    Vector3 tmpGlobalForward = transform.forward.normalized;
                    Vector3 tmpForward = new Vector3(tmpGlobalForward.x,
                        Vector3.ProjectOnPlane(transform.forward.normalized, slopeHit.normal).normalized.y,
                        tmpGlobalForward.z);
                    Vector3 tmpReactionForward =
                        new Vector3(tmpForward.x, tmpGlobalForward.y - tmpForward.y, tmpForward.z);

                    if (_currentSurfaceAngle <= maxClimbableSlopeAngle && !_isTouchingStep)
                    {
                        //set forward
                        _forward = tmpForward * ((speedMultiplierOnAngle.Evaluate(_currentSurfaceAngle / 90f) *
                                                  canSlideMultiplierCurve) + 1f);
                        _globalForward = tmpGlobalForward *
                                         ((speedMultiplierOnAngle.Evaluate(_currentSurfaceAngle / 90f) *
                                           canSlideMultiplierCurve) + 1f);
                        _reactionForward = tmpReactionForward *
                                           ((speedMultiplierOnAngle.Evaluate(_currentSurfaceAngle / 90f) *
                                             canSlideMultiplierCurve) + 1f);

                        SetFriction(frictionAgainstFloor, true);
                        _currentLockOnSlope = lockOnSlope;
                    }
                    else if (_isTouchingStep)
                    {
                        //set forward
                        _forward = tmpForward * ((speedMultiplierOnAngle.Evaluate(_currentSurfaceAngle / 90f) *
                                                  climbingStairsMultiplierCurve) + 1f);
                        _globalForward = tmpGlobalForward *
                                         ((speedMultiplierOnAngle.Evaluate(_currentSurfaceAngle / 90f) *
                                           climbingStairsMultiplierCurve) + 1f);
                        _reactionForward = tmpReactionForward *
                                           ((speedMultiplierOnAngle.Evaluate(_currentSurfaceAngle / 90f) *
                                             climbingStairsMultiplierCurve) + 1f);

                        SetFriction(frictionAgainstFloor, true);
                        _currentLockOnSlope = true;
                    }
                    else
                    {
                        //set forward
                        _forward = tmpForward * ((speedMultiplierOnAngle.Evaluate(_currentSurfaceAngle / 90f) *
                                                  cantSlideMultiplierCurve) + 1f);
                        _globalForward = tmpGlobalForward *
                                         ((speedMultiplierOnAngle.Evaluate(_currentSurfaceAngle / 90f) *
                                           cantSlideMultiplierCurve) + 1f);
                        _reactionForward = tmpReactionForward *
                                           ((speedMultiplierOnAngle.Evaluate(_currentSurfaceAngle / 90f) *
                                             cantSlideMultiplierCurve) + 1f);

                        SetFriction(0f, true);
                        _currentLockOnSlope = lockOnSlope;
                    }

                    _currentSurfaceAngle = Vector3.Angle(Vector3.up, slopeHit.normal);
                    _isTouchingSlope = true;
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

                SetFriction(frictionAgainstFloor, true);
                _currentLockOnSlope = lockOnSlope;
            }
        }
        #endregion

        #region Movement
        private void HandleCrouchModel()
        {
            if (_crouch && _isGrounded)
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

        private void Walk()
        {
            float crouchMultiplier = 1f;
            if (_isCrouch) crouchMultiplier = crouchSpeedMultiplier;

            if (_axisInput.magnitude > movementThreshold)
            {
                targetAngle = Mathf.Atan2(_axisInput.x, _axisInput.y) * Mathf.Rad2Deg +
                              characterCamera.transform.eulerAngles.y;

                if (!_sprint)
                {
                    _rigidbody.linearVelocity = Vector3.SmoothDamp(_rigidbody.linearVelocity,
                        _forward * (movementSpeed * crouchMultiplier), ref _currVelocity, dampSpeedUp);
                }
                else
                {
                    _rigidbody.linearVelocity = Vector3.SmoothDamp(_rigidbody.linearVelocity,
                        _forward * (sprintSpeed * crouchMultiplier), ref _currVelocity, dampSpeedUp);
                }
            }
            else
            {
                _rigidbody.linearVelocity = Vector3.SmoothDamp(_rigidbody.linearVelocity,
                    Vector3.zero * crouchMultiplier, ref _currVelocity, dampSpeedDown);
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
            //jumped
            if (_jump && _isGrounded &&
                (_isTouchingSlope && _currentSurfaceAngle <= maxClimbableSlopeAngle || !_isTouchingSlope) && !_isTouchingWall)
            {
                _rigidbody.linearVelocity += Vector3.up * jumpVelocity;
                _isJumping = true;
            }
            //jumped from wall
            else if (_jump && !_isGrounded && _isTouchingWall)
            {
                _rigidbody.linearVelocity += _wallNormal * jumpFromWallMultiplier +
                                             Vector3.up * (jumpFromWallMultiplier * multiplierVerticalLeap);
                _isJumping = true;

                targetAngle = Mathf.Atan2(_wallNormal.x, _wallNormal.z) * Mathf.Rad2Deg;

                _forward = _wallNormal;
                _globalForward = _forward;
                _reactionForward = _forward;
            }

            //is falling
            if (_rigidbody.linearVelocity.y < 0 && !_isGrounded)
            {
                _jumpGravityMultiplier = fallMultiplier;
            }
            else if (_rigidbody.linearVelocity.y > 0.1f && (_currentSurfaceAngle <= maxClimbableSlopeAngle || _isTouchingStep))
            {
                //is short jumping
                if (!_jumpHold || !canLongJump) _jumpGravityMultiplier = 1f;
                //is long jumping
                else _jumpGravityMultiplier = 1f / holdJumpMultiplier;
            }
            else
            {
                _isJumping = false;
                _jumpGravityMultiplier = 1f;
            }
        }
        #endregion

        #region Gravity
        private void ApplyGravity()
        {
            Vector3 gravity;

            if (_currentLockOnSlope || _isTouchingStep)
            {
                gravity = _down * (gravityMultiplier * -Physics.gravity.y * _jumpGravityMultiplier);
            }
            else
            {
                gravity = _globalDown * (gravityMultiplier * -Physics.gravity.y * _jumpGravityMultiplier);
            }

            //avoid little jump
            if (_groundNormal.y != 1 && _groundNormal.y != 0 && _isTouchingSlope && _prevGroundNormal != _groundNormal)
            {
                //Debug.Log("Added correction jump on slope");
                gravity *= gravityMultiplierOnSlideChange;
            }

            //slide if angle too big
            if (_groundNormal.y != 1 && _groundNormal.y != 0 && _currentSurfaceAngle > maxClimbableSlopeAngle && !_isTouchingStep)
            {
                //Debug.Log("Slope angle too high, character is sliding");
                if (_currentSurfaceAngle is > 0f and <= 30f)
                    gravity = _globalDown * (gravityMultiplierIfUnclimbableSlope * -Physics.gravity.y);
                else if (_currentSurfaceAngle is > 30f and <= 89f)
                    gravity = _globalDown * gravityMultiplierIfUnclimbableSlope / 2f * -Physics.gravity.y;
            }

            //friction when touching wall
            if (_isTouchingWall && _rigidbody.linearVelocity.y < 0) gravity *= frictionAgainstWall;

            _rigidbody.AddForce(gravity);
        }
        #endregion

        #region Events
        private void UpdateEvents()
        {
            if (_jump && _isGrounded && (_isTouchingSlope && _currentSurfaceAngle <= maxClimbableSlopeAngle ||
                                         !_isTouchingSlope) ||
                (_jump && !_isGrounded && _isTouchingWall)) onJump.Invoke();
            if (_isGrounded && !_prevIsGrounded && _rigidbody.linearVelocity.y > -minimumVerticalSpeedToLandEvent)
                onLand.Invoke();
            if (Mathf.Abs(_rigidbody.linearVelocity.x) + Mathf.Abs(_rigidbody.linearVelocity.z) >
                minimumHorizontalSpeedToFastEvent) onFast.Invoke();
            if (_isTouchingWall && _rigidbody.linearVelocity.y < 0) onWallSlide.Invoke();
            if (_sprint) onSprint.Invoke();
            if (_isCrouch) onCrouch.Invoke();
        }
        #endregion

        #region Friction and Round
        private void SetFriction(float frictionWall, bool isMinimum)
        {
            _collider.material.dynamicFriction = 0.6f * frictionWall;
            _collider.material.staticFriction = 0.6f * frictionWall;

            _collider.material.frictionCombine = isMinimum ? PhysicsMaterialCombine.Minimum : PhysicsMaterialCombine.Maximum;
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
                Gizmos.color = Color.black;
                Gizmos.DrawLine(topWallPos, topWallPos + _globalForward * wallCheckerThreshold);

                Gizmos.color = Color.black;
                Gizmos.DrawLine(topWallPos,
                    topWallPos + Quaternion.AngleAxis(45, transform.up) * (_globalForward * wallCheckerThreshold));

                Gizmos.color = Color.black;
                Gizmos.DrawLine(topWallPos,
                    topWallPos + Quaternion.AngleAxis(90, transform.up) * (_globalForward * wallCheckerThreshold));

                Gizmos.color = Color.black;
                Gizmos.DrawLine(topWallPos,
                    topWallPos + Quaternion.AngleAxis(135, transform.up) * (_globalForward * wallCheckerThreshold));

                Gizmos.color = Color.black;
                Gizmos.DrawLine(topWallPos,
                    topWallPos + Quaternion.AngleAxis(180, transform.up) * (_globalForward * wallCheckerThreshold));

                Gizmos.color = Color.black;
                Gizmos.DrawLine(topWallPos,
                    topWallPos + Quaternion.AngleAxis(225, transform.up) * (_globalForward * wallCheckerThreshold));

                Gizmos.color = Color.black;
                Gizmos.DrawLine(topWallPos,
                    topWallPos + Quaternion.AngleAxis(270, transform.up) * (_globalForward * wallCheckerThreshold));

                Gizmos.color = Color.black;
                Gizmos.DrawLine(topWallPos,
                    topWallPos + Quaternion.AngleAxis(315, transform.up) * (_globalForward * wallCheckerThreshold));
            }
        }
        #endregion
    }
}