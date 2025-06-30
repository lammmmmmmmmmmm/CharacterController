using UnityEngine;
using UnityEngine.Events;
using UnityEngine.InputSystem;

namespace PhysicsCharacterController
{
    public class InputReader : MonoBehaviour
    {
        [Header("Input specs")]
        public UnityEvent changedInputToMouseAndKeyboard;
        public UnityEvent changedInputToGamepad;

        [Header("Enable inputs")]
        public bool enableJump = true;
        public bool enableCrouch = true;
        public bool enableSprint = true;

        [HideInInspector]
        public Vector2 axisInput;
        [HideInInspector]
        public bool jump;
        [HideInInspector]
        public bool jumpHold;
        [HideInInspector]
        public bool sprint;
        [HideInInspector]
        public bool crouch;

        private bool _hasJumped;
        private bool _skippedFrame;
        private bool _isMouseAndKeyboard = true;
        private bool _oldInput = true;

        private MovementActions _movementActions;

        private void Awake()
        {
            _movementActions = new MovementActions();

            _movementActions.Gameplay.Movement.performed += OnMove;

            _movementActions.Gameplay.Jump.performed += _ => OnJump();
            _movementActions.Gameplay.Jump.canceled += _ => JumpEnded();

            _movementActions.Gameplay.Camera.performed += OnCamera;

            _movementActions.Gameplay.Sprint.performed += OnSprint;
            _movementActions.Gameplay.Sprint.canceled += SprintEnded;

            _movementActions.Gameplay.Crouch.performed += OnCrouch;
            _movementActions.Gameplay.Crouch.canceled += CrouchEnded;
        }

        private void GetDeviceNew(InputAction.CallbackContext ctx)
        {
            _oldInput = _isMouseAndKeyboard;

            _isMouseAndKeyboard = ctx.control.device is Keyboard or Mouse;

            if (_oldInput != _isMouseAndKeyboard && _isMouseAndKeyboard) changedInputToMouseAndKeyboard.Invoke();
            else if (_oldInput != _isMouseAndKeyboard && !_isMouseAndKeyboard) changedInputToGamepad.Invoke();
        }

        #region Actions
        public void OnMove(InputAction.CallbackContext ctx)
        {
            axisInput = ctx.ReadValue<Vector2>();
            GetDeviceNew(ctx);
        }

        public void OnJump()
        {
            if (enableJump)
            {
                jump = true;
                jumpHold = true;

                _hasJumped = true;
                _skippedFrame = false;
            }
        }

        public void JumpEnded()
        {
            jump = false;
            jumpHold = false;
        }

        private void FixedUpdate()
        {
            if (_hasJumped && _skippedFrame)
            {
                jump = false;
                _hasJumped = false;
            }

            if (!_skippedFrame && enableJump) _skippedFrame = true;
        }
        
        public void OnCamera(InputAction.CallbackContext ctx)
        {
            GetDeviceNew(ctx);
        }

        public void OnSprint(InputAction.CallbackContext ctx)
        {
            if (enableSprint) sprint = true;
        }

        public void SprintEnded(InputAction.CallbackContext ctx)
        {
            sprint = false;
        }
        
        public void OnCrouch(InputAction.CallbackContext ctx)
        {
            if (enableCrouch) crouch = true;
        }
        
        public void CrouchEnded(InputAction.CallbackContext ctx)
        {
            crouch = false;
        }
        #endregion

        private void OnEnable()
        {
            _movementActions.Enable();
        }

        private void OnDisable()
        {
            _movementActions.Disable();
        }
    }
}