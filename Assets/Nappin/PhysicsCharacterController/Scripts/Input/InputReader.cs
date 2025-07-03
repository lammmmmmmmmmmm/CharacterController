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
        [SerializeField] private InputActionReference moveAction;
        [SerializeField] private InputActionReference jumpAction;
        [SerializeField] private InputActionReference cameraAction;
        [SerializeField] private InputActionReference sprintAction;
        [SerializeField] private InputActionReference crouchAction;
        public bool enableJump = true;
        public bool enableCrouch = true;
        public bool enableSprint = true;

        [HideInInspector]
        public Vector2 axisInput;
        [HideInInspector]
        public bool jump;
        [HideInInspector]
        public bool sprint;
        [HideInInspector]
        public bool crouch;

        private bool _hasJumped;
        private bool _skippedFrame;
        private bool _isMouseAndKeyboard = true;
        private bool _oldInput = true;

        private void OnEnable()
        {
            moveAction.action.Enable();
            moveAction.action.performed += OnMove;
            
            jumpAction.action.performed += OnJump;
            jumpAction.action.canceled += JumpEnded;
            
            cameraAction.action.performed += OnCamera;
            
            sprintAction.action.performed += OnSprint;
            sprintAction.action.canceled += SprintEnded;
            
            crouchAction.action.performed += OnCrouch;
            crouchAction.action.canceled += CrouchEnded;
        }

        private void OnDisable()
        {
            moveAction.action.performed -= OnMove;
            
            jumpAction.action.performed -= OnJump;
            jumpAction.action.canceled -= JumpEnded;
            
            cameraAction.action.performed -= OnCamera;
            
            sprintAction.action.performed -= OnSprint;
            sprintAction.action.canceled -= SprintEnded;
            
            crouchAction.action.performed -= OnCrouch;
            crouchAction.action.canceled -= CrouchEnded;
        }

        private void GetDeviceNew(InputAction.CallbackContext ctx)
        {
            _oldInput = _isMouseAndKeyboard;

            _isMouseAndKeyboard = ctx.control.device is Keyboard or Mouse;

            if (_oldInput != _isMouseAndKeyboard && _isMouseAndKeyboard) changedInputToMouseAndKeyboard.Invoke();
            else if (_oldInput != _isMouseAndKeyboard && !_isMouseAndKeyboard) changedInputToGamepad.Invoke();
        }

        #region Actions
        private void FixedUpdate()
        {
            if (_hasJumped && _skippedFrame)
            {
                jump = false;
                _hasJumped = false;
            }

            if (!_skippedFrame && enableJump) _skippedFrame = true;
        }

        public void OnMove(InputAction.CallbackContext ctx)
        {
            axisInput = ctx.ReadValue<Vector2>();
            GetDeviceNew(ctx);
        }

        public void OnJump(InputAction.CallbackContext ctx)
        {
            if (enableJump)
            {
                jump = true;

                _hasJumped = true;
                _skippedFrame = false;
            }
        }

        public void JumpEnded(InputAction.CallbackContext ctx)
        {
            jump = false;
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
    }
}