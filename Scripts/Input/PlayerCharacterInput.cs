using UnityEngine;
using UnityEngine.InputSystem;

namespace PhysicsCharacterController
{
	/// <summary>
	/// Provides player movement and action input to the character systems. Desktop movement comes
	/// from the configured Input Action, while mobile movement reads the touch joystick directly so
	/// no virtual gamepad control is required.
	/// </summary>
	public class PlayerCharacterInput : BaseCharacterInput
	{
		[Header("Inputs")]
		[SerializeField] private GameObject _characterCamera;
		[SerializeField] private InputActionReference _moveAction;
		[SerializeField] private InputActionReference _jumpAction;
		[SerializeField] private InputActionReference _sprintAction;
		[SerializeField] private InputActionReference _crouchAction;
		[SerializeField] private TouchJoystickInput _touchJoystickInput;

		[SerializeField] private bool _enableJump = true;
		[SerializeField] private bool _enableCrouch = true;
		[SerializeField] private bool _enableSprint = true;

		private float _targetAngle;

		#region Unity Lifecycle

		private void OnEnable()
		{
			SubscribeToMovementInput();

			_jumpAction.action.started += JumpWhenRequested;

			_sprintAction.action.started += StartSprintingWhenRequested;
			_sprintAction.action.canceled += StopSprintingWhenRequested;

			_crouchAction.action.started += StartCrouchingWhenRequested;
			_crouchAction.action.canceled += StopCrouchingWhenRequested;
		}

		private void OnDisable()
		{
			UnsubscribeFromMovementInput();

			_jumpAction.action.started -= JumpWhenRequested;

			_sprintAction.action.started -= StartSprintingWhenRequested;
			_sprintAction.action.canceled -= StopSprintingWhenRequested;

			_crouchAction.action.started -= StartCrouchingWhenRequested;
			_crouchAction.action.canceled -= StopCrouchingWhenRequested;
		}

		#endregion

		#region Public Methods

		public override Vector2 GetMoveInput()
		{
			if (!AreNormalActionsEnabled)
			{
				return Vector2.zero;
			}

			return UnityEngine.Device.Application.isMobilePlatform
				? _touchJoystickInput.InputValue
				: _moveAction.action.ReadValue<Vector2>();
		}

		public override float GetMoveAngle()
		{
			if (!AreNormalActionsEnabled)
			{
				return _targetAngle;
			}

			Vector2 axisInput = GetMoveInput();

			if (axisInput == Vector2.zero)
			{
				return _targetAngle;
			}

			_targetAngle = Mathf.Atan2(axisInput.x, axisInput.y) * Mathf.Rad2Deg + _characterCamera.transform.eulerAngles.y;
			return _targetAngle;
		}

		#endregion

		#region Private Methods

		private void SubscribeToMovementInput()
		{
			if (UnityEngine.Device.Application.isMobilePlatform)
			{
				_touchJoystickInput.OnInputChanged += PublishTouchMovementChange;
				return;
			}

			_moveAction.action.performed += PublishMovementFromAction;
			_moveAction.action.canceled += StopMovementFromAction;
		}

		private void UnsubscribeFromMovementInput()
		{
			if (UnityEngine.Device.Application.isMobilePlatform)
			{
				_touchJoystickInput.OnInputChanged -= PublishTouchMovementChange;
				return;
			}

			_moveAction.action.performed -= PublishMovementFromAction;
			_moveAction.action.canceled -= StopMovementFromAction;
		}

		private void PublishTouchMovementChange(Vector2 movementInput)
		{
			if (!AreNormalActionsEnabled)
			{
				return;
			}

			if (movementInput == Vector2.zero)
			{
				InvokeMoveStop();
				return;
			}

			InvokeMoveStart(movementInput);
		}

		private void PublishMovementFromAction(InputAction.CallbackContext context)
		{
			if (!AreNormalActionsEnabled)
			{
				return;
			}

			InvokeMoveStart(context.ReadValue<Vector2>());
		}

		private void StopMovementFromAction(InputAction.CallbackContext context)
		{
			InvokeMoveStop();
		}

		private void JumpWhenRequested(InputAction.CallbackContext context)
		{
			if (_enableJump && AreNormalActionsEnabled)
			{
				InvokeJumpPressed();
			}
		}

		private void StartSprintingWhenRequested(InputAction.CallbackContext context)
		{
			if (_enableSprint && AreNormalActionsEnabled)
			{
				InvokeSprint(true);
			}
		}

		private void StopSprintingWhenRequested(InputAction.CallbackContext context)
		{
			InvokeSprint(false);
		}

		private void StartCrouchingWhenRequested(InputAction.CallbackContext context)
		{
			if (_enableCrouch && AreNormalActionsEnabled)
			{
				InvokeCrouch(true);
			}
		}

		private void StopCrouchingWhenRequested(InputAction.CallbackContext context)
		{
			InvokeCrouch(false);
		}

		#endregion
	}
}
