using UnityEngine;
using UnityEngine.InputSystem;

namespace PhysicsCharacterController
{
	public class PlayerCharacterInput : BaseCharacterInput
	{
		[Header("Inputs")]
		[SerializeField] private GameObject _characterCamera;
		[SerializeField] private InputActionReference _moveAction;
		[SerializeField] private InputActionReference _jumpAction;
		[SerializeField] private InputActionReference _sprintAction;
		[SerializeField] private InputActionReference _crouchAction;

		[SerializeField] private bool _enableJump = true;
		[SerializeField] private bool _enableCrouch = true;
		[SerializeField] private bool _enableSprint = true;

		private float _targetAngle;

		private void OnEnable()
		{
			_moveAction.action.Enable();
			_moveAction.action.performed += OnMoveActionPerformed;
			_moveAction.action.canceled += OnMoveActionCanceled;

			_jumpAction.action.Enable();
			_jumpAction.action.started += OnJumpAction;

			_sprintAction.action.Enable();
			_sprintAction.action.started += OnSprintActionStarted;
			_sprintAction.action.canceled += OnSprintActionEnded;

			_crouchAction.action.Enable();
			_crouchAction.action.started += OnCrouchAction;
			_crouchAction.action.canceled += OnCrouchActionEnded;
		}

		private void OnDisable()
		{
			_moveAction.action.performed -= OnMoveActionPerformed;
			_moveAction.action.canceled -= OnMoveActionCanceled;
			_moveAction.action.Disable();

			_jumpAction.action.started -= OnJumpAction;
			_jumpAction.action.Disable();

			_sprintAction.action.started -= OnSprintActionStarted;
			_sprintAction.action.canceled -= OnSprintActionEnded;
			_sprintAction.action.Disable();

			_crouchAction.action.started -= OnCrouchAction;
			_crouchAction.action.canceled -= OnCrouchActionEnded;
			_crouchAction.action.Disable();
		}

		public override Vector2 GetMoveInput()
		{
			return _moveAction.action.ReadValue<Vector2>();
		}

		public override float GetMoveAngle()
		{
			Vector2 axisInput = GetMoveInput();

			// IMPORTANT: If there is no input, return the last target angle
			if (axisInput == Vector2.zero) return _targetAngle;

			_targetAngle = Mathf.Atan2(axisInput.x, axisInput.y) * Mathf.Rad2Deg + _characterCamera.transform.eulerAngles.y;
			return _targetAngle;
		}

		#region Actions
		private void OnMoveActionPerformed(InputAction.CallbackContext ctx)
		{
			InvokeMoveStart(ctx.ReadValue<Vector2>());
		}

		private void OnMoveActionCanceled(InputAction.CallbackContext ctx)
		{
			InvokeMoveStop();
		}

		private void OnJumpAction(InputAction.CallbackContext ctx)
		{
			if (_enableJump) InvokeJumpPressed();
		}

		private void OnSprintActionStarted(InputAction.CallbackContext ctx)
		{
			if (_enableSprint) InvokeSprint(true);
		}

		private void OnSprintActionEnded(InputAction.CallbackContext ctx)
		{
			InvokeSprint(false);
		}

		private void OnCrouchAction(InputAction.CallbackContext ctx)
		{
			if (_enableCrouch) InvokeCrouch(true);
		}

		private void OnCrouchActionEnded(InputAction.CallbackContext ctx)
		{
			InvokeCrouch(false);
		}
		#endregion
	}
}