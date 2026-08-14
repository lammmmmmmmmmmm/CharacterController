using System;
using UnityEngine;

namespace PhysicsCharacterController
{
	/// <summary>
	/// Base class for all input controllers (human input, AI, etc.)
	/// Provides common interface for character movement input
	/// </summary>
	public abstract class BaseCharacterInput : MonoBehaviour
	{
		public event Action<Vector2> OnMoveStart;
		public event Action OnMoveStop;
		public event Action OnJumpPressed;
		public event Action<bool> OnSprint;
		public event Action<bool> OnCrouch;
		public event Action<bool> OnNormalActionsAvailabilityChanged;

		public bool AreNormalActionsEnabled { get; private set; } = true;

		public abstract Vector2 GetMoveInput();
		public abstract float GetMoveAngle();
		public Vector3 HorizontalMoveDirection => Quaternion.Euler(0f, GetMoveAngle(), 0f) * Vector3.forward;

		#region Public Methods

		public void SetNormalActionsEnabled(bool areEnabled)
		{
			if (AreNormalActionsEnabled == areEnabled)
			{
				return;
			}

			AreNormalActionsEnabled = areEnabled;
			if (!areEnabled)
			{
				InvokeMoveStop();
				InvokeSprint(false);
				InvokeCrouch(false);
			}

			OnNormalActionsAvailabilityChanged?.Invoke(areEnabled);
		}

		#endregion

		#region Protected Methods

		protected void InvokeMoveStart(Vector2 value) => OnMoveStart?.Invoke(value);
		protected void InvokeMoveStop() => OnMoveStop?.Invoke();
		protected void InvokeJumpPressed() => OnJumpPressed?.Invoke();
		protected void InvokeSprint(bool active) => OnSprint?.Invoke(active);
		protected void InvokeCrouch(bool active) => OnCrouch?.Invoke(active);

		#endregion
	}
}
