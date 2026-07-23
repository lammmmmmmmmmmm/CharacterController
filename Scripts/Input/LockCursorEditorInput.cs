using UnityEngine;
using UnityEngine.InputSystem;

namespace PhysicsCharacterController
{
	/// <summary>
	/// Routes an editor keyboard key to a LockCursor component.
	/// This component contains input wiring only; cursor state remains owned by LockCursor.
	/// It is inert in player builds because all keyboard-specific behavior is editor-only.
	/// </summary>
	public class LockCursorEditorInput : MonoBehaviour
	{
#if UNITY_EDITOR
		[SerializeField] private LockCursor _lockCursor;
		[SerializeField] private Key _toggleKey = Key.F1;

		private bool _hasLoggedMissingKeyboard;

		#region Unity Lifecycle

		private void Update()
		{
			if (Keyboard.current == null)
			{
				LogMissingKeyboardOnce();
				return;
			}

			if (Keyboard.current[_toggleKey].wasPressedThisFrame)
			{
				_lockCursor.ToggleCursorLock();
			}
		}

		#endregion

		#region Private Methods

		private void LogMissingKeyboardOnce()
		{
			if (_hasLoggedMissingKeyboard)
			{
				return;
			}

			_hasLoggedMissingKeyboard = true;
			Debug.LogWarning("[LockCursorEditorInput] No keyboard device is available; editor cursor toggle is disabled.", this);
		}

		#endregion
#endif
	}
}
