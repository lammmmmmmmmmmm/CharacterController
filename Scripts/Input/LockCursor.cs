using UnityEngine;

namespace PhysicsCharacterController
{
	/// <summary>
	/// Owns the application's cursor lock state and exposes explicit operations for other systems.
	/// Editor keyboard input is intentionally handled by LockCursorEditorInput so menus and other
	/// runtime systems can reuse this component without depending on editor-only input wiring.
	/// </summary>
	public class LockCursor : MonoBehaviour
	{
		[SerializeField] private bool _lockCursor;

		public bool IsCursorLocked => Cursor.lockState == CursorLockMode.Locked;

		#region Unity Lifecycle

		private void Awake()
		{
			SetCursorLock(_lockCursor);
		}

		#endregion

		#region Public Methods

		public void ToggleCursorLock()
		{
			SetCursorLock(!IsCursorLocked);
		}

		public void SetCursorLock(bool shouldLockCursor)
		{
			Cursor.lockState = shouldLockCursor ? CursorLockMode.Locked : CursorLockMode.None;
			Cursor.visible = !shouldLockCursor;
		}

		#endregion
	}
}
