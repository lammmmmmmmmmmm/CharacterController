using UnityEngine;

namespace PhysicsCharacterController
{
	/// <summary>
	/// Owns cursor locking for desktop players and exposes explicit operations for camera and menu
	/// systems. Mobile platforms always keep the cursor unlocked because their camera controllers use
	/// touch input independently of cursor state. Editor keyboard input remains in
	/// LockCursorEditorInput so this component has no editor-only input dependency.
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
			bool isCursorLocked = shouldLockCursor && !UnityEngine.Device.Application.isMobilePlatform;
			Cursor.lockState = isCursorLocked ? CursorLockMode.Locked : CursorLockMode.None;
			Cursor.visible = !isCursorLocked;
		}

		#endregion
	}
}
