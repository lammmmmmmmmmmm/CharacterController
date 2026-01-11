using UnityEngine;

namespace PhysicsCharacterController
{
	public class LockCursor : MonoBehaviour
	{
		[SerializeField] private bool _lockCursor;

		private void Awake()
		{
			if (_lockCursor) Cursor.lockState = CursorLockMode.Locked;
		}
	}
}