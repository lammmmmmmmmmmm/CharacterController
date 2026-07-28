using Unity.Cinemachine;
using UnityEngine;
using UnityEngine.InputSystem;


namespace PhysicsCharacterController
{
	/// <summary>
    /// Converts camera input into smoothed first-person pan and tilt values. Desktop input is accepted
    /// only while the shared cursor is locked; mobile and Device Simulator input ignores cursor state
    /// because touch camera control does not use a desktop cursor.
	/// </summary>
    public class FirstPersonCameraController : MonoBehaviour
    {
        [Header("Camera controls")]
        public Vector2 mouseSensitivity = new(8f, -50f);
        public float smoothSpeed = 0.01f;

        [SerializeField] private InputActionReference cameraActionReference;
        [SerializeField] private LockCursor _lockCursor;
        
        private CinemachinePOV _cinemachinePov;

        private Vector2 _smoothVelocity;
        private Vector2 _currentInputVector;
        private Vector2 _input;

        private float _switchValueX;
        private float _switchValueY;

        private void Awake()
        {
            _cinemachinePov = GetComponent<CinemachineVirtualCamera>().GetCinemachineComponent<CinemachinePOV>();
        }

        private void Update()
        {
            bool canReadCameraInput = UnityEngine.Device.Application.isMobilePlatform || _lockCursor.IsCursorLocked;
            if (!canReadCameraInput)
            {
                return;
            }

            _input += cameraActionReference.action.ReadValue<Vector2>() * mouseSensitivity * new Vector2(0.01f, 0.001f);

            if (_input.y > _cinemachinePov.m_VerticalAxis.m_MaxValue) _input.y = _cinemachinePov.m_VerticalAxis.m_MaxValue;
            else if (_input.y < _cinemachinePov.m_VerticalAxis.m_MinValue) _input.y = _cinemachinePov.m_VerticalAxis.m_MinValue;

            _currentInputVector = Vector2.SmoothDamp(_currentInputVector, _input, ref _smoothVelocity, smoothSpeed);
            _cinemachinePov.m_HorizontalAxis.Value = _currentInputVector.x;
            _cinemachinePov.m_VerticalAxis.Value = _currentInputVector.y;
        }

        public void SetInitialValue(float valueX, float valueY)
        {
            _input = new Vector2(valueX, valueY);
            _currentInputVector = _input;

            _cinemachinePov.m_HorizontalAxis.Value = valueX;
            _cinemachinePov.m_VerticalAxis.Value = valueY;
        }
    }
}
