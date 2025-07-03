using Unity.Cinemachine;
using UnityEngine;
using UnityEngine.InputSystem;

namespace PhysicsCharacterController
{
    public class ThirdPersonCameraController : MonoBehaviour
    {
        [Header("Camera controls")]
        public Vector2 mouseSensitivity = new(5f, 1f);
        public float smoothSpeed = 0.05f;
        
        [SerializeField] private InputActionReference cameraActionReference;

        private CinemachineOrbitalFollow _orbitalFollow;

        private Vector2 _smoothVelocity;
        private Vector2 _currentInputVector;
        private Vector2 _input;

        private float _switchValueX;
        private float _switchValueY;

        private void Awake()
        {
            _orbitalFollow = GetComponent<CinemachineOrbitalFollow>();
        }

        private void Update()
        {
            _input += cameraActionReference.action.ReadValue<Vector2>() * mouseSensitivity * new Vector2(0.01f, 0.001f);

            if (_input.y > 1f) _input.y = 1f;
            else if (_input.y < 0f) _input.y = 0f;

            _currentInputVector = Vector2.SmoothDamp(_currentInputVector, _input, ref _smoothVelocity, smoothSpeed);
            _orbitalFollow.HorizontalAxis.Value = _currentInputVector.x;
            _orbitalFollow.VerticalAxis.Value = 1 - _currentInputVector.y;
        }

        public void SetInitialValue(float valueX, float valueY)
        {
            _input = new Vector2(valueX, valueY);
            _currentInputVector = _input;

            _orbitalFollow.HorizontalAxis.Value = valueX;
            _orbitalFollow.VerticalAxis.Value = valueY;
        }
    }
}