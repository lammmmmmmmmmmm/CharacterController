﻿using Unity.Cinemachine;
using UnityEngine;
using UnityEngine.InputSystem;


namespace PhysicsCharacterController
{
    public class FirstPersonCameraController : MonoBehaviour
    {
        [Header("Camera controls")]
        public Vector2 mouseSensitivity = new(8f, -50f);
        public float smoothSpeed = 0.01f;

        [SerializeField] private InputActionReference cameraActionReference;
        
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