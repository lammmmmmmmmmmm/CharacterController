using System.Collections;
using Unity.Cinemachine;
using UnityEngine;
using UnityEngine.InputSystem;

namespace PhysicsCharacterController
{
    public class CameraManager : MonoBehaviour
    {
        [Header("Camera References")]
        [SerializeField] private CinemachineCamera _firstPersonCamera;
        [SerializeField] private CinemachineCamera _thirdPersonCamera;
        [SerializeField] private Camera _mainCamera;
        [SerializeField] private CharacterRotation _characterRotation;

        [Header("First Person Settings")]
        [SerializeField] private LayerMask _firstPersonMask;
        [SerializeField] private float _firstPersonMaskChangeDelaySeconds = 0.1f;
        [SerializeField] private float _firstPersonHeightOnTransition;

        [Header("Third Person Settings")]
        [SerializeField] private LayerMask _thirdPersonMask;
        [SerializeField] private float _thirdPersonMaskChangeDelaySeconds = 0.1f;
        [SerializeField] private float _thirdPersonHeightOnTransition = 0.5f;

        [Header("State")]
        [SerializeField] private bool _isThirdPersonActive = true;
        [SerializeField] private Key _toggleKey = Key.F2;

        private FirstPersonCameraController _firstPersonCameraController;
        private CinemachinePanTilt _firstPersonPanTilt;
        private ThirdPersonCameraController _thirdPersonCameraController;
        private CinemachineOrbitalFollow _thirdPersonOrbitalFollow;
        private Coroutine _maskUpdateCoroutine;

        public bool IsThirdPersonActive => _isThirdPersonActive;

        private void Awake()
        {
            CacheComponents();
            ApplyCameraMode();
        }

        private void Update()
        {
            HandleCameraToggleInput();
        }

        private void CacheComponents()
        {
            _firstPersonCameraController = _firstPersonCamera.GetComponent<FirstPersonCameraController>();
            _firstPersonPanTilt = _firstPersonCamera.GetComponent<CinemachinePanTilt>();
            _thirdPersonCameraController = _thirdPersonCamera.GetComponent<ThirdPersonCameraController>();
            _thirdPersonOrbitalFollow = _thirdPersonCamera.GetComponent<CinemachineOrbitalFollow>();
        }

        private void HandleCameraToggleInput()
        {
            if (Keyboard.current[_toggleKey].wasPressedThisFrame)
            {
                ToggleCameraMode();
            }
        }

        public void ToggleCameraMode()
        {
            _isThirdPersonActive = !_isThirdPersonActive;
            ApplyCameraMode();
        }

        public void SetThirdPersonMode(bool isThirdPerson)
        {
            if (_isThirdPersonActive == isThirdPerson)
            {
                return;
            }

            _isThirdPersonActive = isThirdPerson;
            ApplyCameraMode();
        }

        private void ApplyCameraMode()
        {
            StopMaskUpdateCoroutine();

            if (_isThirdPersonActive)
            {
                ActivateThirdPersonCamera();
            }
            else
            {
                ActivateFirstPersonCamera();
            }
        }

        private void ActivateThirdPersonCamera()
        {
            _characterRotation.SetLockedToCamera(false);

            _firstPersonCamera.gameObject.SetActive(false);
            _thirdPersonCamera.gameObject.SetActive(true);

            float horizontalAxisValue = _firstPersonPanTilt.PanAxis.Value;
            _thirdPersonCameraController.SetInitialValue(horizontalAxisValue, _thirdPersonHeightOnTransition);

            _maskUpdateCoroutine = StartCoroutine(UpdateMaskCoroutine(_thirdPersonMaskChangeDelaySeconds, _thirdPersonMask));
        }

        private void ActivateFirstPersonCamera()
        {
            _characterRotation.SetLockedToCamera(true);

            _firstPersonCamera.gameObject.SetActive(true);
            _thirdPersonCamera.gameObject.SetActive(false);

            float horizontalAxisValue = _thirdPersonOrbitalFollow.HorizontalAxis.Value;
            _firstPersonCameraController.SetInitialValue(horizontalAxisValue, _firstPersonHeightOnTransition);

            _maskUpdateCoroutine = StartCoroutine(UpdateMaskCoroutine(_firstPersonMaskChangeDelaySeconds, _firstPersonMask));
        }

        private IEnumerator UpdateMaskCoroutine(float delaySeconds, LayerMask mask)
        {
            yield return new WaitForSeconds(delaySeconds);
            _mainCamera.cullingMask = mask;
        }

        private void StopMaskUpdateCoroutine()
        {
            if (_maskUpdateCoroutine != null)
            {
                StopCoroutine(_maskUpdateCoroutine);
                _maskUpdateCoroutine = null;
            }
        }
    }
}