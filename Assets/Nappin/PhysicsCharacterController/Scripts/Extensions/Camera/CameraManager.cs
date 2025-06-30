using UnityEngine;
using UnityEngine.InputSystem;
using System.Collections;
using Unity.Cinemachine;

namespace PhysicsCharacterController
{
    public class CameraManager : MonoBehaviour
    {
        [Header("Camera properties")]
        public CinemachineCamera firstPersonCamera;
        public CinemachineCamera thirdPersonCamera;
        public Camera mainCamera;
        public CharacterManager characterManager;
        [Space(10)]
        public LayerMask firstPersonMask;
        public float firstPersonMaskChangeDelay = 0.1f;
        public float firstPersonHeightOnTransition = 0f;
        [Space(10)]
        public LayerMask thirdPersonMask;
        public float thirdPersonMaskChangeDelay = 0.1f;
        public float thirdPersonHeightOnTransition = 0.5f;
        [Space(10)]
        public bool activeThirdPerson = true;

        private FirstPersonCameraController _firstPersonCameraController;
        private CinemachinePanTilt _firstPersonCameraControllerPov;

        private ThirdPersonCameraController _thirdPersonCameraController;

        private void Awake()
        {
            _firstPersonCameraController = firstPersonCamera.GetComponent<FirstPersonCameraController>();
            _firstPersonCameraControllerPov = firstPersonCamera.GetComponent<CinemachinePanTilt>();

            _thirdPersonCameraController = thirdPersonCamera.GetComponent<ThirdPersonCameraController>();

            SetCamera();
        }

        private void Update()
        {
            if (Keyboard.current.mKey.wasPressedThisFrame)
            {
                activeThirdPerson = !activeThirdPerson;
                SetCamera();
            }

            if (Keyboard.current.nKey.wasPressedThisFrame)
            {
                SetDebug();
            }
        }

        public void SetCamera()
        {
            if (activeThirdPerson)
            {
                characterManager.SetLockToCamera(false);

                firstPersonCamera.gameObject.SetActive(false);
                thirdPersonCamera.gameObject.SetActive(true);

                _thirdPersonCameraController.SetInitialValue(_firstPersonCameraControllerPov.PanAxis.Value,
                    thirdPersonHeightOnTransition);

                StartCoroutine(UpdateMask(thirdPersonMaskChangeDelay, thirdPersonMask));
            }
            else
            {
                characterManager.SetLockToCamera(true);

                firstPersonCamera.gameObject.SetActive(true);
                thirdPersonCamera.gameObject.SetActive(false);

                _firstPersonCameraController.SetInitialValue(thirdPersonCamera.GetComponent<CinemachineOrbitalFollow>().HorizontalAxis.Value,
                    firstPersonHeightOnTransition);

                StartCoroutine(UpdateMask(firstPersonMaskChangeDelay, firstPersonMask));
            }
        }

        public void SetDebug()
        {
            characterManager.debug = !characterManager.debug;
        }

        private IEnumerator UpdateMask(float duration, LayerMask mask)
        {
            yield return new WaitForSeconds(duration);
            mainCamera.cullingMask = mask;
        }
    }
}