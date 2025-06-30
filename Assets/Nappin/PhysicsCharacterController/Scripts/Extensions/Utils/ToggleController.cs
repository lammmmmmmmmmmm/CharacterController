using UnityEngine;


namespace PhysicsCharacterController
{
    public class ToggleController : MonoBehaviour
    {
        [Header("Camera specs")]
        public GameObject gamepadCamera;
        public GameObject mouseAndKeyboardCamera;


        public void IsInputGamepad(bool status)
        {
            gamepadCamera.SetActive(status);
            mouseAndKeyboardCamera.SetActive(!status);
        }
    }
}