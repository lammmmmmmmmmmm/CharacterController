using UnityEngine;

namespace PhysicsCharacterController
{
    public class ClimbArea : MonoBehaviour
    {
        [Header("Area properties")]
        [SerializeField] private Vector3 _climbSpeed = new(0f, 1.37f, 0f);
        [Space(10)]

        [SerializeField] private BaseCharacterInput _inputController;

        private Rigidbody _player;
        private Vector2 _currentInput;

        private void FixedUpdate()
        {
            if (_player != null)
            {
                _currentInput = _inputController.GetMoveInput();
                _player.linearVelocity = new Vector3(_player.linearVelocity.x * _climbSpeed.x, _climbSpeed.y * _currentInput.y, _player.linearVelocity.z * _climbSpeed.z);
            }
        }

        private void OnTriggerEnter(Collider other)
        {
            Rigidbody rigidbody = other.GetComponent<Rigidbody>();

            if (other.GetComponent<CharacterManager>())
            {
                _player = rigidbody;
            }
        }

        private void OnTriggerExit(Collider other)
        {
            Rigidbody rigidbody = other.GetComponent<Rigidbody>();

            if (other.GetComponent<CharacterManager>())
            {
                _player = null;
            }
        }
    }
}