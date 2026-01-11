using System.Collections.Generic;
using UnityEngine;

namespace PhysicsCharacterController
{
    [RequireComponent(typeof(Collider))]
    public class Ladder : MonoBehaviour
    {
        [Header("Climb properties")]
        [SerializeField] private float _climbSpeed = 7f;
        [SerializeField] private float _forceOnDismount = -200f;

        private class Climber
        {
            public Rigidbody Rigidbody { get; }
            public BaseCharacterInput InputController { get; }
            public CharacterMove CharacterMove { get; }

            public Climber(Rigidbody rigidbody, BaseCharacterInput inputController)
            {
                Rigidbody = rigidbody;
                InputController = inputController;
            }
        }

        private readonly List<Climber> _climbers = new();

        private void FixedUpdate()
        {
            foreach (Climber climber in _climbers)
            {
                Vector2 moveInput = climber.InputController.GetMoveInput();

                if (climber.CharacterMove)
                {
                    if (moveInput.sqrMagnitude < climber.CharacterMove.MovementThreshold * climber.CharacterMove.MovementThreshold)
                    {
                        continue;
                    }

                    float alignment = Vector3.Dot(climber.InputController.HorizontalMoveDirection, transform.forward);

                    if (alignment > 0.5f)
                    {
                        climber.Rigidbody.linearVelocity = new Vector3(0f, _climbSpeed, 0f);
                    }
                    else if (alignment < -0.5f)
                    {
                        climber.Rigidbody.AddForce(_forceOnDismount * transform.forward);
                    }
                }
                else
                {
                    // Fallback to legacy behavior if CharacterMove is missing
                    if (moveInput.y > 0)
                    {
                        climber.Rigidbody.linearVelocity = new Vector3(0f, _climbSpeed, 0f);
                    }
                    else if (moveInput.y < 0)
                    {
                        climber.Rigidbody.AddForce(_forceOnDismount * transform.forward);
                    }
                }
            }
        }

        private void OnTriggerEnter(Collider other)
        {
            Rigidbody rigidbody = other.attachedRigidbody;

            if (rigidbody)
            {
                BaseCharacterInput inputController = rigidbody.GetComponent<BaseCharacterInput>();
                if (inputController && !_climbers.Exists(c => c.Rigidbody == rigidbody))
                {
                    _climbers.Add(new Climber(rigidbody, inputController));
                }
            }
        }

        private void OnTriggerExit(Collider other)
        {
            Rigidbody rigidbody = other.attachedRigidbody;

            if (rigidbody)
            {
                _climbers.RemoveAll(c => c.Rigidbody == rigidbody);
            }
        }
    }
}