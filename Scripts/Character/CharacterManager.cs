using UnityEngine;
using UnityEngine.Events;

namespace PhysicsCharacterController
{
	[RequireComponent(typeof(CapsuleCollider))]
	[RequireComponent(typeof(Rigidbody))]
	[RequireComponent(typeof(CharacterMove))]
	[RequireComponent(typeof(CharacterCrouch))]
	[RequireComponent(typeof(CharacterGravity))]
	[RequireComponent(typeof(CharacterRotation))]
	[RequireComponent(typeof(SlopeChecker))]
	[RequireComponent(typeof(CharacterStateMachine.CharacterStateMachineDriver))]
	//TODO: Remove this script
	public class CharacterManager : MonoBehaviour
	{
		[Header("Character components")]
		[SerializeField] private CharacterMove _characterMove;
		[SerializeField] private CharacterCrouch _characterCrouch;

		[Header("Checkers")]
		[SerializeField] private GroundChecker _groundChecker;
		[SerializeField] private StepChecker _stepChecker;

		[Header("Events")]
		[SerializeField] private float _minimumVerticalSpeedToLightLandEvent;
		[SerializeField] private UnityEvent _onLightLand;

		private Rigidbody _rigidbody;
		private CapsuleCollider _collider;
		private float _originalColliderHeight;

		private float _previousYVelocity;

		private void Awake()
		{
			_rigidbody = GetComponent<Rigidbody>();
			_collider = GetComponent<CapsuleCollider>();
			_originalColliderHeight = _collider.height;

			SetNoFriction();

			_stepChecker.SetFeetOffset(_originalColliderHeight / 2f);
		}

		private void Update()
		{
			UpdateEvents();
		}

		private void UpdateEvents()
		{
			if (_groundChecker.JustLanded)
			{
				if (_previousYVelocity < -_minimumVerticalSpeedToLightLandEvent)
				{
					_onLightLand.Invoke();
				}
			}
			_previousYVelocity = _rigidbody.linearVelocity.y;
		}

		private void SetNoFriction()
		{
			_collider.material.dynamicFriction = 0f;
			_collider.material.staticFriction = 0f;

			_collider.material.frictionCombine = PhysicsMaterialCombine.Minimum;
		}

		public bool IsJumping()
		{
			return _rigidbody.linearVelocity.y > 1f;
		}

		public bool IsFalling()
		{
			return _rigidbody.linearVelocity.y < 0f && !_groundChecker.IsGrounded;
		}

		public bool IsRunning()
		{
			return _characterMove.HasMovementInput && !_characterCrouch.IsCrouching && _groundChecker.IsGrounded;
		}

		public float GetOriginalColliderHeight()
		{
			return _originalColliderHeight;
		}
	}
}