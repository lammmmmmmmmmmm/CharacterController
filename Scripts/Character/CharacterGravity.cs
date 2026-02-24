using UnityEngine;

namespace PhysicsCharacterController
{
    [RequireComponent(typeof(Rigidbody))]
    public class CharacterGravity : MonoBehaviour
    {
        [Header("Gravity Settings")]
        [SerializeField] private float _gravityMultiplier = 6f;
        [Tooltip("Multiplier factor for gravity used on change of normal")]
        [SerializeField] private float _gravityMultiplierOnSlideChange = 3f;
        [Tooltip("Multiplier factor for gravity used on non climbable slope")]
        [SerializeField] private float _gravityMultiplierIfUnclimbableSlope = 30f;
        [SerializeField] private float _maxDownwardSpeed = 200f;

        [Header("Dependencies")]
        [SerializeField] private SlopeChecker _slopeChecker;
        [SerializeField] private StepChecker _stepChecker;
        [SerializeField] private Rigidbody _rigidbody;

        public float VerticalVelocity => _rigidbody.linearVelocity.y;

        public void ApplyGravity()
        {
            Vector3 gravity = CalculateBaseGravity();

            if (HasGroundNormalChanged())
            {
                gravity *= _gravityMultiplierOnSlideChange;
            }

            if (IsOnUnclimbableSlope())
            {
                gravity = CalculateUnclimbableSlopeGravity();
            }

            if (_rigidbody.linearVelocity.y > -_maxDownwardSpeed)
            {
                _rigidbody.AddForce(gravity);
            }
        }

        private Vector3 CalculateBaseGravity()
        {
            bool shouldUseNormalGravity = _slopeChecker.IsSlidePreventionActive
                || _stepChecker.IsTouchingStep;

            if (shouldUseNormalGravity)
            {
                return _slopeChecker.GravityProjectedOnNormal * (_gravityMultiplier * -Physics.gravity.y);
            }

            return Vector3.down * (_gravityMultiplier * -Physics.gravity.y);
        }

        private bool HasGroundNormalChanged()
        {
            return !Mathf.Approximately(_slopeChecker.GroundNormal.y, 1)
                   && _slopeChecker.GroundNormal.y != 0
                   && _slopeChecker.IsTouchingSlope
                   && _slopeChecker.PreviousGroundNormal != _slopeChecker.GroundNormal;
        }

        private bool IsOnUnclimbableSlope()
        {
            return !Mathf.Approximately(_slopeChecker.GroundNormal.y, 1)
                   && _slopeChecker.GroundNormal.y != 0
                   && _slopeChecker.CurrentSurfaceAngle > _slopeChecker.MaxClimbableSlopeAngle
                   && !_stepChecker.IsTouchingStep;
        }

        private Vector3 CalculateUnclimbableSlopeGravity()
        {
            float surfaceAngle = _slopeChecker.CurrentSurfaceAngle;

            if (surfaceAngle is > 0f and <= 30f)
            {
                return Vector3.down * (_gravityMultiplierIfUnclimbableSlope * -Physics.gravity.y);
            }

            if (surfaceAngle is > 30f and <= 89f)
            {
                return Vector3.down * (_gravityMultiplierIfUnclimbableSlope * 0.5f * -Physics.gravity.y);
            }

            return Vector3.down * (_gravityMultiplier * -Physics.gravity.y);
        }
    }
}
