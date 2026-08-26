using UnityEngine;

namespace PhysicsCharacterController
{
    /// <summary>
    /// Runs one character-facing policy and delegates the actual Rigidbody rotation to CharacterRotation.
    /// </summary>
    [DisallowMultipleComponent]
    public abstract class CharacterRotationPolicy : MonoBehaviour
    {
        [Header("Dependencies")]
        [SerializeField] private CharacterRotation _characterRotation;

        private bool _isAutomaticRotationEnabled = true;

        public abstract Vector3 MovementForwardDirection { get; }
        public abstract Vector3 MovementRightDirection { get; }

        protected CharacterRotation CharacterRotation => _characterRotation;

        #region Unity Lifecycle

        protected void FixedUpdate()
        {
            if (!_isAutomaticRotationEnabled || !TryResolveFacingDirection(out Vector3 worldDirection))
            {
                return;
            }

            _characterRotation.RotateTowardsDirection(worldDirection, Time.fixedDeltaTime);
        }

        #endregion

        #region Public Methods

        public void SetAutomaticRotationEnabled(bool isEnabled)
        {
            _isAutomaticRotationEnabled = isEnabled;
        }

        #endregion

        #region Protected Methods

        protected abstract bool TryResolveFacingDirection(out Vector3 worldDirection);

        #endregion
    }
}
