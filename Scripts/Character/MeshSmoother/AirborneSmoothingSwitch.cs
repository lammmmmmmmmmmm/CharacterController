using System;
using UnityEngine;

namespace PhysicsCharacterController
{
    /// <summary>
    /// Selects a smoothing time based on grounded state, with hysteresis: airborne smoothing only
    /// engages after the character has been continuously off the ground for an activation delay.
    /// Prevents ground-contact flicker (e.g. grinding against a wall or step) from rapidly toggling
    /// the smoothing mode, which reads as camera or mesh shaking.
    /// </summary>
    [Serializable]
    public class AirborneSmoothingSwitch
    {
        [Tooltip("Smoothing time while grounded, absorbs terrain bumps")]
        [SerializeField] private float _groundedSmoothTimeSeconds;
        [Tooltip("Smoothing time once airborne, keep small for 1:1 tracking of jumps and falls")]
        [SerializeField] private float _airborneSmoothTimeSeconds;
        [Tooltip("Continuous off-ground time required before airborne smoothing engages. Filters out ground-contact flicker")]
        [SerializeField] private float _airborneActivationDelaySeconds = 0.1f;

        private float _offGroundSeconds;

        public AirborneSmoothingSwitch(float groundedSmoothTimeSeconds, float airborneSmoothTimeSeconds)
        {
            _groundedSmoothTimeSeconds = groundedSmoothTimeSeconds;
            _airborneSmoothTimeSeconds = airborneSmoothTimeSeconds;
        }

        public float SelectSmoothTimeSeconds(bool isGrounded, float deltaTimeSeconds)
        {
            if (isGrounded)
            {
                _offGroundSeconds = 0f;
                return _groundedSmoothTimeSeconds;
            }

            _offGroundSeconds += deltaTimeSeconds;
            bool hasClearedActivationDelay = _offGroundSeconds >= _airborneActivationDelaySeconds;

            return hasClearedActivationDelay ? _airborneSmoothTimeSeconds : _groundedSmoothTimeSeconds;
        }
    }
}
