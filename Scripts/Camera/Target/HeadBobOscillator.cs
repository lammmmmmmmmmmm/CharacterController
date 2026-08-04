using UnityEngine;

namespace PhysicsCharacterController
{
    /// <summary>
    /// Produces a bounded procedural walking offset from actual planar speed. The oscillator fades
    /// in only during grounded locomotion and always settles back to the clean camera target so it
    /// cannot leave a persistent offset after stopping, jumping, or losing movement authority.
    /// </summary>
    public class HeadBobOscillator
    {
        private const float RADIANS_PER_CYCLE = Mathf.PI * 2f;

        private readonly float _minimumMovementSpeedMetersPerSecond;
        private readonly float _maximumLateralOffsetMeters;
        private readonly float _maximumVerticalOffsetMeters;
        private readonly float _minimumFrequencyHertz;
        private readonly float _maximumFrequencyHertz;
        private readonly float _activationSeconds;
        private readonly float _returnSeconds;

        private float _phaseRadians;
        private float _weight01;

        #region Public Methods

        public HeadBobOscillator(
            float minimumMovementSpeedMetersPerSecond,
            float maximumLateralOffsetMeters,
            float maximumVerticalOffsetMeters,
            float minimumFrequencyHertz,
            float maximumFrequencyHertz,
            float activationSeconds,
            float returnSeconds)
        {
            _minimumMovementSpeedMetersPerSecond = Mathf.Max(0f, minimumMovementSpeedMetersPerSecond);
            _maximumLateralOffsetMeters = Mathf.Max(0f, maximumLateralOffsetMeters);
            _maximumVerticalOffsetMeters = Mathf.Max(0f, maximumVerticalOffsetMeters);
            _minimumFrequencyHertz = Mathf.Max(0f, minimumFrequencyHertz);
            _maximumFrequencyHertz = Mathf.Max(_minimumFrequencyHertz, maximumFrequencyHertz);
            _activationSeconds = Mathf.Max(0f, activationSeconds);
            _returnSeconds = Mathf.Max(0f, returnSeconds);
        }

        public Vector3 UpdateOffsetMeters(
            float planarSpeedMetersPerSecond,
            float maximumCharacterSpeedMetersPerSecond,
            bool isGrounded,
            bool hasMovementInput,
            float deltaTimeSeconds)
        {
            if (deltaTimeSeconds <= 0f)
            {
                return EvaluateCurrentOffset();
            }

            bool canBob = isGrounded &&
                          hasMovementInput &&
                          planarSpeedMetersPerSecond >= _minimumMovementSpeedMetersPerSecond &&
                          maximumCharacterSpeedMetersPerSecond > 0f;
            float speed01 = canBob
                ? Mathf.Clamp01(planarSpeedMetersPerSecond / maximumCharacterSpeedMetersPerSecond)
                : 0f;
            float targetWeight01 = canBob ? speed01 : 0f;
            float transitionSeconds = canBob ? _activationSeconds : _returnSeconds;

            _weight01 = transitionSeconds <= 0f
                ? targetWeight01
                : Mathf.MoveTowards(_weight01, targetWeight01, deltaTimeSeconds / transitionSeconds);

            if (canBob)
            {
                float frequencyHertz = Mathf.Lerp(_minimumFrequencyHertz, _maximumFrequencyHertz, speed01);
                _phaseRadians = Mathf.Repeat(_phaseRadians + frequencyHertz * RADIANS_PER_CYCLE * deltaTimeSeconds, RADIANS_PER_CYCLE);
            }
            else if (_weight01 <= 0f)
            {
                _phaseRadians = 0f;
            }

            return EvaluateCurrentOffset();
        }

        public void Reset()
        {
            _phaseRadians = 0f;
            _weight01 = 0f;
        }

        #endregion

        #region Private Methods

        private Vector3 EvaluateCurrentOffset()
        {
            float lateralOffsetMeters = Mathf.Sin(_phaseRadians) * _maximumLateralOffsetMeters * _weight01;
            float verticalOffsetMeters = Mathf.Sin(_phaseRadians * 2f) * _maximumVerticalOffsetMeters * _weight01;

            return new Vector3(lateralOffsetMeters, verticalOffsetMeters, 0f);
        }

        #endregion
    }
}
