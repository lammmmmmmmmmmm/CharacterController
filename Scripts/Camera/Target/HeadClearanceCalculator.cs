using UnityEngine;

namespace PhysicsCharacterController
{
    /// <summary>
    /// Maintains a one-sided forward clearance envelope around the animated head. Increasing
    /// clearance is immediate so the model cannot catch the camera. Its caller can lock backward
    /// return during locomotion, preventing gait animation from becoming a second camera bob.
    /// </summary>
    public class HeadClearanceCalculator
    {
        private readonly float _headClearanceMeters;
        private readonly float _safetyMarginMeters;
        private readonly float _peakHoldSeconds;
        private readonly float _returnSmoothTimeSeconds;
        private readonly float _maximumCorrectionMeters;

        private float _currentCorrectionMeters;
        private float _returnVelocityMetersPerSecond;
        private float _peakHoldRemainingSeconds;

        #region Public Methods

        public HeadClearanceCalculator(
            float headClearanceMeters,
            float safetyMarginMeters,
            float peakHoldSeconds,
            float returnSmoothTimeSeconds,
            float maximumCorrectionMeters)
        {
            _headClearanceMeters = Mathf.Max(0f, headClearanceMeters);
            _safetyMarginMeters = Mathf.Max(0f, safetyMarginMeters);
            _peakHoldSeconds = Mathf.Max(0f, peakHoldSeconds);
            _returnSmoothTimeSeconds = Mathf.Max(0f, returnSmoothTimeSeconds);
            _maximumCorrectionMeters = Mathf.Max(0f, maximumCorrectionMeters);
        }

        public float UpdateCorrectionMeters(
            float headForwardFromStablePointMeters,
            float deltaTimeSeconds,
            bool canReturnTowardHead,
            out bool hasExceededMaximumCorrection)
        {
            float requiredCorrectionMeters = Mathf.Max(0f, headForwardFromStablePointMeters + _headClearanceMeters + _safetyMarginMeters);
            hasExceededMaximumCorrection = requiredCorrectionMeters > _maximumCorrectionMeters;
            requiredCorrectionMeters = Mathf.Min(requiredCorrectionMeters, _maximumCorrectionMeters);

            if (requiredCorrectionMeters >= _currentCorrectionMeters)
            {
                _currentCorrectionMeters = requiredCorrectionMeters;
                _returnVelocityMetersPerSecond = 0f;
                _peakHoldRemainingSeconds = _peakHoldSeconds;
                return _currentCorrectionMeters;
            }

            if (!canReturnTowardHead || deltaTimeSeconds <= 0f)
            {
                return _currentCorrectionMeters;
            }

            if (_peakHoldRemainingSeconds > 0f)
            {
                _peakHoldRemainingSeconds = Mathf.Max(0f, _peakHoldRemainingSeconds - deltaTimeSeconds);
                return _currentCorrectionMeters;
            }

            _currentCorrectionMeters = _returnSmoothTimeSeconds <= 0f
                ? requiredCorrectionMeters
                : Mathf.SmoothDamp(
                    _currentCorrectionMeters,
                    requiredCorrectionMeters,
                    ref _returnVelocityMetersPerSecond,
                    _returnSmoothTimeSeconds,
                    Mathf.Infinity,
                    deltaTimeSeconds);

            return _currentCorrectionMeters;
        }

        public void Reset()
        {
            _currentCorrectionMeters = 0f;
            _returnVelocityMetersPerSecond = 0f;
            _peakHoldRemainingSeconds = 0f;
        }

        #endregion
    }
}
