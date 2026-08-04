using System;
using UnityEngine;

namespace PhysicsCharacterController
{
    /// <summary>
    /// Measures how abruptly the current ground normal changes and retains the resulting roughness
    /// long enough for visual followers to remain stable across a complete uneven-terrain patch.
    /// Consumers share this calculation instead of independently interpreting ground contacts.
    /// </summary>
    [Serializable]
    public class GroundSurfaceRoughnessTracker
    {
        private const float MINIMUM_NORMAL_RATE_DEGREES_PER_SECOND = 0.01f;
        private const float MINIMUM_RELEASE_SECONDS = 0.01f;

        [Tooltip("Ground-normal rotation rate that represents fully rough terrain")]
        [SerializeField, Min(MINIMUM_NORMAL_RATE_DEGREES_PER_SECOND)] private float _fullRoughnessNormalRateDegreesPerSecond = 60f;
        [Tooltip("Time for detected roughness to fade after the last uneven contact")]
        [SerializeField, Min(MINIMUM_RELEASE_SECONDS)] private float _roughnessReleaseSeconds = 0.4f;

        private Vector3 _previousGroundNormal;

        public float Roughness01 { get; private set; }

        #region Public Methods

        public GroundSurfaceRoughnessTracker()
        {
        }

        public GroundSurfaceRoughnessTracker(float fullRoughnessNormalRateDegreesPerSecond, float roughnessReleaseSeconds)
        {
            _fullRoughnessNormalRateDegreesPerSecond = Mathf.Max(MINIMUM_NORMAL_RATE_DEGREES_PER_SECOND, fullRoughnessNormalRateDegreesPerSecond);
            _roughnessReleaseSeconds = Mathf.Max(MINIMUM_RELEASE_SECONDS, roughnessReleaseSeconds);
        }

        public float Update(bool isGrounded, Vector3 groundNormal, float fixedDeltaTimeSeconds, float deltaTimeSeconds)
        {
            if (!isGrounded)
            {
                _previousGroundNormal = Vector3.zero;
                ReleaseRoughness(deltaTimeSeconds);
                return Roughness01;
            }

            RecordGroundNormalImpulse(groundNormal, fixedDeltaTimeSeconds);
            ReleaseRoughness(deltaTimeSeconds);
            return Roughness01;
        }

        public void Reset()
        {
            _previousGroundNormal = Vector3.zero;
            Roughness01 = 0f;
        }

        #endregion

        #region Private Methods

        private void RecordGroundNormalImpulse(Vector3 groundNormal, float fixedDeltaTimeSeconds)
        {
            if (groundNormal.sqrMagnitude <= Mathf.Epsilon)
            {
                _previousGroundNormal = Vector3.zero;
                return;
            }

            Vector3 normalizedGroundNormal = groundNormal.normalized;
            bool hasPreviousSample = _previousGroundNormal.sqrMagnitude > Mathf.Epsilon;
            if (hasPreviousSample && fixedDeltaTimeSeconds > 0f)
            {
                float normalRateDegreesPerSecond = Vector3.Angle(_previousGroundNormal, normalizedGroundNormal) / fixedDeltaTimeSeconds;
                float roughnessImpulse01 = Mathf.Clamp01(normalRateDegreesPerSecond / _fullRoughnessNormalRateDegreesPerSecond);

                Roughness01 = Mathf.Max(Roughness01, roughnessImpulse01);
            }

            _previousGroundNormal = normalizedGroundNormal;
        }

        private void ReleaseRoughness(float deltaTimeSeconds)
        {
            if (deltaTimeSeconds <= 0f)
            {
                return;
            }

            Roughness01 = Mathf.MoveTowards(Roughness01, 0f, deltaTimeSeconds / _roughnessReleaseSeconds);
        }

        #endregion
    }
}
