using UnityEngine;

namespace PhysicsCharacterController
{
    [CreateAssetMenu(fileName = "Swimming Movement Settings", menuName = "Character Movement/Swimming Movement Settings")]
    public sealed class SwimmingMovementSettingsSO : ScriptableObject
    {
        [Header("State Thresholds")]
        [Tooltip("Standing-collider immersion required to enter swimming.")]
        [Range(0f, 1f)]
        [SerializeField] private float _enterSwimmingImmersion01 = 0.65f;
        [Tooltip("Standing-collider immersion below which swimming exits.")]
        [Range(0f, 1f)]
        [SerializeField] private float _exitSwimmingImmersion01 = 0.45f;
        [Tooltip("Camera-relative vertical direction required to deliberately dive.")]
        [Range(-1f, 0f)]
        [SerializeField] private float _diveDirectionYThreshold = -0.2f;

        [Header("Surface")]
        [SerializeField] private float _surfaceRootDepthMeters = 0.5f;
        [SerializeField] private float _surfaceToleranceMeters = 0.08f;
        [SerializeField] private float _maximumSurfaceStabilizationSpeedMetersPerSecond = 2.5f;

        [Header("Movement")]
        [SerializeField] private float _normalSpeedMetersPerSecond = 3f;
        [SerializeField] private float _fastSpeedMetersPerSecond = 6f;
        [SerializeField] private float _accelerationMetersPerSecondSquared = 10f;
        [SerializeField] private float _decelerationMetersPerSecondSquared = 8f;
        [SerializeField] private float _movementInputThreshold = 0.01f;
        [SerializeField] private float _collisionSweepSkinMeters = 0.05f;

        [Header("Orientation")]
        [SerializeField] private float _yawSpeedDegreesPerSecond = 360f;
        [SerializeField] private float _pitchSpeedDegreesPerSecond = 180f;
        [SerializeField] private float _underwaterColliderRotationSpeedDegreesPerSecond = 360f;

        public float EnterSwimmingImmersion01 => _enterSwimmingImmersion01;
        public float ExitSwimmingImmersion01 => _exitSwimmingImmersion01;
        public float DiveDirectionYThreshold => _diveDirectionYThreshold;
        public float SurfaceRootDepthMeters => _surfaceRootDepthMeters;
        public float SurfaceToleranceMeters => _surfaceToleranceMeters;
        public float MaximumSurfaceStabilizationSpeedMetersPerSecond => _maximumSurfaceStabilizationSpeedMetersPerSecond;
        public float NormalSpeedMetersPerSecond => _normalSpeedMetersPerSecond;
        public float FastSpeedMetersPerSecond => _fastSpeedMetersPerSecond;
        public float AccelerationMetersPerSecondSquared => _accelerationMetersPerSecondSquared;
        public float DecelerationMetersPerSecondSquared => _decelerationMetersPerSecondSquared;
        public float MovementInputThreshold => _movementInputThreshold;
        public float CollisionSweepSkinMeters => _collisionSweepSkinMeters;
        public float YawSpeedDegreesPerSecond => _yawSpeedDegreesPerSecond;
        public float PitchSpeedDegreesPerSecond => _pitchSpeedDegreesPerSecond;
        public float UnderwaterColliderRotationSpeedDegreesPerSecond => _underwaterColliderRotationSpeedDegreesPerSecond;
    }
}
