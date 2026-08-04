using System.Collections.Generic;
using UnityEngine;

namespace PhysicsCharacterController
{
    /// <summary>
    /// Samples and spatially smooths the ground along the character's facing direction, then positions
    /// this GameObject at the character's eye level above that smoothed ground. It delegates crouch
    /// eye-height calculation to a separate calculator while remaining the only transform writer.
    /// </summary>
    public sealed class SmoothedGroundFollower : MonoBehaviour
    {
        [Header("Character")]
        [SerializeField] private Transform _characterTransform;
        [SerializeField] private CharacterColliderShape _characterColliderShape;
        [SerializeField] private CharacterCrouch _characterCrouch;

        [Header("Eye Position")]
        [SerializeField] private Vector2 _planarEyeOffsetMeters = new(0f, 0.045f);
        [SerializeField] private float _standingEyeOffsetFromColliderTopMeters = -0.44f;
        [SerializeField] private float _crouchedEyeOffsetFromColliderTopMeters;

        [Header("Ground Detection")]
        [SerializeField, Min(2)] private int _sampleCount = 12;
        [SerializeField, Min(0.01f)] private float _sampleSpacingMeters = 0.5f;
        [SerializeField, Min(0f)] private float _raycastStartHeightMeters = 5f;
        [SerializeField, Min(0.1f)] private float _raycastDistanceMeters = 20f;
        [SerializeField] private LayerMask _groundMask = ~0;
        [SerializeField] private QueryTriggerInteraction _triggerInteraction = QueryTriggerInteraction.Ignore;

        [Header("Path Smoothing")]
        [SerializeField, Range(1, 10)] private int _smoothingPassCount = 3;
        [SerializeField, Range(0f, 1f)] private float _neighborInfluence01 = 0.5f;
        [SerializeField, Min(0.001f)] private float _verticalSmoothTimeSeconds = 0.1f;

        [Header("Debug")]
        [SerializeField] private bool _drawDebugPath = true;

        private readonly List<Vector3> _sampledPoints = new();
        private readonly List<Vector3> _smoothedPoints = new();
        private readonly List<Vector3> _smoothingBuffer = new();
        private readonly List<Vector3> _rayOrigins = new();

        private ColliderRelativeEyeHeightCalculator _eyeHeightCalculator;
        private float _verticalVelocityMetersPerSecond;

        #region Unity Lifecycle

        private void Start()
        {
            float standingColliderHeightMeters = _characterColliderShape.OriginalHeightMeters;
            float crouchedColliderHeightMeters = standingColliderHeightMeters * _characterCrouch.CrouchHeightMultiplier;
            _eyeHeightCalculator = new ColliderRelativeEyeHeightCalculator(standingColliderHeightMeters, crouchedColliderHeightMeters, _standingEyeOffsetFromColliderTopMeters, _crouchedEyeOffsetFromColliderTopMeters);
        }

        private void LateUpdate()
        {
            SampleGroundPath();
            SmoothGroundPath();

            if (_smoothedPoints.Count == 0)
            {
                Debug.LogWarning(
                    $"{nameof(SmoothedGroundFollower)} on '{name}' has no ground samples. " +
                    "The eye target keeps its current position for this frame.",
                    this);
                return;
            }

            MoveToSmoothedEyePosition();
        }

        private void OnDrawGizmosSelected()
        {
            if (!_drawDebugPath)
            {
                return;
            }

            Gizmos.color = Color.white;
            Gizmos.DrawSphere(_characterTransform.position, 0.08f);

            Gizmos.color = Color.yellow;
            DrawGroundRayGizmos();
            DrawPathGizmos(_sampledPoints, true);

            Gizmos.color = Color.cyan;
            DrawPathGizmos(_smoothedPoints, false);
        }

        #endregion

        #region Private Methods

        private void MoveToSmoothedEyePosition()
        {
            float desiredHeightMeters = EvaluateSmoothedHeightAtCharacterMeters() + CalculateEyeHeightAboveGroundMeters();
            float smoothedHeightMeters = Mathf.SmoothDamp(transform.position.y, desiredHeightMeters, ref _verticalVelocityMetersPerSecond, _verticalSmoothTimeSeconds);
            Vector3 planarEyeWorldPosition = _characterTransform.TransformPoint(new Vector3(_planarEyeOffsetMeters.x, 0f, _planarEyeOffsetMeters.y));

            transform.position = new Vector3(planarEyeWorldPosition.x, smoothedHeightMeters, planarEyeWorldPosition.z);
        }

        private void SampleGroundPath()
        {
            _sampledPoints.Clear();
            _rayOrigins.Clear();
            Vector3 forward = GetPlanarCharacterForward();
            float startingDistanceMeters = -_sampleSpacingMeters;

            for (int sampleIndex = 0; sampleIndex < _sampleCount; sampleIndex++)
            {
                float forwardDistanceMeters = startingDistanceMeters + sampleIndex * _sampleSpacingMeters;
                Vector3 horizontalPoint = _characterTransform.position + forward * forwardDistanceMeters;
                Vector3 rayOrigin = horizontalPoint + Vector3.up * _raycastStartHeightMeters;
                _rayOrigins.Add(rayOrigin);

                if (Physics.Raycast(rayOrigin, Vector3.down, out RaycastHit groundHit, _raycastDistanceMeters, _groundMask, _triggerInteraction))
                {
                    _sampledPoints.Add(groundHit.point);
                    continue;
                }

                float fallbackHeightMeters = _sampledPoints.Count > 0 ? _sampledPoints[^1].y : _characterTransform.position.y - _characterColliderShape.FeetOffsetMeters * Mathf.Abs(_characterTransform.lossyScale.y);
                _sampledPoints.Add(new Vector3(horizontalPoint.x, fallbackHeightMeters, horizontalPoint.z));
            }
        }

        private void SmoothGroundPath()
        {
            _smoothedPoints.Clear();
            _smoothedPoints.AddRange(_sampledPoints);

            if (_smoothedPoints.Count < 3)
            {
                return;
            }

            for (int passIndex = 0; passIndex < _smoothingPassCount; passIndex++)
            {
                SmoothGroundPathOnce();
            }
        }

        private void SmoothGroundPathOnce()
        {
            _smoothingBuffer.Clear();
            _smoothingBuffer.Add(_smoothedPoints[0]);

            for (int pointIndex = 1; pointIndex < _smoothedPoints.Count - 1; pointIndex++)
            {
                Vector3 previousPoint = _smoothedPoints[pointIndex - 1];
                Vector3 currentPoint = _smoothedPoints[pointIndex];
                Vector3 nextPoint = _smoothedPoints[pointIndex + 1];
                float neighborAverageHeightMeters = (previousPoint.y + nextPoint.y) * 0.5f;
                float smoothedHeightMeters = Mathf.Lerp(currentPoint.y, neighborAverageHeightMeters, _neighborInfluence01);

                _smoothingBuffer.Add(new Vector3(currentPoint.x, smoothedHeightMeters, currentPoint.z));
            }

            _smoothingBuffer.Add(_smoothedPoints[^1]);
            _smoothedPoints.Clear();
            _smoothedPoints.AddRange(_smoothingBuffer);
        }

        private float EvaluateSmoothedHeightAtCharacterMeters()
        {
            if (_smoothedPoints.Count == 1)
            {
                return _smoothedPoints[0].y;
            }

            Vector3 forward = GetPlanarCharacterForward();
            float nearestHeightMeters = _smoothedPoints[0].y;
            float nearestDistanceMeters = float.MaxValue;

            for (int pointIndex = 0; pointIndex < _smoothedPoints.Count - 1; pointIndex++)
            {
                Vector3 firstPoint = _smoothedPoints[pointIndex];
                Vector3 secondPoint = _smoothedPoints[pointIndex + 1];
                float firstDistanceMeters = Vector3.Dot(firstPoint - _characterTransform.position, forward);
                float secondDistanceMeters = Vector3.Dot(secondPoint - _characterTransform.position, forward);
                bool containsCharacter = firstDistanceMeters <= 0f && secondDistanceMeters >= 0f || secondDistanceMeters <= 0f && firstDistanceMeters >= 0f;

                if (containsCharacter)
                {
                    float segmentRangeMeters = secondDistanceMeters - firstDistanceMeters;
                    float interpolation01 = Mathf.Abs(segmentRangeMeters) > 0.0001f
                        ? Mathf.InverseLerp(firstDistanceMeters, secondDistanceMeters, 0f)
                        : 0f;
                    return Mathf.Lerp(firstPoint.y, secondPoint.y, interpolation01);
                }

                float absoluteDistanceMeters = Mathf.Abs(firstDistanceMeters);
                if (absoluteDistanceMeters < nearestDistanceMeters)
                {
                    nearestDistanceMeters = absoluteDistanceMeters;
                    nearestHeightMeters = firstPoint.y;
                }
            }

            return nearestHeightMeters;
        }

        private float CalculateEyeHeightAboveGroundMeters()
        {
            float localEyeHeightMeters = _eyeHeightCalculator.CalculateEyeHeightMeters(_characterColliderShape.CurrentTopOffsetMeters, _characterColliderShape.HeightMeters);
            float localEyeHeightAboveGroundMeters = _characterColliderShape.FeetOffsetMeters + localEyeHeightMeters;
            return localEyeHeightAboveGroundMeters * Mathf.Abs(_characterTransform.lossyScale.y);
        }

        private Vector3 GetPlanarCharacterForward()
        {
            Vector3 forward = Vector3.ProjectOnPlane(_characterTransform.forward, Vector3.up);
            return forward.sqrMagnitude < 0.0001f
                ? Vector3.forward
                : forward.normalized;
        }

        private void DrawGroundRayGizmos()
        {
            for (int rayIndex = 0; rayIndex < _rayOrigins.Count; rayIndex++)
            {
                Vector3 rayOrigin = _rayOrigins[rayIndex];
                Gizmos.DrawLine(rayOrigin, rayOrigin + Vector3.down * _raycastDistanceMeters);
            }
        }

        private static void DrawPathGizmos(IReadOnlyList<Vector3> pathPoints, bool isWireframe)
        {
            for (int pointIndex = 0; pointIndex < pathPoints.Count; pointIndex++)
            {
                if (isWireframe)
                {
                    Gizmos.DrawWireSphere(pathPoints[pointIndex], 0.06f);
                }
                else
                {
                    Gizmos.DrawSphere(pathPoints[pointIndex], 0.04f);
                }

                if (pointIndex > 0)
                {
                    Gizmos.DrawLine(pathPoints[pointIndex - 1], pathPoints[pointIndex]);
                }
            }
        }

        #endregion
    }
}
