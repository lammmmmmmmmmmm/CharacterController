using UnityEngine;

namespace PhysicsCharacterController
{
    /// <summary>
    /// Detects walls around the character and cancels the velocity component pushing into them,
    /// so the character slides along walls instead of ramming the physics solver every tick.
    /// Without the cancellation, constant penetration/depenetration makes the body — and any
    /// camera tracking it — visibly shake when running against a wall.
    /// </summary>
    public class WallChecker : MonoBehaviour, IVelocityModifier
    {
        [Tooltip("Distance from the player head used to check if the player is touching a wall")]
        [SerializeField] private float _checkDistance = 0.8f;
        [Tooltip("Heights above the character center to cast wall rays. Near-vertical walls lean away, so their distance grows with height — a single high ray misses them while the capsule is already touching lower down")]
        [SerializeField] private float[] _checkHeightOffsetsMeters = { 0f, 0.5f };
        [SerializeField] private LayerMask _wallMask;
        [SerializeField] private BaseCharacterInput _input;

        private static readonly float[] CheckAngles = { 0f, 45f, 90f, 135f, 180f, 225f, 270f, 315f };

        public bool IsTouchingWall { get; private set; }
        public Vector3 WallNormal { get; private set; }

        private void FixedUpdate()
        {
            CheckWall(_input.HorizontalMoveDirection);
        }

        public Vector3 GetVelocityContribution(Vector3 currentVelocity, Vector3 desiredMovement)
        {
            if (!IsTouchingWall)
            {
                return Vector3.zero;
            }

            // Near-vertical walls have a slight vertical normal component; flatten it so the
            // cancellation never injects vertical velocity.
            Vector3 horizontalWallNormal = new(WallNormal.x, 0f, WallNormal.z);
            if (horizontalWallNormal.sqrMagnitude < 0.001f)
            {
                return Vector3.zero;
            }

            horizontalWallNormal.Normalize();

            float speedIntoWall = Vector3.Dot(desiredMovement, horizontalWallNormal);
            if (speedIntoWall >= 0f)
            {
                return Vector3.zero;
            }

            return -speedIntoWall * horizontalWallNormal;
        }

        private void CheckWall(Vector3 forwardDirection)
        {
            IsTouchingWall = false;
            WallNormal = Vector3.zero;

            foreach (float heightOffsetMeters in _checkHeightOffsetsMeters)
            {
                Vector3 checkOrigin = GetCheckOrigin(heightOffsetMeters);

                foreach (float angle in CheckAngles)
                {
                    Vector3 checkDirection = Quaternion.AngleAxis(angle, transform.up) * forwardDirection;

                    if (Physics.Raycast(checkOrigin, checkDirection, out RaycastHit hit, _checkDistance, _wallMask))
                    {
                        WallNormal = hit.normal;
                        IsTouchingWall = true;
                        return;
                    }
                }
            }
        }

        private Vector3 GetCheckOrigin(float heightOffsetMeters)
        {
            return new Vector3(transform.position.x, transform.position.y + heightOffsetMeters, transform.position.z);
        }

#if UNITY_EDITOR
        [Header("Debug")]
        [SerializeField] private bool _debug;
        [SerializeField] private Vector3 _debugForwardDirection = Vector3.forward;

        private void OnDrawGizmos()
        {
            if (!_debug) return;

            Gizmos.color = Color.black;

            foreach (float heightOffsetMeters in _checkHeightOffsetsMeters)
            {
                Vector3 checkOrigin = GetCheckOrigin(heightOffsetMeters);

                foreach (float angle in CheckAngles)
                {
                    Vector3 checkDirection = Quaternion.AngleAxis(angle, transform.up) * _debugForwardDirection;
                    Gizmos.DrawLine(checkOrigin, checkOrigin + checkDirection * _checkDistance);
                }
            }
        }
#endif
    }
}
