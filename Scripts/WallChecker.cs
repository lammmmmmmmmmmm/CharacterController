using UnityEngine;

namespace PhysicsCharacterController
{
    public class WallChecker : MonoBehaviour
    {
        [Tooltip("Distance from the player head used to check if the player is touching a wall")]
        [SerializeField] private float _checkDistance = 0.8f;
        [Tooltip("Wall checker distance from the player center")]
        [SerializeField] private float _heightOffset = 0.5f;
        [SerializeField] private LayerMask _wallMask;
        [SerializeField] private BaseCharacterInput _input;

        private static readonly float[] CheckAngles = { 0f, 45f, 90f, 135f, 180f, 225f, 270f, 315f };

        public bool IsTouchingWall { get; private set; }
        public Vector3 WallNormal { get; private set; }

        private void FixedUpdate()
        {
            CheckWall(_input.HorizontalMoveDirection);
        }

        private void CheckWall(Vector3 forwardDirection)
        {
            IsTouchingWall = false;
            WallNormal = Vector3.zero;

            Vector3 checkOrigin = GetCheckOrigin();

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

        private Vector3 GetCheckOrigin()
        {
            return new Vector3(transform.position.x, transform.position.y + _heightOffset, transform.position.z);
        }

#if UNITY_EDITOR
        [Header("Debug")]
        [SerializeField] private bool _debug;
        [SerializeField] private Vector3 _debugForwardDirection = Vector3.forward;

        private void OnDrawGizmos()
        {
            if (!_debug) return;

            Gizmos.color = Color.black;
            Vector3 checkOrigin = GetCheckOrigin();

            foreach (float angle in CheckAngles)
            {
                Vector3 checkDirection = Quaternion.AngleAxis(angle, transform.up) * _debugForwardDirection;
                Gizmos.DrawLine(checkOrigin, checkOrigin + checkDirection * _checkDistance);
            }
        }
#endif
    }
}
