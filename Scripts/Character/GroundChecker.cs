using UnityEngine;

namespace PhysicsCharacterController
{
    /// <summary>
    /// Performs ground detection using sphere checks.
    /// Attach to the character and configure via Inspector.
    /// </summary>
    public class GroundChecker : MonoBehaviour
    {
        [SerializeField] private LayerMask _groundMask;
        [SerializeField] private float _checkRadius = 0.2f;
        [SerializeField] private float _feetOffset = 1f;

        private bool _wasGrounded;
        private bool _isGrounded;
        private RaycastHit _groundHit;

        public bool IsGrounded => _isGrounded;
        public bool WasGrounded => _wasGrounded;
        public bool JustLanded => _isGrounded && !_wasGrounded;
        public bool JustLeftGround => !_isGrounded && _wasGrounded;
        public RaycastHit GroundHit => _groundHit;

        private void FixedUpdate()
        {
            Check();
        }

        private void Check()
        {
            _wasGrounded = _isGrounded;

            Vector3 castOrigin = transform.position;
            float castDistance = _feetOffset;

            _isGrounded = Physics.SphereCast(castOrigin, _checkRadius, Vector3.down, out _groundHit, castDistance, _groundMask);
        }

        private void OnDrawGizmosSelected()
        {
            Vector3 castOrigin = transform.position;
            float castDistance = _feetOffset;
            Vector3 castEnd = castOrigin + Vector3.down * castDistance;

            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(castOrigin, _checkRadius);

            Gizmos.color = _isGrounded ? Color.green : Color.red;
            Gizmos.DrawWireSphere(castEnd, _checkRadius);

            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(castOrigin, castEnd);
        }
    }
}
