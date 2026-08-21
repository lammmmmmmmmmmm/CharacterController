using Pathfinding;
using UnityEngine;

namespace PhysicsCharacterController
{
    public class AICharacterInput : BaseCharacterInput
    {
        [SerializeField] private Seeker _seeker;
        [SerializeField] private float _nextWaypointDistance = 3f;

        // Pathfinding variables
        private Path _currentPath;
        private int _currentWaypointIndex;
        private bool _reachedEndOfPath = true;
        private Vector3 _currentTarget;

        public bool HasReachedDestination => _reachedEndOfPath;
        public Vector3 CurrentTargetPositionMeters => _currentTarget;

        private void Update()
        {
            UpdateCurrentTarget();
        }

        public override float GetMoveAngle()
        {
            if (!AreNormalActionsEnabled)
            {
                return transform.eulerAngles.y;
            }

            Vector3 directionToTarget = (_currentTarget - transform.position).normalized;
            float angle = Mathf.Atan2(directionToTarget.x, directionToTarget.z) * Mathf.Rad2Deg;

            return angle;
        }

        public override Vector2 GetMoveInput()
        {
            if (!AreNormalActionsEnabled)
            {
                return Vector2.zero;
            }

            return _reachedEndOfPath ? Vector2.zero : Vector2.one;
        }

        public void SetTarget(Vector3 targetPosition)
        {
            _currentTarget = targetPosition;
            _seeker.StartPath(transform.position, targetPosition, OnPathComplete);
        }

        public void Stop()
        {
            _currentPath = null;
            _currentWaypointIndex = 0;
            _currentTarget = transform.position;
            if (!_reachedEndOfPath)
            {
                _reachedEndOfPath = true;
                InvokeMoveStop();
            }
        }

        private void UpdateCurrentTarget()
        {
            if (_currentPath == null || _currentPath.vectorPath == null)
                return;

            // Check if we need to move to the next waypoint
            if (_currentWaypointIndex >= _currentPath.vectorPath.Count)
            {
                _reachedEndOfPath = true;
                InvokeMoveStop();
                return;
            }

            float distanceToWaypoint = Vector3.Distance(transform.position, _currentTarget);

            // If close enough to current waypoint, move to next one
            if (distanceToWaypoint <= _nextWaypointDistance)
            {
                _currentWaypointIndex++;
                if (_currentWaypointIndex >= _currentPath.vectorPath.Count)
                {
                    _reachedEndOfPath = true;
                    InvokeMoveStop();
                    return;
                }
            }

            // Set current target to the waypoint we're moving towards
            _currentTarget = _currentPath.vectorPath[_currentWaypointIndex];
        }

        private void OnPathComplete(Path p)
        {
            if (!p.error)
            {
                _currentPath = p;
                _currentWaypointIndex = 0;
                _reachedEndOfPath = false;
                InvokeMoveStart(Vector2.one);
            }
        }
    }
}
