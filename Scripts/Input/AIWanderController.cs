// using Pathfinding;
// using UnityEngine;

// namespace PhysicsCharacterController
// {
// 	/// <summary>
// 	/// AI controller that replaces InputReader for autonomous character movement.
// 	/// Provides wandering behavior using A* pathfinding with periodic stops.
// 	/// </summary>
// 	public class AIWanderController : BaseInputController
// 	{
// 		private const int MAX_PATH_ATTEMPTS = 3;
// 		private const int CIRCLE_SEARCH_POINTS = 8;

// 		[Header("Wandering Behavior")]
// 		[SerializeField] private float wanderRadius = 15f;
// 		[SerializeField] private float minWaitTime = 1f;
// 		[SerializeField] private float maxWaitTime = 4f;
// 		[SerializeField] private float arrivalDistance = 2f;

// 		[Header("Jump Behavior")]
// 		[SerializeField] private float jumpProbability = 0.3f;
// 		[SerializeField] private float jumpCooldown = 2f;

// 		[Header("Movement Settings")]
// 		[SerializeField] private float movementSpeed = 1f;
// 		[SerializeField] private float nextWaypointDistance = 3f;

// 		[Header("Debug")]
// 		[SerializeField] private bool enableDebugVisualization = true;

// 		private Vector3 _homePosition;
// 		private Vector3 _currentDestination;
// 		private AIState _currentState;
// 		private float _stateTimer;
// 		private float _nextWaitDuration;
// 		private float _lastPathUpdateTime;
// 		private float _lastJumpTime;

// 		// Pathfinding variables
// 		private Path _currentPath;
// 		private int _currentWaypoint;
// 		private bool _reachedEndOfPath;
// 		private Vector3 _currentTarget;
// 		private Seeker _seeker;

// 		private Camera _mainCamera;
// 		private CharacterManager _characterManager;

// 		private const float MAX_MOVEMENT_TIME = 5f; // Prevent stuck in movement

// 		private enum AIState
// 		{
// 			Wandering,
// 			Waiting,
// 			FindingNewDestination,
// 			Jumping,
// 			WaitingForGround
// 		}

// 		protected override void Awake()
// 		{
// 			base.Awake();
// 			_seeker = GetComponent<Seeker>();
// 			_characterManager = GetComponent<CharacterManager>();
// 			_homePosition = transform.position;
// 			_currentState = AIState.FindingNewDestination;
// 			_mainCamera = Camera.main;
// 			_lastJumpTime = -jumpCooldown; // Allow jumping immediately
// 		}

// 		private void Start()
// 		{
// 			FindNewDestination();
// 		}

// 		private void Update()
// 		{
// 			UpdateAIStateMachine();
// 			UpdateMovementInput();
// 		}

// 		private void UpdateMovementInput()
// 		{
// 			if (_currentState == AIState.Wandering && _currentPath != null && !_reachedEndOfPath)
// 			{
// 				// Calculate direction to target in world space
// 				Vector3 directionToTarget = (_currentTarget - transform.position).normalized;
// 				directionToTarget.y = 0f;

// 				if (directionToTarget.magnitude > 0.1f)
// 				{
// 					// Convert world direction to camera-relative input (like joystick would provide)
// 					// Get camera's forward and right directions (projected on horizontal plane)
// 					Vector3 cameraForward = Vector3.ProjectOnPlane(_mainCamera.transform.forward, Vector3.up).normalized;
// 					Vector3 cameraRight = Vector3.ProjectOnPlane(_mainCamera.transform.right, Vector3.up).normalized;

// 					// Convert world direction to camera-relative coordinates
// 					float horizontal = Vector3.Dot(directionToTarget, cameraRight);
// 					float vertical = Vector3.Dot(directionToTarget, cameraForward);

// 					// Set axis input like joystick would (normalized and scaled by movement speed)
// 					axisInput = new Vector2(horizontal, vertical).normalized * movementSpeed;

// 					// Clamp to valid joystick range
// 					axisInput = Vector2.ClampMagnitude(axisInput, 1f);
// 				}
// 				else
// 				{
// 					axisInput = Vector2.zero;
// 				}
// 			}
// 			// else
// 			// {
// 			//     axisInput = Vector2.zero;
// 			// }
// 		}

// 		private void UpdateAIStateMachine()
// 		{
// 			_stateTimer += Time.deltaTime;

// 			switch (_currentState)
// 			{
// 				case AIState.Wandering:
// 					HandleWanderingState();
// 					break;

// 				case AIState.Waiting:
// 					HandleWaitingState();
// 					break;

// 				case AIState.FindingNewDestination:
// 					FindNewDestination();
// 					break;

// 				case AIState.Jumping:
// 					HandleJumpingState();
// 					break;

// 				case AIState.WaitingForGround:
// 					HandleWaitingForGroundState();
// 					break;
// 			}
// 		}

// 		private void HandleWanderingState()
// 		{
// 			if (_currentPath == null)
// 			{
// 				// No path available, try to find new destination
// 				_currentState = AIState.FindingNewDestination;
// 				return;
// 			}

// 			// Check if we've reached the end of the path
// 			if (_reachedEndOfPath || _stateTimer > MAX_MOVEMENT_TIME)
// 			{
// 				TransitionToWaiting();
// 				return;
// 			}

// 			// Calculate next waypoint to move towards
// 			UpdateCurrentTarget();

// 			// Check if we're close enough to the final destination
// 			float distanceToDestination = Vector3.Distance(transform.position, _currentDestination);
// 			if (distanceToDestination <= arrivalDistance)
// 			{
// 				TransitionToWaiting();
// 			}
// 		}

// 		private void UpdateCurrentTarget()
// 		{
// 			if (_currentPath == null || _currentPath.vectorPath == null)
// 				return;

// 			// Check if we need to move to the next waypoint
// 			if (_currentWaypoint >= _currentPath.vectorPath.Count)
// 			{
// 				_reachedEndOfPath = true;
// 				return;
// 			}

// 			// Get distance to current waypoint
// 			Vector3 currentWaypointPos = _currentPath.vectorPath[_currentWaypoint];
// 			float distanceToWaypoint = Vector3.Distance(transform.position, currentWaypointPos);

// 			// If close enough to current waypoint, move to next one
// 			if (distanceToWaypoint < nextWaypointDistance)
// 			{
// 				_currentWaypoint++;
// 				if (_currentWaypoint >= _currentPath.vectorPath.Count)
// 				{
// 					_reachedEndOfPath = true;
// 					return;
// 				}
// 			}

// 			// Set current target to the waypoint we're moving towards
// 			_currentTarget = _currentPath.vectorPath[_currentWaypoint];
// 		}

// 		private void TransitionToWaiting()
// 		{
// 			// Only transition to waiting when grounded
// 			if (!_characterManager.GetGrounded())
// 			{
// 				_currentState = AIState.WaitingForGround;
// 				axisInput = Vector2.zero;
// 				return; // Stay in WaitingForGround state until grounded
// 			}

// 			_currentState = AIState.Waiting;
// 			_stateTimer = 0f;
// 			_nextWaitDuration = Random.Range(minWaitTime, maxWaitTime);
// 			axisInput = Vector2.zero;

// 			// Clear pathfinding data
// 			_currentPath = null;
// 			_currentWaypoint = 0;
// 			_reachedEndOfPath = false;
// 		}

// 		private void HandleWaitingState()
// 		{
// 			if (_stateTimer >= _nextWaitDuration)
// 			{
// 				// Randomly choose between finding new destination or jumping
// 				bool canJump = Time.time - _lastJumpTime >= jumpCooldown;
// 				bool shouldJump = canJump && Random.value < jumpProbability;

// 				if (shouldJump)
// 				{
// 					_currentState = AIState.Jumping;
// 				}
// 				else
// 				{
// 					_currentState = AIState.FindingNewDestination;
// 				}
// 				_stateTimer = 0f;
// 			}
// 		}

// 		private void HandleWaitingForGroundState()
// 		{
// 			// Keep checking if we're grounded, then transition to normal waiting
// 			if (_characterManager.GetGrounded())
// 			{
// 				_currentState = AIState.Waiting;
// 				_stateTimer = 0f;
// 				_nextWaitDuration = Random.Range(minWaitTime, maxWaitTime);
// 				axisInput = Vector2.zero;

// 				// Clear pathfinding data
// 				_currentPath = null;
// 				_currentWaypoint = 0;
// 				_reachedEndOfPath = false;
// 			}
// 			else
// 			{
// 				axisInput = transform.forward.normalized * (movementSpeed * 0.1f);
// 			}
// 		}

// 		private void FindNewDestination()
// 		{
// 			Vector3 randomDestination = GenerateRandomDestination();

// 			if (randomDestination != transform.position)
// 			{
// 				_currentDestination = randomDestination;
// 				_seeker.StartPath(transform.position, _currentDestination, OnPathComplete);
// 			}
// 			else
// 			{
// 				// If AI can't find valid destination, wait and try again
// 				TransitionToWaiting();
// 			}
// 		}

// 		private void HandleJumpingState()
// 		{
// 			// Trigger jump input for one frame
// 			if (_stateTimer < 0.1f) // Brief jump input duration
// 			{
// 				OnJump();
// 			}
// 			else
// 			{
// 				JumpEnded();

// 				// Record jump time and only transition to waiting when grounded
// 				_lastJumpTime = Time.time;
// 				TransitionToWaiting();
// 			}
// 		}

// 		private Vector3 GenerateRandomDestination()
// 		{
// 			Vector2 randomDirection = Random.insideUnitCircle * wanderRadius;
// 			Vector3 targetPosition = _homePosition + new Vector3(randomDirection.x, 0f, randomDirection.y);

// 			return FindNearestWalkablePosition(targetPosition);

// 			Vector3 FindNearestWalkablePosition(Vector3 desiredPosition)
// 			{
// 				if (AstarPath.active == null)
// 				{
// 					return desiredPosition;
// 				}

// 				var nearestNode = AstarPath.active.GetNearest(desiredPosition);
// 				if (nearestNode.node != null && nearestNode.node.Walkable)
// 				{
// 					return nearestNode.position;
// 				}

// 				// Search in expanding circles
// 				float searchRadius = 2f;
// 				for (int attempt = 0; attempt < MAX_PATH_ATTEMPTS; attempt++)
// 				{
// 					for (int i = 0; i < CIRCLE_SEARCH_POINTS; i++)
// 					{
// 						float angle = (360f / CIRCLE_SEARCH_POINTS) * i * Mathf.Deg2Rad;
// 						Vector3 searchPosition = desiredPosition + new Vector3(
// 							Mathf.Cos(angle) * searchRadius,
// 							0f,
// 							Mathf.Sin(angle) * searchRadius
// 						);

// 						var testNode = AstarPath.active.GetNearest(searchPosition);
// 						if (testNode.node != null && testNode.node.Walkable)
// 						{
// 							return testNode.position;
// 						}
// 					}
// 					searchRadius *= 1.5f;
// 				}

// 				return transform.position;
// 			}
// 		}

// 		private void OnPathComplete(Path p)
// 		{
// 			if (!p.error)
// 			{
// 				_currentPath = p;
// 				_currentWaypoint = 0;
// 				_reachedEndOfPath = false;
// 				_currentState = AIState.Wandering;
// 				_stateTimer = 0f;
// 			}
// 			else
// 			{
// 				// Path failed, try waiting and then find new destination
// 				TransitionToWaiting();
// 			}
// 		}

// 		public override void OnJump()
// 		{
// 			if (_currentState == AIState.Jumping)
// 			{
// 				jump = true;
// 			}
// 		}

// 		public override void JumpEnded()
// 		{
// 			jump = false;
// 		}

// 		#region Debug Visualization

// 		private void OnDrawGizmosSelected()
// 		{
// 			if (!enableDebugVisualization) return;

// 			// Draw home position and wander radius
// 			Gizmos.color = Color.blue;
// 			Gizmos.DrawWireSphere(_homePosition, 0.5f);

// 			Gizmos.color = Color.yellow;
// 			DrawWireCircle(Application.isPlaying ? _homePosition : transform.position, wanderRadius);

// 			// Draw current destination
// 			if (_currentDestination != Vector3.zero)
// 			{
// 				Gizmos.color = _currentState == AIState.Wandering ? Color.green : Color.red;
// 				Gizmos.DrawSphere(_currentDestination, 0.3f);
// 			}

// 			// Draw current path
// 			if (_currentPath != null && _currentPath.vectorPath != null && _currentPath.vectorPath.Count > 1)
// 			{
// 				Gizmos.color = Color.cyan;
// 				for (int i = 0; i < _currentPath.vectorPath.Count - 1; i++)
// 				{
// 					Gizmos.DrawLine(_currentPath.vectorPath[i], _currentPath.vectorPath[i + 1]);
// 				}

// 				// Draw waypoints
// 				Gizmos.color = Color.white;
// 				for (int i = 0; i < _currentPath.vectorPath.Count; i++)
// 				{
// 					float size = (i == _currentWaypoint) ? 0.3f : 0.1f;
// 					Gizmos.color = (i == _currentWaypoint) ? Color.magenta : Color.white;
// 					Gizmos.DrawSphere(_currentPath.vectorPath[i], size);
// 				}

// 				// Draw current target
// 				if (_currentTarget != Vector3.zero)
// 				{
// 					Gizmos.color = Color.red;
// 					Gizmos.DrawLine(transform.position, _currentTarget);
// 					Gizmos.DrawWireSphere(_currentTarget, 0.5f);
// 				}
// 			}

// 			// Draw state information
// #if UNITY_EDITOR
// 			if (Application.isPlaying)
// 			{
// 				Gizmos.color = Color.white;
// 				Vector3 labelPosition = transform.position + Vector3.up * 3f;

// 				string stateInfo = $"State: {_currentState}\nTimer: {_stateTimer:F1}s";
// 				if (_currentState == AIState.Waiting)
// 				{
// 					stateInfo += $"\nWait: {_nextWaitDuration:F1}s";
// 				}
// 				if (_currentState == AIState.Jumping)
// 				{
// 					stateInfo += $"\nJump Cooldown: {Mathf.Max(0f, jumpCooldown - (Time.time - _lastJumpTime)):F1}s";
// 				}
// 				if (_currentState == AIState.WaitingForGround)
// 				{
// 					stateInfo += $"\nWaiting for ground: {!_characterManager.GetGrounded()}";
// 				}
// 				stateInfo += $"\nGrounded: {_characterManager.GetGrounded()}";
// 				if (_currentPath != null)
// 				{
// 					stateInfo += $"\nWaypoint: {_currentWaypoint}/{_currentPath.vectorPath?.Count ?? 0}";
// 					stateInfo += $"\nReached End: {_reachedEndOfPath}";
// 				}
// 				UnityEditor.Handles.Label(labelPosition, stateInfo);
// 			}
// #endif
// 		}

// 		private void DrawWireCircle(Vector3 center, float radius)
// 		{
// 			const int segments = 32;
// 			float angleStep = 360f / segments;
// 			Vector3 previousPoint = center + Vector3.forward * radius;

// 			for (int i = 1; i <= segments; i++)
// 			{
// 				float angle = angleStep * i * Mathf.Deg2Rad;
// 				Vector3 currentPoint = center + new Vector3(Mathf.Sin(angle), 0f, Mathf.Cos(angle)) * radius;
// 				Gizmos.DrawLine(previousPoint, currentPoint);
// 				previousPoint = currentPoint;
// 			}
// 		}

// 		#endregion
// 	}
// }
