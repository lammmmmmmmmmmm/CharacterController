using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace PhysicsCharacterController
{
	[RequireComponent(typeof(Collider))]
	[RequireComponent(typeof(Rigidbody))]
	public class MovingPlatform : MonoBehaviour
	{
		[Header("Platform movement")]
		[SerializeField] private Vector3[] _destinations;
		[SerializeField] private float _timeDelay;
		[SerializeField] private float _timeDelayBeginningEnd;
		[SerializeField] private float _platformSpeedDamp;
		[SerializeField] private bool _smoothMovement;
		[Space(10)]
		[SerializeField] private bool _canTranslate = true;

		[Header("Platform rotation")]
		[SerializeField] private Vector3 _rotationSpeed;
		[Space(10)]
		[SerializeField] private bool _canRotate = true;
		[SerializeField] private bool _canBeMoved;

		private Vector3 _nextDestination;
		private int _currentDestination = 0;
		private Vector3 _velocity = Vector3.zero;
		private bool _canMove = true;

		private readonly List<Rigidbody> _rigidbodies = new();

		private Vector3 _lastEulerAngles;
		private Vector3 _lastPosition;
		private Transform _transform;
		private Rigidbody _rigidbody;

		private void Awake()
		{
			_transform = GetComponent<Transform>();
			_lastPosition = _transform.position;
			_lastEulerAngles = _transform.eulerAngles;
			_rigidbody = GetComponent<Rigidbody>();

			if (_canTranslate)
			{
				_transform.position = _destinations[0];
			}
			_nextDestination = _transform.position;
		}

		private void FixedUpdate()
		{
			UpdateDestination();
			UpdatePositionAndRotation();
			UpdateBodies();
		}

		#region Platform and Rigidbody
		private void UpdateDestination()
		{
			if (Vector3.Distance(_transform.position, _nextDestination) <= 0.01f)
			{
				_rigidbody.position = _nextDestination;

				if ((_currentDestination == 0 || _currentDestination == _destinations.Length - 1) && _canMove)
				{
					StartCoroutine(WaitTime(_timeDelayBeginningEnd));
				}
				else if (_canMove)
				{
					StartCoroutine(WaitTime(_timeDelay));
				}

				SetNextDestination();
			}
		}

		private void UpdatePositionAndRotation()
		{
			if (_canMove)
			{
				if (_canTranslate)
				{
					if (_smoothMovement)
					{
						_rigidbody.position = Vector3.SmoothDamp(_transform.position, _nextDestination, ref _velocity,
							_platformSpeedDamp * Time.deltaTime);
					}
					else
					{
						_rigidbody.position = Vector3.MoveTowards(_transform.position, _nextDestination,
							_platformSpeedDamp * Time.deltaTime);
					}
				}

				if (_canRotate)
				{
					if (!_canBeMoved)
					{
						_transform.Rotate(_rotationSpeed.x * Time.deltaTime, _rotationSpeed.y * Time.deltaTime,
							_rotationSpeed.z * Time.deltaTime);
					}
					else
					{
						_rigidbody.AddTorque(new Vector3(_rotationSpeed.x, _rotationSpeed.y, _rotationSpeed.z),
							ForceMode.Force);
					}
				}
			}
		}

		private void UpdateBodies()
		{
			if (_rigidbodies.Count > 0)
			{
				Vector3 velocity = _transform.position - _lastPosition;
				Vector3 angularVelocity = _transform.eulerAngles - _lastEulerAngles;

				foreach (var rb in _rigidbodies)
				{
					CharacterManager characterManager = rb.GetComponent<CharacterManager>();
					if (characterManager && characterManager.IsJumping()) continue;

					if (angularVelocity.y > 0)
					{
						rb.transform.RotateAround(_transform.position, Vector3.up, angularVelocity.y);
						try
						{
							// characterManager.targetAngle += angularVelocity.y;
						}
						catch
						{
							/* Debug.Log("There is no player on the platform") */
						}
					}

					if (_rigidbody.linearVelocity.magnitude > 0) rb.linearVelocity += _rigidbody.linearVelocity;

					rb.position += velocity;
				}
			}

			_lastPosition = _transform.position;
			_lastEulerAngles = _transform.eulerAngles;
		}
		#endregion

		#region Handle list
		public void Add(Rigidbody rb)
		{
			if (!_rigidbodies.Contains(rb)) _rigidbodies.Add(rb);
		}

		public void Remove(Rigidbody rb)
		{
			if (_rigidbodies.Contains(rb)) _rigidbodies.Remove(rb);
		}
		#endregion

		#region Platform Handlers
		private void SetNextDestination()
		{
			_currentDestination++;
			if (_currentDestination > _destinations.Length - 1) _currentDestination = 0;

			_nextDestination = _destinations[_currentDestination];
		}

		private IEnumerator WaitTime(float time)
		{
			_canMove = false;
			yield return new WaitForSeconds(time);
			_canMove = true;
		}
		#endregion
	}
}