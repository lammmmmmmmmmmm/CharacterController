using System;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.EnhancedTouch;
using UnityEngine.InputSystem.Layouts;
using UnityEngine.InputSystem.OnScreen;
using Touch = UnityEngine.InputSystem.EnhancedTouch.Touch;

/// <summary>
/// An on-screen touch field control that sends delta movement to the Input System.
/// Extends OnScreenControl to properly integrate with Unity's Input System.
/// </summary>
public class OnScreenTouchField : OnScreenControl, IPointerDownHandler, IPointerUpHandler
{
	[InputControl(layout = "Vector2")]
	[SerializeField]
	private string m_ControlPath;

	[NonSerialized] public Vector2 TouchDist;
	[NonSerialized] public Vector2 PointerOld;
	private Vector2 _initialTouchPosition;
	[NonSerialized] public float DeltaMagnitudeDiff; // For zooming
	[NonSerialized] public int NumPressed = 0;
	[NonSerialized] public bool Pressed;

	private const float TOUCH_MATCH_TOLERANCE_SQUARED = 250000f;

	// Cached properties
	private readonly Touch[] _validTouches = new Touch[2];
	private int _validTouchCount;
	private Vector2 _touchZeroPrevPos;
	private Vector2 _touchOnePrevPos;

	protected override string controlPathInternal
	{
		get => m_ControlPath;
		set => m_ControlPath = value;
	}

	protected override void OnEnable()
	{
		base.OnEnable();
		EnhancedTouchSupport.Enable();
	}

	protected override void OnDisable()
	{
		base.OnDisable();
		EnhancedTouchSupport.Disable();
	}

	private bool IsTouchDirectlyOnPanel(Vector2 screenPosition)
	{
		PointerEventData pointerData = new(EventSystem.current)
		{
			position = screenPosition
		};

		var raycastResults = new System.Collections.Generic.List<RaycastResult>();
		EventSystem.current.RaycastAll(pointerData, raycastResults);

		if (raycastResults.Count > 0)
		{
			RaycastResult topResult = raycastResults[0];
			return topResult.gameObject == gameObject || topResult.gameObject.transform.IsChildOf(transform);
		}

		return false;
	}

	private void Update()
	{
		if (Pressed)
		{
			// Rotation Cam - Find the touch that matches our initial position
			Touch currentTouch = default;
			bool foundTouch = false;
			float minDistanceSquared = TOUCH_MATCH_TOLERANCE_SQUARED;

			foreach (var touch in Touch.activeTouches)
			{
				float distanceSquared = (touch.screenPosition - _initialTouchPosition).sqrMagnitude;
				if (distanceSquared < minDistanceSquared)
				{
					currentTouch = touch;
					foundTouch = true;
					minDistanceSquared = distanceSquared;
					_initialTouchPosition = touch.screenPosition; // Update for next frame
				}
			}

			if (foundTouch)
			{
				TouchDist = currentTouch.screenPosition - PointerOld;
				PointerOld = currentTouch.screenPosition;
			}
			else
			{
				TouchDist = Vector2.zero;
			}

			// Send the delta value to the control
			SendValueToControl(TouchDist);

			// Zoom Cam
			if (NumPressed == 2)
			{
				_validTouchCount = 0;
				var activeTouches = Touch.activeTouches;

				for (int i = 0; i < activeTouches.Count && _validTouchCount < 2; i++)
				{
					Touch touch = activeTouches[i];
					if (IsTouchDirectlyOnPanel(touch.screenPosition))
					{
						_validTouches[_validTouchCount++] = touch;
					}
				}

				if (_validTouchCount == 2)
				{
					Touch touchZero = _validTouches[0];
					Touch touchOne = _validTouches[1];

					_touchZeroPrevPos = touchZero.screenPosition - touchZero.delta;
					_touchOnePrevPos = touchOne.screenPosition - touchOne.delta;
					float prevTouchDeltaMag = (_touchZeroPrevPos - _touchOnePrevPos).magnitude;
					float touchDeltaMag = (touchZero.screenPosition - touchOne.screenPosition).magnitude;

					DeltaMagnitudeDiff = prevTouchDeltaMag - touchDeltaMag;
				}
			}
			else
			{
				DeltaMagnitudeDiff = 0;
			}
		}
		else
		{
			TouchDist = Vector2.zero;
		}
	}

	public void OnPointerDown(PointerEventData eventData)
	{
		++NumPressed;
		Pressed = true;
		_initialTouchPosition = eventData.position;
		PointerOld = eventData.position;
	}

	public void OnPointerUp(PointerEventData eventData)
	{
		--NumPressed;
		Pressed = false;

		// Reset to default state when pointer is released
		SentDefaultValueToControl();
	}
}