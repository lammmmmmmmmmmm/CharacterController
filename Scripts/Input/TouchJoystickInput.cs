using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Controls;

namespace PhysicsCharacterController
{
    /// <summary>
    /// Tracks one touch inside a UI joystick zone and exposes a normalized movement value directly.
    /// It owns the joystick visuals, deadzone, and touch lifecycle without creating a virtual Input
    /// System device, allowing character input to consume mobile movement without control-path wiring.
    /// </summary>
    [DefaultExecutionOrder(-100)]
    [DisallowMultipleComponent]
    [AddComponentMenu("UI/Touch Joystick Input")]
    public sealed class TouchJoystickInput : MonoBehaviour
    {
        private const int NO_TOUCH_ID = -1;

        private static readonly List<RaycastResult> RAYCAST_RESULTS = new();

        [Header("References")]
        [SerializeField] private RectTransform _touchZone;
        [SerializeField] private RectTransform _joystickBackground;
        [SerializeField] private RectTransform _joystickHandle;

        [Header("Behavior")]
        [SerializeField] private TouchJoystickMode _joystickMode = TouchJoystickMode.Dynamic;
        [SerializeField, Range(20f, 300f)] private float _handleRangePixels = 80f;
        [SerializeField, Range(0f, 0.5f)] private float _deadzone01 = 0.1f;
        [SerializeField, Range(0f, 25f)] private float _returnSpeedPerSecond = 15f;
        [SerializeField] private bool _hidesWhenIdle = true;

        private int _activeTouchId = NO_TOUCH_ID;
        private Canvas _rootCanvas;
        private CanvasGroup _joystickCanvasGroup;
        private Vector2 _fixedBackgroundPositionPixels;

        public event Action<Vector2> OnInputChanged;

        public bool IsDragging { get; private set; }
        public Vector2 InputValue { get; private set; }

        #region Unity Lifecycle

        private void Awake()
        {
            _rootCanvas = GetComponentInParent<Canvas>().rootCanvas;
            _joystickCanvasGroup = _joystickBackground.GetComponent<CanvasGroup>();
            _joystickCanvasGroup.blocksRaycasts = true;
            _fixedBackgroundPositionPixels = _joystickBackground.anchoredPosition;
        }

        private void OnEnable()
        {
            ResetInputState();
            SetVisuals(_joystickMode == TouchJoystickMode.Fixed && !_hidesWhenIdle);
        }

        private void Update()
        {
            Touchscreen touchscreen = Touchscreen.current;
            if (touchscreen == null)
            {
                CancelTouchWhenTouchscreenBecomesUnavailable();
                AnimateHandleReturn();
                return;
            }

            UpdateTouchscreen(touchscreen);
            AnimateHandleReturn();
        }

        private void OnDisable()
        {
            ResetInputState();
        }

        #endregion

        #region Private Methods

        private void CancelTouchWhenTouchscreenBecomesUnavailable()
        {
            if (!IsDragging)
            {
                return;
            }

            Debug.LogWarning(
                "Canceling joystick input because the touchscreen became unavailable.",
                this);
            EndTouch();
        }

        private void UpdateTouchscreen(Touchscreen touchscreen)
        {
            bool hasFoundActiveTouch = false;

            foreach (TouchControl touch in touchscreen.touches)
            {
                if (!touch.press.isPressed)
                {
                    continue;
                }

                int touchId = touch.touchId.ReadValue();
                Vector2 screenPositionPixels = touch.position.ReadValue();

                if (IsDragging && touchId == _activeTouchId)
                {
                    UpdateInputValue(screenPositionPixels);
                    hasFoundActiveTouch = true;
                    continue;
                }

                if (!IsDragging &&
                    touch.press.wasPressedThisFrame &&
                    IsScreenPositionEligible(screenPositionPixels))
                {
                    BeginTouch(touchId, screenPositionPixels);
                    hasFoundActiveTouch = true;
                }
            }

            if (IsDragging && !hasFoundActiveTouch)
            {
                EndTouch();
            }
        }

        private bool IsScreenPositionEligible(Vector2 screenPositionPixels)
        {
            if (!RectTransformUtility.RectangleContainsScreenPoint(
                _touchZone,
                screenPositionPixels,
                GetUiCamera()))
            {
                return false;
            }

            if (EventSystem.current == null)
            {
                return true;
            }

            RAYCAST_RESULTS.Clear();
            var pointerEvent = new PointerEventData(EventSystem.current)
            {
                position = screenPositionPixels
            };
            EventSystem.current.RaycastAll(pointerEvent, RAYCAST_RESULTS);

            if (RAYCAST_RESULTS.Count == 0)
            {
                return true;
            }

            Transform topmostRaycastTarget = RAYCAST_RESULTS[0].gameObject.transform;
            return topmostRaycastTarget == _touchZone ||
                   topmostRaycastTarget.IsChildOf(_touchZone);
        }

        private void BeginTouch(int touchId, Vector2 screenPositionPixels)
        {
            _activeTouchId = touchId;
            IsDragging = true;

            if (_joystickMode == TouchJoystickMode.Dynamic)
            {
                RectTransformUtility.ScreenPointToLocalPointInRectangle(
                    _joystickBackground.parent as RectTransform,
                    screenPositionPixels,
                    GetUiCamera(),
                    out Vector2 localPositionPixels);
                _joystickBackground.anchoredPosition = localPositionPixels;
                _joystickHandle.anchoredPosition = Vector2.zero;
            }

            SetVisuals(true);
            UpdateInputValue(screenPositionPixels);
        }

        private void UpdateInputValue(Vector2 screenPositionPixels)
        {
            RectTransformUtility.ScreenPointToLocalPointInRectangle(
                _joystickBackground,
                screenPositionPixels,
                GetUiCamera(),
                out Vector2 localPositionPixels);

            Vector2 clampedPositionPixels = Vector2.ClampMagnitude(
                localPositionPixels,
                _handleRangePixels);
            _joystickHandle.anchoredPosition = clampedPositionPixels;

            Vector2 normalizedInput = clampedPositionPixels / _handleRangePixels;
            float magnitude01 = normalizedInput.magnitude;

            if (magnitude01 < _deadzone01)
            {
                SetInputValue(Vector2.zero);
                return;
            }

            normalizedInput = normalizedInput.normalized *
                              Mathf.InverseLerp(_deadzone01, 1f, magnitude01);
            SetInputValue(normalizedInput);
        }

        private void EndTouch()
        {
            _activeTouchId = NO_TOUCH_ID;
            IsDragging = false;
            SetInputValue(Vector2.zero);

            if (_joystickMode == TouchJoystickMode.Dynamic)
            {
                _joystickBackground.anchoredPosition = _fixedBackgroundPositionPixels;
                SetVisuals(false);
                return;
            }

            if (_hidesWhenIdle)
            {
                SetVisuals(false);
            }
        }

        private void ResetInputState()
        {
            _activeTouchId = NO_TOUCH_ID;
            IsDragging = false;
            _joystickHandle.anchoredPosition = Vector2.zero;
            _joystickBackground.anchoredPosition = _fixedBackgroundPositionPixels;
            SetInputValue(Vector2.zero);
        }

        private void AnimateHandleReturn()
        {
            if (IsDragging || _joystickHandle.anchoredPosition == Vector2.zero)
            {
                return;
            }

            _joystickHandle.anchoredPosition = _returnSpeedPerSecond > 0f
                ? Vector2.Lerp(
                    _joystickHandle.anchoredPosition,
                    Vector2.zero,
                    Time.unscaledDeltaTime * _returnSpeedPerSecond)
                : Vector2.zero;

            if (_joystickHandle.anchoredPosition.sqrMagnitude < 0.01f)
            {
                _joystickHandle.anchoredPosition = Vector2.zero;
            }
        }

        private void SetInputValue(Vector2 inputValue)
        {
            if (InputValue == inputValue)
            {
                return;
            }

            InputValue = inputValue;
            OnInputChanged?.Invoke(InputValue);
        }

        private void SetVisuals(bool isVisible)
        {
            _joystickCanvasGroup.alpha = isVisible ? 1f : 0f;
        }

        private Camera GetUiCamera()
        {
            return _rootCanvas.renderMode == RenderMode.ScreenSpaceOverlay
                ? null
                : _rootCanvas.worldCamera;
        }

        #endregion
    }

    public enum TouchJoystickMode
    {
        Dynamic,
        Fixed
    }
}
