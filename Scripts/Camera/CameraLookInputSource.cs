using System;
using System.Collections.Generic;
using UnityEngine;

namespace PhysicsCharacterController
{
    public sealed class CameraLookInputSource : MonoBehaviour
    {
        private readonly List<Registration> _registrations = new();

        #region Public Methods

        public IDisposable Register(ICameraLookInputSource inputSource, int priority)
        {
            if (inputSource == null)
            {
                throw new ArgumentNullException(nameof(inputSource));
            }

            var registration = new Registration(this, inputSource, priority);
            _registrations.Add(registration);
            return registration;
        }

        public Vector2 ReadMobileLookDeltaPixels()
        {
            Registration selectedRegistration = null;
            foreach (Registration registration in _registrations)
            {
                if (!registration.InputSource.IsLookActive ||
                    selectedRegistration != null && registration.Priority <= selectedRegistration.Priority)
                {
                    continue;
                }

                selectedRegistration = registration;
            }

            return selectedRegistration?.InputSource.ReadLookDeltaPixels() ?? Vector2.zero;
        }

        #endregion

        #region Private Methods

        private void Unregister(Registration registration)
        {
            if (!_registrations.Remove(registration))
            {
                Debug.LogWarning("Cannot release a camera-look input registration that is not owned by this registry.", this);
            }
        }

        private sealed class Registration : IDisposable
        {
            private readonly CameraLookInputSource _owner;
            private bool _isDisposed;

            public ICameraLookInputSource InputSource { get; }
            public int Priority { get; }

            public Registration(CameraLookInputSource owner, ICameraLookInputSource inputSource, int priority)
            {
                _owner = owner;
                InputSource = inputSource;
                Priority = priority;
            }

            public void Dispose()
            {
                if (_isDisposed)
                {
                    Debug.LogWarning("Camera-look input registration was released more than once.", _owner);
                    return;
                }

                _isDisposed = true;
                _owner.Unregister(this);
            }
        }

        #endregion
    }
}
