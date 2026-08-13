using UnityEngine;

namespace PhysicsCharacterController
{
    public interface ICameraLookInputSource
    {
        bool IsLookActive { get; }
        Vector2 ReadLookDeltaPixels();
    }
}
