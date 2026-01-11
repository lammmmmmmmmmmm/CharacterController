using UnityEngine;

namespace PhysicsCharacterController
{
    public interface IMovementModifier
    {
        /// <summary>
        /// Returns a multiplier (0f to 1f+) to apply to the character's speed.
        /// </summary>
        /// <param name="intendedDirection">The direction the player wants to move (input relative to camera).</param>
        float GetSpeedMultiplier(Vector3 intendedDirection);
    }
}