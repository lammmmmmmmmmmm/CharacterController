using UnityEngine;

namespace PhysicsCharacterController
{
    /// <summary>
    /// Contract for components that apply additive velocity forces (e.g., wind, sliding, knockback).
    /// </summary>
    public interface IVelocityModifier
    {
        /// <summary>
        /// Calculates the additive velocity vector this modifier contributes.
        /// </summary>
        /// <param name="currentVelocity">The rigidbody's current velocity.</param>
        /// <param name="desiredMovement">The intended movement velocity from input.</param>
        Vector3 GetVelocityContribution(Vector3 currentVelocity, Vector3 desiredMovement);
    }
}