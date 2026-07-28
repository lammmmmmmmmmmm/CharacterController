using UnityEngine;

namespace PhysicsCharacterController
{
    /// <summary>
    /// Adapts a vertical CapsuleCollider to the measurements required by the character controller.
    /// Character systems depend on CharacterColliderShape, so capsule-specific properties remain
    /// isolated here and can be replaced without changing movement or crouching behavior.
    /// </summary>
    [RequireComponent(typeof(CapsuleCollider))]
    public sealed class CharacterCapsuleColliderShape : CharacterColliderShape
    {
        private CapsuleCollider _capsuleCollider;

        public override Collider PhysicsCollider => _capsuleCollider;
        public override float HeightMeters => _capsuleCollider.height;
        public override Vector3 Center => _capsuleCollider.center;

        #region Private Methods

        protected override void CacheCollider()
        {
            _capsuleCollider = GetComponent<CapsuleCollider>();
        }

        protected override void SetHeightAndCenter(float heightMeters, Vector3 center)
        {
            _capsuleCollider.height = heightMeters;
            _capsuleCollider.center = center;
        }

        #endregion
    }
}
