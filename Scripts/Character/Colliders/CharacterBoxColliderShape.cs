using UnityEngine;

namespace PhysicsCharacterController
{
    /// <summary>
    /// Adapts a BoxCollider to the measurements required by the character controller.
    /// Crouching changes only the box height and vertical center, preserving its configured
    /// width, depth, horizontal center, and original feet position.
    /// </summary>
    [RequireComponent(typeof(BoxCollider))]
    public sealed class CharacterBoxColliderShape : CharacterColliderShape
    {
        private BoxCollider _boxCollider;

        public override Collider PhysicsCollider => _boxCollider;
        public override float HeightMeters => _boxCollider.size.y;
        public override Vector3 Center => _boxCollider.center;

        #region Private Methods

        protected override void CacheCollider()
        {
            _boxCollider = GetComponent<BoxCollider>();
        }

        protected override void SetHeightAndCenter(float heightMeters, Vector3 center)
        {
            Vector3 sizeMeters = _boxCollider.size;
            sizeMeters.y = heightMeters;

            _boxCollider.size = sizeMeters;
            _boxCollider.center = center;
        }

        #endregion
    }
}
