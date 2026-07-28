using UnityEngine;

namespace PhysicsCharacterController
{
    /// <summary>
    /// Defines the character-controller measurements shared by movement, crouching, and effects.
    /// Concrete shape components translate these measurements to a specific Unity collider while
    /// keeping the collider's original bottom position fixed during height changes.
    /// </summary>
    [DisallowMultipleComponent]
    public abstract class CharacterColliderShape : MonoBehaviour
    {
        private float _originalBottomOffsetMeters;

        public abstract Collider PhysicsCollider { get; }
        public abstract float HeightMeters { get; }
        public abstract Vector3 Center { get; }

        public float OriginalHeightMeters { get; private set; }
        public float OriginalTopOffsetMeters => _originalBottomOffsetMeters + OriginalHeightMeters;
        public float CurrentTopOffsetMeters => Center.y + HeightMeters * 0.5f;
        public float FeetOffsetMeters => HeightMeters * 0.5f - Center.y;

        #region Unity Lifecycle

        protected virtual void Awake()
        {
            RefreshColliderCache();
            OriginalHeightMeters = HeightMeters;
            _originalBottomOffsetMeters = Center.y - OriginalHeightMeters * 0.5f;
        }

        #endregion

        #region Public Methods

        public void SetHeightPreservingBottom(float heightMeters)
        {
            float centerYMeters = _originalBottomOffsetMeters + heightMeters * 0.5f;
            SetHeightAndCenter(heightMeters, new Vector3(Center.x, centerYMeters, Center.z));
        }

        #endregion

        #region Internal Methods

        internal void RefreshColliderCache()
        {
            CacheCollider();
        }

        #endregion

        #region Private Methods

        protected abstract void CacheCollider();
        protected abstract void SetHeightAndCenter(float heightMeters, Vector3 center);

        #endregion
    }
}
