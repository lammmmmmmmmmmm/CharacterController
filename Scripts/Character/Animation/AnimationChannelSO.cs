using UnityEngine;

namespace PhysicsCharacterController
{
    [CreateAssetMenu(fileName = "Animation Channel", menuName = "Physics Character Controller/Animation Channel")]
    public sealed class AnimationChannelSO : ScriptableObject
    {
        [SerializeField, Min(1)] private int _layerIndex = 1;
        [SerializeField] private AvatarMask _defaultAvatarMask;
        [SerializeField] private bool _isAdditive;

        public int LayerIndex => _layerIndex;
        public AvatarMask DefaultAvatarMask => _defaultAvatarMask;
        public bool IsAdditive => _isAdditive;
    }
}
