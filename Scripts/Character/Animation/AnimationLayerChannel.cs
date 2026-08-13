using Animancer;
using UnityEngine;

namespace PhysicsCharacterController
{
    public sealed class AnimationLayerChannel
    {
        private readonly AnimationChannelSO _animationChannelSO;
        private readonly AnimancerLayer _layer;

        private object _owner;
        private object _lastRejectedOwner;
        private int _priority;
        private AnimationClip _activeAnimationClip;

        public AnimationLayerChannel(AnimationChannelSO animationChannelSO, AnimancerLayer layer)
        {
            _animationChannelSO = animationChannelSO;
            _layer = layer;

            _layer.Mask = animationChannelSO.DefaultAvatarMask;
            _layer.IsAdditive = animationChannelSO.IsAdditive;
            _layer.Weight = 0f;
        }

        #region Public Methods

        public bool Play(object animationOwner, int animationPriority, AnimationClip animationClip, float fadeDurationSeconds)
        {
            if (!CanControl(animationOwner, animationPriority))
            {
                LogRejectedPlayOnce(animationOwner, animationClip);
                return false;
            }

            _lastRejectedOwner = null;
            bool hasOwnerChanged = _owner != animationOwner;
            _owner = animationOwner;
            _priority = animationPriority;

            if (!hasOwnerChanged && _activeAnimationClip == animationClip)
            {
                return true; // Same owner, same clip: nothing to do.
            }

            _layer.Play(animationClip, fadeDurationSeconds);
            _layer.StartFade(1f, fadeDurationSeconds);
            _activeAnimationClip = animationClip;
            return true;
        }

        public bool Stop(object animationOwner, float fadeDurationSeconds)
        {
            if (_owner == null)
            {
                return true;
            }

            if (_owner != animationOwner)
            {
                Debug.LogWarning($"Ignoring animation stop for channel '{_animationChannelSO.name}' because the requester does not own it.");
                return false;
            }

            _layer.StartFade(0f, fadeDurationSeconds);
            ClearOwnership();
            return true;
        }

        public void Reset()
        {
            _layer.Weight = 0f;
            ClearOwnership();
            _lastRejectedOwner = null;
        }

        #endregion

        #region Private Methods

        private bool CanControl(object animationOwner, int animationPriority)
        {
            return _owner == null
                || _owner == animationOwner
                || animationPriority > _priority;
        }

        private void LogRejectedPlayOnce(object animationOwner, AnimationClip animationClip)
        {
            if (_lastRejectedOwner == animationOwner)
            {
                return;
            }

            _lastRejectedOwner = animationOwner;
            Debug.LogWarning(
                $"Ignoring animation '{animationClip.name}' on channel '{_animationChannelSO.name}' because another owner controls it at priority {_priority}.");
        }

        private void ClearOwnership()
        {
            _owner = null;
            _priority = 0;
            _activeAnimationClip = null;
        }

        #endregion
    }
}
