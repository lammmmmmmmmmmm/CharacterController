using System.Collections.Generic;
using Animancer;
using UnityEngine;

namespace PhysicsCharacterController
{
    public class CharacterAnimator : MonoBehaviour
    {
        private const int BASE_LAYER_INDEX = 0;

        [SerializeField] private AnimancerComponent _animancer;
        [SerializeField] private TransitionLibrary _transitions;
        [SerializeField] private AnimationChannelSO[] _animationChannelSOs;

        private readonly Dictionary<AnimationChannelSO, AnimationLayerChannel> _animationChannels = new();
        private LinearMixerTransition _activeTransitionMixer;

        private AnimancerState _activeTransitionState;
        private AnimancerState _currentBaseState;

        private LinearMixerTransition _queuedBaseMixer;
        private ClipTransition _queuedBaseClip;
        private float _queuedBaseSourceSpeed;

        private bool _isTransitionPlayingOnBaseLayer;

        private AnimancerLayer BaseLayer => _animancer.Layers[BASE_LAYER_INDEX];

        public StateId CurrentTag { get; private set; }

        #region Unity Lifecycle

        private void Awake()
        {
            InitializeAnimationChannels();
        }

        private void OnEnable()
        {
            ResetRuntimeStateCache();
        }

        private void Update()
        {
            SynchronizeBaseStateWithTransition();
        }

        private void OnDisable()
        {
            CancelPendingTransitionPlayback();
            ResetAnimationChannels();
            _currentBaseState = null;
            CurrentTag = null;
        }

        private void OnDestroy()
        {
            CancelPendingTransitionPlayback();
        }

        #endregion

        #region Public Methods

        public void SetBase(LinearMixerTransition mixer, StateId tag, float sourceSpeed)
        {
            if (TryGetTransitionSelection(tag, sourceSpeed, out var selection))
            {
                CancelPendingTransitionPlayback();
                QueueBaseAnimation(mixer, sourceSpeed);
                PlayOneShotOnBaseLayer(selection, sourceSpeed);
            }
            else
            {
                CancelPendingTransitionPlayback();
                _currentBaseState = BaseLayer.Play(mixer);
            }

            CurrentTag = tag;
        }

        public void SetBase(ClipTransition clip, StateId tag, float sourceSpeed)
        {
            if (TryGetTransitionSelection(tag, sourceSpeed, out var selection))
            {
                CancelPendingTransitionPlayback();
                QueueBaseAnimation(clip);
                PlayOneShotOnBaseLayer(selection, sourceSpeed);
            }
            else
            {
                CancelPendingTransitionPlayback();
                _currentBaseState = BaseLayer.Play(clip);
            }

            CurrentTag = tag;
        }

        public void UpdateTransitionMixerParameter(float sourceSpeed)
        {
            if (!TrySetMixerParameter(_activeTransitionMixer, sourceSpeed))
            {
                _activeTransitionMixer = null;
            }
        }

        public void UpdateLocomotionAnimationParameters(LinearMixerTransition locomotionMixer, float sourceSpeed)
        {
            TrySetMixerParameter(locomotionMixer, sourceSpeed);
        }

        public bool Play(
            AnimationChannelSO animationChannelSO,
            object animationOwner,
            int animationPriority,
            AnimationClip animationClip,
            float fadeDurationSeconds)
        {
            if (animationOwner == null || animationClip == null || fadeDurationSeconds < 0f)
            {
                Debug.LogError(
                    $"Cannot play animation on '{name}'. Owner and clip are required, and fade duration cannot be negative.",
                    this);
                return false;
            }

            if (!TryGetAnimationChannel(animationChannelSO, out AnimationLayerChannel animationChannel))
            {
                return false;
            }

            return animationChannel.Play(animationOwner, animationPriority, animationClip, fadeDurationSeconds);
        }

        public bool Stop(AnimationChannelSO animationChannelSO, object animationOwner, float fadeDurationSeconds)
        {
            if (animationOwner == null || fadeDurationSeconds < 0f)
            {
                Debug.LogError(
                    $"Cannot stop an animation channel on '{name}'. Owner is required, and fade duration cannot be negative.",
                    this);
                return false;
            }

            if (!TryGetAnimationChannel(animationChannelSO, out AnimationLayerChannel animationChannel))
            {
                return false;
            }

            return animationChannel.Stop(animationOwner, fadeDurationSeconds);
        }

        #endregion

        #region Private Methods

        private void InitializeAnimationChannels()
        {
            _animationChannels.Clear();
            var configuredLayerIndices = new HashSet<int>();

            foreach (AnimationChannelSO animationChannelSO in _animationChannelSOs)
            {
                if (animationChannelSO.LayerIndex <= BASE_LAYER_INDEX)
                {
                    Debug.LogError(
                        $"Animation channel '{animationChannelSO.name}' targets reserved base layer {BASE_LAYER_INDEX}. Overlay channels must use a higher layer index.",
                        this);
                    continue;
                }

                if (_animationChannels.ContainsKey(animationChannelSO))
                {
                    Debug.LogError($"Animation channel '{animationChannelSO.name}' is configured more than once.", this);
                    continue;
                }

                if (!configuredLayerIndices.Add(animationChannelSO.LayerIndex))
                {
                    Debug.LogError(
                        $"Animation channel '{animationChannelSO.name}' reuses Animancer layer {animationChannelSO.LayerIndex}. Layer indices must be unique.",
                        this);
                    continue;
                }

                AnimancerLayer layer = _animancer.Layers[animationChannelSO.LayerIndex];
                _animationChannels.Add(animationChannelSO, new AnimationLayerChannel(animationChannelSO, layer));
            }
        }

        private bool TryGetAnimationChannel(AnimationChannelSO animationChannelSO, out AnimationLayerChannel animationChannel)
        {
            if (animationChannelSO == null)
            {
                Debug.LogError($"Cannot control an animation channel on '{name}' because no channel was provided.", this);
                animationChannel = null;
                return false;
            }

            if (_animationChannels.TryGetValue(animationChannelSO, out animationChannel))
            {
                return true;
            }

            Debug.LogError($"Animation channel '{animationChannelSO.name}' is not configured on '{name}'.", this);
            return false;
        }

        private bool TryGetTransitionSelection(StateId newTag, float sourceSpeed, out TransitionLibrary.TransitionSelection selection)
        {
            if (!CurrentTag || CurrentTag == newTag)
            {
                CancelPendingTransitionPlayback();
                selection = default;
                return false;
            }

            if (_transitions.TryGet(CurrentTag, newTag, sourceSpeed, out var transition))
            {
                selection = transition;
                return true;
            }

            selection = default;
            return false;
        }

        private void QueueBaseAnimation(LinearMixerTransition mixer, float sourceSpeed)
        {
            _queuedBaseClip = null;
            _queuedBaseMixer = mixer;
            _queuedBaseSourceSpeed = sourceSpeed;
        }

        private void QueueBaseAnimation(ClipTransition clip)
        {
            _queuedBaseMixer = null;
            _queuedBaseClip = clip;
            _queuedBaseSourceSpeed = 0f;
        }

        private void PlayOneShotOnBaseLayer(TransitionLibrary.TransitionSelection selection, float sourceSpeed)
        {
            if (selection.Mode == TransitionLibrary.TransitionMode.Mixer)
            {
                _activeTransitionMixer = selection.Mixer;
                _activeTransitionState = BaseLayer.Play(selection.Mixer);
                TrySetMixerParameter(selection.Mixer, sourceSpeed);
            }
            else
            {
                _activeTransitionMixer = null;
                _activeTransitionState = BaseLayer.Play(selection.Clip);
            }

            _isTransitionPlayingOnBaseLayer = true;
            _activeTransitionState.Events(this).OnEnd = FadeToQueuedBaseAnimation;
        }

        private void FadeToQueuedBaseAnimation()
        {
            if (!_isTransitionPlayingOnBaseLayer)
            {
                return;
            }

            // The transition may be a clip (no mixer), or the mixer may have been nulled by
            // UpdateTransitionMixerParameter when its state was momentarily invalid. In either
            // case the queue-time speed seeded in QueueBaseAnimation remains the correct value.
            if (_activeTransitionMixer != null && IsRuntimeStateValid(_activeTransitionMixer.State))
            {
                _queuedBaseSourceSpeed = _activeTransitionMixer.State.Parameter;
            }

            _isTransitionPlayingOnBaseLayer = false;
            _activeTransitionMixer = null;
            if (IsRuntimeStateValid(_activeTransitionState))
            {
                _activeTransitionState.Events(this).OnEnd = null;
            }
            _activeTransitionState = null;

            if (_queuedBaseMixer != null)
            {
                _currentBaseState = BaseLayer.Play(_queuedBaseMixer);
                TrySetMixerParameter(_queuedBaseMixer, _queuedBaseSourceSpeed);
            }
            else if (_queuedBaseClip != null)
            {
                _currentBaseState = BaseLayer.Play(_queuedBaseClip);
            }

            ClearQueuedBaseAnimation();
        }

        private void CancelPendingTransitionPlayback()
        {
            _isTransitionPlayingOnBaseLayer = false;
            _activeTransitionMixer = null;

            if (IsRuntimeStateValid(_activeTransitionState))
            {
                _activeTransitionState.Events(this).OnEnd = null;
            }

            _activeTransitionState = null;
            ClearQueuedBaseAnimation();
        }

        private void ClearQueuedBaseAnimation()
        {
            _queuedBaseMixer = null;
            _queuedBaseClip = null;
            _queuedBaseSourceSpeed = 0f;
        }

        private void SynchronizeBaseStateWithTransition()
        {
            if (!IsRuntimeStateValid(_activeTransitionState) || !IsRuntimeStateValid(_currentBaseState))
            {
                return;
            }

            // Adjust speed so the cycles match perfectly
            // 2 animations need to have the similar pose at the same time so that syncing them looks good
            _currentBaseState.Speed = _currentBaseState.Length / _activeTransitionState.Length * _activeTransitionState.Speed;
            _currentBaseState.NormalizedTime = _activeTransitionState.NormalizedTime;
        }

        private void ResetRuntimeStateCache()
        {
            _activeTransitionMixer = null;
            _activeTransitionState = null;
            _currentBaseState = null;
            _isTransitionPlayingOnBaseLayer = false;
            CurrentTag = null;
            ClearQueuedBaseAnimation();
        }

        private void ResetAnimationChannels()
        {
            foreach (AnimationLayerChannel animationChannel in _animationChannels.Values)
            {
                animationChannel.Reset();
            }
        }

        private static bool TrySetMixerParameter(LinearMixerTransition mixer, float sourceSpeed)
        {
            if (mixer == null)
            {
                return false;
            }

            AnimancerState state = mixer.State;

            if (!IsRuntimeStateValid(state))
            {
                return false;
            }

            ((LinearMixerState)state).Parameter = sourceSpeed;
            return true;
        }

        private static bool IsRuntimeStateValid(AnimancerState state)
        {
            return state != null && state.IsValid();
        }

        #endregion
    }
}
