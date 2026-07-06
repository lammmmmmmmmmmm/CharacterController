using Animancer;
using UnityEngine;

namespace PhysicsCharacterController
{
    public class CharacterAnimator : MonoBehaviour
    {
        [SerializeField] private AnimancerComponent _animancer;
        [SerializeField] private TransitionLibrary _transitions;

        private AnimancerLayer BaseLayer => _animancer.Layers[0];
        private LinearMixerTransition _activeTransitionMixer;

        private AnimancerState _activeTransitionState;
        private AnimancerState _currentBaseState;

        private LinearMixerTransition _queuedBaseMixer;
        private ClipTransition _queuedBaseClip;
        private float _queuedBaseSourceSpeed;

        private bool _isTransitionPlayingOnBaseLayer;

        public StateId CurrentTag { get; private set; }

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
            _currentBaseState = null;
            CurrentTag = null;
        }

        private void OnDestroy()
        {
            CancelPendingTransitionPlayback();
        }

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

            _queuedBaseSourceSpeed = _activeTransitionMixer.State.Parameter;

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
    }
}