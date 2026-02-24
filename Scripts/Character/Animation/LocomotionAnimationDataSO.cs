using Animancer;
using UnityEngine;

namespace PhysicsCharacterController
{
    [CreateAssetMenu(menuName = "Character Animation/Locomotion Animation Data")]
    public class LocomotionAnimationDataSO : ScriptableObject
    {
        [SerializeField] private LinearMixerTransition _locomotionMixer;

        public LinearMixerTransition LocomotionMixer => _locomotionMixer;
    }
}
