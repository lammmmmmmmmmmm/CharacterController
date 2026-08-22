using UnityEngine;

namespace PhysicsCharacterController
{
    [DisallowMultipleComponent]
    public sealed class CharacterSwimmingVisualOrientation : MonoBehaviour
    {
        [Header("Visual Root")]
        [SerializeField] private Transform _meshTransform;

        [Header("Rotation Smoothing")]
        [SerializeField, Min(0f)] private float _swimmingRotationSharpness = 8f;
        [SerializeField, Min(0f)] private float _uprightRotationSharpness = 6f;

        private readonly SwimmingVisualOrientationSolver _orientationSolver = new();
        private Quaternion _authoredLocalRotation;

        #region Unity Lifecycle

        private void Awake()
        {
            _authoredLocalRotation = _meshTransform.localRotation;
        }

        #endregion

        #region Public Methods

        public void AlignToColliderRotation(
            Quaternion colliderWorldRotation,
            Quaternion characterRootRotation,
            float swimmingAnimationBlend01,
            float fixedDeltaTime)
        {
            Quaternion targetLocalRotation = _orientationSolver.CalculateTargetLocalRotation(
                colliderWorldRotation,
                characterRootRotation,
                _authoredLocalRotation,
                swimmingAnimationBlend01);

            _meshTransform.localRotation = DampRotation(
                _meshTransform.localRotation,
                targetLocalRotation,
                _swimmingRotationSharpness,
                fixedDeltaTime);
        }

        public void ReturnToUpright(float fixedDeltaTime)
        {
            _meshTransform.localRotation = DampRotation(
                _meshTransform.localRotation,
                _authoredLocalRotation,
                _uprightRotationSharpness,
                fixedDeltaTime);
        }

        public void PreserveWorldRotationAfterRootRotation(Quaternion previousCharacterRootRotation, Quaternion currentCharacterRootRotation)
        {
            _meshTransform.localRotation = _orientationSolver.CalculateRebasedLocalRotation(
                previousCharacterRootRotation,
                currentCharacterRootRotation,
                _meshTransform.localRotation);
        }

        public void ResetImmediately()
        {
            _meshTransform.localRotation = _authoredLocalRotation;
        }

        private static Quaternion DampRotation(Quaternion current, Quaternion target, float sharpness, float deltaTime)
        {
            float t = 1f - Mathf.Exp(-sharpness * deltaTime);
            return Quaternion.Slerp(current, target, t);
        }

        #endregion
    }
}
