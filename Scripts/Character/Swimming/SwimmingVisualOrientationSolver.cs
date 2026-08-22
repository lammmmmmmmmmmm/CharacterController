using UnityEngine;

namespace PhysicsCharacterController
{
    public sealed class SwimmingVisualOrientationSolver
    {
        private static readonly Quaternion SWIMMING_BODY_AXIS_CORRECTION = Quaternion.Euler(90f, 0f, 0f);

        public Quaternion CalculateTargetLocalRotation(
            Quaternion colliderWorldRotation,
            Quaternion characterRootRotation,
            Quaternion authoredLocalRotation,
            float swimmingAnimationBlend01)
        {
            Quaternion bodyAxisCorrection = Quaternion.Slerp(
                SWIMMING_BODY_AXIS_CORRECTION,
                Quaternion.identity,
                Mathf.Clamp01(swimmingAnimationBlend01));
            Quaternion swimmingWorldRotation = colliderWorldRotation
                * bodyAxisCorrection
                * authoredLocalRotation;
            return Quaternion.Inverse(characterRootRotation) * swimmingWorldRotation;
        }

        public Quaternion CalculateRebasedLocalRotation(
            Quaternion previousCharacterRootRotation,
            Quaternion currentCharacterRootRotation,
            Quaternion currentLocalRotation)
        {
            Quaternion currentWorldRotation = previousCharacterRootRotation * currentLocalRotation;
            return Quaternion.Inverse(currentCharacterRootRotation) * currentWorldRotation;
        }
    }
}
