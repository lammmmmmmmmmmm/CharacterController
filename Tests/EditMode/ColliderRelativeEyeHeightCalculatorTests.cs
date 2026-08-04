using NUnit.Framework;

namespace PhysicsCharacterController.Tests.EditMode
{
    public sealed class ColliderRelativeEyeHeightCalculatorTests
    {
        [Test]
        public void CalculateEyeHeightMeters_WhenStanding_UsesStandingOffset()
        {
            ColliderRelativeEyeHeightCalculator calculator = CreateCalculator();

            float eyeHeightMeters = calculator.CalculateEyeHeightMeters(1f, 2f);

            Assert.That(eyeHeightMeters, Is.EqualTo(0.56f).Within(0.0001f));
        }

        [Test]
        public void CalculateEyeHeightMeters_WhenCrouched_UsesCrouchedOffset()
        {
            ColliderRelativeEyeHeightCalculator calculator = CreateCalculator();

            float eyeHeightMeters = calculator.CalculateEyeHeightMeters(0f, 1f);

            Assert.That(eyeHeightMeters, Is.EqualTo(0f).Within(0.0001f));
        }

        [Test]
        public void CalculateEyeHeightMeters_DuringCrouch_InterpolatesOffsets()
        {
            ColliderRelativeEyeHeightCalculator calculator = CreateCalculator();

            float eyeHeightMeters = calculator.CalculateEyeHeightMeters(0.5f, 1.5f);

            Assert.That(eyeHeightMeters, Is.EqualTo(0.28f).Within(0.0001f));
        }

        [Test]
        public void CalculateEyeHeightMeters_WithNoColliderHeightRange_UsesStandingOffset()
        {
            ColliderRelativeEyeHeightCalculator calculator = new(2f, 2f, -0.44f, 0f);

            float eyeHeightMeters = calculator.CalculateEyeHeightMeters(1f, 2f);

            Assert.That(eyeHeightMeters, Is.EqualTo(0.56f).Within(0.0001f));
        }

        #region Private Methods

        private static ColliderRelativeEyeHeightCalculator CreateCalculator()
        {
            return new ColliderRelativeEyeHeightCalculator(2f, 1f, -0.44f, 0f);
        }

        #endregion
    }
}
