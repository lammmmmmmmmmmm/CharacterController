using NUnit.Framework;
using UnityEngine;

namespace PhysicsCharacterController.Tests
{
    /// <summary>
    /// Verifies that procedural bob is bounded by speed and locomotion state, excludes passive or
    /// airborne movement, and always returns the camera target to its clean local origin.
    /// </summary>
    public class HeadBobOscillatorTests
    {
        private HeadBobOscillator _oscillator;

        [TearDown]
        public void TearDown()
        {
            _oscillator = null;
        }

        [Test]
        public void UpdateOffsetMeters_WhileIdle_ReturnsZero()
        {
            _oscillator = CreateOscillator();

            Vector3 offsetMeters = _oscillator.UpdateOffsetMeters(0f, 10f, true, false, 0.1f);

            Assert.That(offsetMeters, Is.EqualTo(Vector3.zero));
        }

        [Test]
        public void UpdateOffsetMeters_WhileAirborne_ReturnsZero()
        {
            _oscillator = CreateOscillator();

            Vector3 offsetMeters = _oscillator.UpdateOffsetMeters(10f, 10f, false, true, 0.1f);

            Assert.That(offsetMeters, Is.EqualTo(Vector3.zero));
        }

        [Test]
        public void UpdateOffsetMeters_WithoutMovementInput_IgnoresPassivePlatformVelocity()
        {
            _oscillator = CreateOscillator();

            Vector3 offsetMeters = _oscillator.UpdateOffsetMeters(5f, 10f, true, false, 0.1f);

            Assert.That(offsetMeters, Is.EqualTo(Vector3.zero));
        }

        [Test]
        public void UpdateOffsetMeters_DuringGroundedMovement_ReturnsBoundedOffset()
        {
            _oscillator = CreateOscillator();

            Vector3 offsetMeters = _oscillator.UpdateOffsetMeters(10f, 10f, true, true, 0.1f);

            Assert.That(Mathf.Abs(offsetMeters.x), Is.LessThanOrEqualTo(0.025f));
            Assert.That(Mathf.Abs(offsetMeters.y), Is.LessThanOrEqualTo(0.035f));
            Assert.That(offsetMeters, Is.Not.EqualTo(Vector3.zero));
        }

        [Test]
        public void UpdateOffsetMeters_DuringGroundedMovement_ProducesVisibleHorizontalSway()
        {
            _oscillator = CreateOscillator();

            Vector3 offsetMeters = _oscillator.UpdateOffsetMeters(10f, 10f, true, true, 0.1f);

            Assert.That(Mathf.Abs(offsetMeters.x), Is.GreaterThan(0.02f));
        }

        [Test]
        public void UpdateOffsetMeters_AfterStopping_ReturnsToZero()
        {
            _oscillator = CreateOscillator();
            _oscillator.UpdateOffsetMeters(10f, 10f, true, true, 0.1f);

            Vector3 offsetMeters = _oscillator.UpdateOffsetMeters(0f, 10f, true, false, 0.15f);

            Assert.That(offsetMeters, Is.EqualTo(Vector3.zero));
        }

        [Test]
        public void UpdateOffsetMeters_WithZeroMaximumSpeed_ReturnsZero()
        {
            _oscillator = CreateOscillator();

            Vector3 offsetMeters = _oscillator.UpdateOffsetMeters(10f, 0f, true, true, 0.1f);

            Assert.That(offsetMeters, Is.EqualTo(Vector3.zero));
        }

        private static HeadBobOscillator CreateOscillator()
        {
            return new HeadBobOscillator(minimumMovementSpeedMetersPerSecond: 0.1f, maximumLateralOffsetMeters: 0.025f, maximumVerticalOffsetMeters: 0.035f, minimumFrequencyHertz: 1.5f, maximumFrequencyHertz: 2.4f, activationSeconds: 0.1f, returnSeconds: 0.15f);
        }
    }
}
