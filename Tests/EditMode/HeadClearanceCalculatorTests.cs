using NUnit.Framework;

namespace PhysicsCharacterController.Tests
{
    /// <summary>
    /// Verifies immediate outward protection, held clearance peaks, bounded correction, and the
    /// deliberately slower inward return used to isolate the camera from animated head motion.
    /// </summary>
    public class HeadClearanceCalculatorTests
    {
        private HeadClearanceCalculator _calculator;

        [TearDown]
        public void TearDown()
        {
            _calculator = null;
        }

        [Test]
        public void UpdateCorrectionMeters_WhenHeadMovesForward_IncreasesImmediately()
        {
            _calculator = CreateCalculator();

            float correctionMeters = _calculator.UpdateCorrectionMeters(0.04f, 0.02f, true, out bool hasExceededMaximumCorrection);

            Assert.That(correctionMeters, Is.EqualTo(0.25f).Within(0.0001f));
            Assert.That(hasExceededMaximumCorrection, Is.False);
        }

        [Test]
        public void UpdateCorrectionMeters_DuringPeakHold_DoesNotMoveBackward()
        {
            _calculator = CreateCalculator();
            float peakMeters = _calculator.UpdateCorrectionMeters(0.04f, 0.02f, true, out _);

            float heldMeters = _calculator.UpdateCorrectionMeters(-0.1f, 0.05f, true, out _);

            Assert.That(heldMeters, Is.EqualTo(peakMeters));
        }

        [Test]
        public void UpdateCorrectionMeters_AfterPeakHold_ReturnsGradually()
        {
            _calculator = CreateCalculator();
            float peakMeters = _calculator.UpdateCorrectionMeters(0.04f, 0.02f, true, out _);
            _calculator.UpdateCorrectionMeters(-0.1f, 0.1f, true, out _);

            float returningMeters = _calculator.UpdateCorrectionMeters(-0.1f, 0.05f, true, out _);

            Assert.That(returningMeters, Is.LessThan(peakMeters));
            Assert.That(returningMeters, Is.GreaterThan(0.11f));
        }

        [Test]
        public void UpdateCorrectionMeters_AboveEnvelope_ClampsAndReportsExceededMaximum()
        {
            _calculator = CreateCalculator();

            float correctionMeters = _calculator.UpdateCorrectionMeters(1f, 0.02f, true, out bool hasExceededMaximumCorrection);

            Assert.That(correctionMeters, Is.EqualTo(0.25f));
            Assert.That(hasExceededMaximumCorrection, Is.True);
        }

        [Test]
        public void UpdateCorrectionMeters_WhileReturnIsLocked_DoesNotFollowGaitBackward()
        {
            _calculator = CreateCalculator();
            float peakMeters = _calculator.UpdateCorrectionMeters(0.04f, 0.02f, true, out _);
            _calculator.UpdateCorrectionMeters(-0.1f, 0.1f, false, out _);

            float lockedMeters = _calculator.UpdateCorrectionMeters(-0.1f, 0.2f, false, out _);

            Assert.That(lockedMeters, Is.EqualTo(peakMeters));
        }

        private static HeadClearanceCalculator CreateCalculator()
        {
            return new HeadClearanceCalculator(headClearanceMeters: 0.18f, safetyMarginMeters: 0.03f, peakHoldSeconds: 0.1f, returnSmoothTimeSeconds: 0.12f, maximumCorrectionMeters: 0.25f);
        }
    }
}
