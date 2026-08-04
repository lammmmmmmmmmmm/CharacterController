using NUnit.Framework;
using UnityEngine;

namespace PhysicsCharacterController.Tests
{
    /// <summary>
    /// Verifies that shared ground-normal sampling identifies uneven contacts, retains the result
    /// across a terrain patch, and resets its comparison sample after leaving the ground.
    /// </summary>
    public class GroundSurfaceRoughnessTrackerTests
    {
        private GroundSurfaceRoughnessTracker _tracker;

        [TearDown]
        public void TearDown()
        {
            _tracker = null;
        }

        [Test]
        public void Update_WithUnchangedGroundNormal_ReturnsZeroRoughness()
        {
            _tracker = new GroundSurfaceRoughnessTracker(60f, 0.4f);

            _tracker.Update(true, Vector3.up, 0.02f, 0.02f);
            float roughness01 = _tracker.Update(true, Vector3.up, 0.02f, 0.02f);

            Assert.That(roughness01, Is.EqualTo(0f));
        }

        [Test]
        public void Update_WithAbruptNormalChange_ProducesStrongRoughness()
        {
            _tracker = new GroundSurfaceRoughnessTracker(60f, 0.4f);

            _tracker.Update(true, Vector3.up, 0.02f, 0f);
            float roughness01 = _tracker.Update(true, Vector3.forward, 0.02f, 0f);

            Assert.That(roughness01, Is.EqualTo(1f));
        }

        [Test]
        public void Update_AfterRoughContact_ReleasesGradually()
        {
            _tracker = new GroundSurfaceRoughnessTracker(60f, 0.4f);
            _tracker.Update(true, Vector3.up, 0.02f, 0f);
            _tracker.Update(true, Vector3.forward, 0.02f, 0f);

            float roughness01 = _tracker.Update(true, Vector3.forward, 0.02f, 0.1f);

            Assert.That(roughness01, Is.EqualTo(0.75f).Within(0.0001f));
        }

        [Test]
        public void Update_AfterAirborneFrame_DoesNotCompareAgainstStaleNormal()
        {
            _tracker = new GroundSurfaceRoughnessTracker(60f, 0.4f);
            _tracker.Update(true, Vector3.up, 0.02f, 0f);
            _tracker.Update(false, Vector3.zero, 0.02f, 1f);

            float roughness01 = _tracker.Update(true, Vector3.forward, 0.02f, 0f);

            Assert.That(roughness01, Is.EqualTo(0f));
        }
    }
}
