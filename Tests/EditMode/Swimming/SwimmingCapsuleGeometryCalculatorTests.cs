using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools.Utils;

namespace PhysicsCharacterController.Tests
{
    public sealed class SwimmingCapsuleGeometryCalculatorTests
    {
        private SwimmingCapsuleGeometryCalculator _calculator;

        [SetUp]
        public void SetUp()
        {
            _calculator = new SwimmingCapsuleGeometryCalculator();
        }

        [TearDown]
        public void TearDown()
        {
            _calculator = null;
        }

        [Test]
        public void Calculate_WithForwardCapsule_ProducesHorizontalEndpoints()
        {
            SwimmingCapsuleGeometry geometry = _calculator.Calculate(
                Vector3.zero,
                Quaternion.identity,
                Vector3.one,
                Vector3.zero,
                heightMeters: 2f,
                radiusMeters: 0.45f,
                directionAxis: 2);

            Assert.That(geometry.PointA, Is.EqualTo(Vector3.forward * 0.55f).Using(Vector3ComparerWithEqualsOperator.Instance));
            Assert.That(geometry.PointB, Is.EqualTo(Vector3.back * 0.55f).Using(Vector3ComparerWithEqualsOperator.Instance));
            Assert.That(geometry.RadiusMeters, Is.EqualTo(0.45f).Within(0.0001f));
        }

        [Test]
        public void Calculate_WithRotatedForwardCapsule_RotatesEndpoints()
        {
            Quaternion rotation = Quaternion.FromToRotation(Vector3.forward, Vector3.up);

            SwimmingCapsuleGeometry geometry = _calculator.Calculate(
                Vector3.zero,
                rotation,
                Vector3.one,
                Vector3.zero,
                2f,
                0.45f,
                2);

            Assert.That(geometry.PointA, Is.EqualTo(Vector3.up * 0.55f).Using(Vector3ComparerWithEqualsOperator.Instance));
            Assert.That(geometry.PointB, Is.EqualTo(Vector3.down * 0.55f).Using(Vector3ComparerWithEqualsOperator.Instance));
        }

        [Test]
        public void Calculate_WithPerpendicularScale_UsesLargestRadiusScale()
        {
            SwimmingCapsuleGeometry geometry = _calculator.Calculate(
                Vector3.zero,
                Quaternion.identity,
                new Vector3(2f, 3f, 1f),
                Vector3.zero,
                2f,
                0.45f,
                2);

            Assert.That(geometry.RadiusMeters, Is.EqualTo(1.35f).Within(0.0001f));
            Assert.That(geometry.PointA, Is.EqualTo(Vector3.zero).Using(Vector3ComparerWithEqualsOperator.Instance));
            Assert.That(geometry.PointB, Is.EqualTo(Vector3.zero).Using(Vector3ComparerWithEqualsOperator.Instance));
        }

        [Test]
        public void Calculate_WithInvalidDirectionAxis_FallsBackToVerticalCapsule()
        {
            SwimmingCapsuleGeometry geometry = _calculator.Calculate(
                Vector3.zero,
                Quaternion.identity,
                Vector3.one,
                Vector3.zero,
                2f,
                0.5f,
                directionAxis: 99);

            Assert.That(geometry.PointA, Is.EqualTo(Vector3.up * 0.5f).Using(Vector3ComparerWithEqualsOperator.Instance));
            Assert.That(geometry.PointB, Is.EqualTo(Vector3.down * 0.5f).Using(Vector3ComparerWithEqualsOperator.Instance));
        }
    }
}
