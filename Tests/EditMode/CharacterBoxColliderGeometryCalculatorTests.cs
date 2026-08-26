using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools.Utils;

namespace PhysicsCharacterController.Tests
{
    public sealed class CharacterBoxColliderGeometryCalculatorTests
    {
        [Test]
        public void Calculate_WithRotationScaleAndOffset_ProducesWorldOverlapBox()
        {
            var calculator = new BoxColliderGeometryCalculator();
            Quaternion worldRotation = Quaternion.Euler(0f, 90f, 0f);

            BoxColliderGeometry geometry = calculator.Calculate(
                worldPosition: new Vector3(10f, 2f, -1f),
                worldRotation,
                lossyScale: new Vector3(2f, 3f, 4f),
                localCenter: new Vector3(1f, 0.5f, -0.25f),
                sizeMeters: new Vector3(2f, 4f, 6f));

            Vector3 expectedCenter = new Vector3(10f, 2f, -1f)
                + worldRotation * new Vector3(2f, 1.5f, -1f);
            Assert.That(geometry.Center, Is.EqualTo(expectedCenter).Using(Vector3ComparerWithEqualsOperator.Instance));
            Assert.That(
                geometry.HalfExtentsMeters,
                Is.EqualTo(new Vector3(2f, 6f, 12f)).Using(Vector3ComparerWithEqualsOperator.Instance));
            Assert.That(Quaternion.Angle(geometry.Rotation, worldRotation), Is.LessThan(0.001f));
        }

        [Test]
        public void Calculate_WithNegativeScale_PreservesMirroredCenterAndPositiveExtents()
        {
            var calculator = new BoxColliderGeometryCalculator();

            BoxColliderGeometry geometry = calculator.Calculate(
                worldPosition: Vector3.zero,
                worldRotation: Quaternion.identity,
                lossyScale: new Vector3(-2f, 3f, -4f),
                localCenter: Vector3.one,
                sizeMeters: Vector3.one * 2f);

            Assert.That(
                geometry.Center,
                Is.EqualTo(new Vector3(-2f, 3f, -4f)).Using(Vector3ComparerWithEqualsOperator.Instance));
            Assert.That(
                geometry.HalfExtentsMeters,
                Is.EqualTo(new Vector3(2f, 3f, 4f)).Using(Vector3ComparerWithEqualsOperator.Instance));
        }
    }
}
