using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools.Utils;

namespace PhysicsCharacterController.Tests
{
    public sealed class CharacterYawSolverTests
    {
        [TestCase(1f)]
        [TestCase(-1f)]
        public void TryResolveHorizontalFacingRotation_HorizontalDirection_FacesThatDirectionUpright(
            float directionX)
        {
            var solver = new CharacterYawSolver();
            Vector3 worldDirection = Vector3.right * directionX;

            bool hasFacingRotation = solver.TryResolveHorizontalFacingRotation(
                worldDirection,
                Quaternion.identity,
                out Quaternion rotation);

            Assert.That(hasFacingRotation, Is.True);
            Assert.That(rotation * Vector3.forward, Is.EqualTo(worldDirection).Using(Vector3ComparerWithEqualsOperator.Instance));
            Assert.That(rotation * Vector3.up, Is.EqualTo(Vector3.up).Using(Vector3ComparerWithEqualsOperator.Instance));
        }

        [Test]
        public void TryResolveHorizontalFacingRotation_VerticalDirection_IgnoresDirection()
        {
            var solver = new CharacterYawSolver();
            Quaternion currentRotation = Quaternion.Euler(0f, 55f, 0f);

            bool hasFacingRotation = solver.TryResolveHorizontalFacingRotation(
                Vector3.down,
                currentRotation,
                out Quaternion rotation);

            Assert.That(hasFacingRotation, Is.False);
            Assert.That(Quaternion.Angle(rotation, currentRotation), Is.LessThan(0.001f));
        }

        [Test]
        public void RotateTowards_ClampsRotationToMaximumTurnDegrees()
        {
            var solver = new CharacterYawSolver();

            Quaternion rotation = solver.RotateTowards(
                Quaternion.identity,
                Quaternion.Euler(0f, 90f, 0f),
                maximumTurnDegrees: 10f);

            Assert.That(rotation.eulerAngles.y, Is.EqualTo(10f).Within(0.001f));
        }
    }
}
