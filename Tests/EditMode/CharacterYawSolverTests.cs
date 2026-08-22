using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools.Utils;
namespace PhysicsCharacterController.Tests
{
    public sealed class CharacterYawSolverTests
    {
        [Test]
        public void ResolveTargetYawDegrees_DefaultMode_UsesMovementYaw()
        {
            var solver = new CharacterYawSolver();

            Assert.That(solver.ResolveTargetYawDegrees(25f, 80f, false, false), Is.EqualTo(25f));
        }

        [TestCase(true, false)]
        [TestCase(false, true)]
        [TestCase(true, true)]
        public void ResolveTargetYawDegrees_CameraMode_UsesCameraYaw(bool isLockedToCamera, bool isCameraFacingOverrideEnabled)
        {
            var solver = new CharacterYawSolver();

            Assert.That(solver.ResolveTargetYawDegrees(25f, 80f, isLockedToCamera, isCameraFacingOverrideEnabled), Is.EqualTo(80f));
        }

        [TestCase(1f)]
        [TestCase(-1f)]
        public void ResolveHorizontalFacingRotation_HorizontalSwimmingDirection_FacesThatDirectionUpright(
            float directionX)
        {
            var solver = new CharacterYawSolver();
            Vector3 worldDirection = Vector3.right * directionX;

            Quaternion rotation = solver.ResolveHorizontalFacingRotation(worldDirection, Quaternion.identity);

            Assert.That(rotation * Vector3.forward, Is.EqualTo(worldDirection).Using(Vector3ComparerWithEqualsOperator.Instance));
            Assert.That(rotation * Vector3.up, Is.EqualTo(Vector3.up).Using(Vector3ComparerWithEqualsOperator.Instance));
        }

        [Test]
        public void ResolveHorizontalFacingRotation_VerticalSwimmingDirection_PreservesCurrentYaw()
        {
            var solver = new CharacterYawSolver();
            Quaternion currentRotation = Quaternion.Euler(0f, 55f, 0f);

            Quaternion rotation = solver.ResolveHorizontalFacingRotation(Vector3.down, currentRotation);

            Assert.That(Quaternion.Angle(rotation, currentRotation), Is.LessThan(0.001f));
        }

        [Test]
        public void ResetSmoothingVelocity_AfterSwimming_DiscardsPreSwimmingTurnMomentum()
        {
            var solver = new CharacterYawSolver();
            solver.SmoothYawDegrees(0f, 90f, 0.1f, 0.02f);

            solver.ResetSmoothingVelocity();
            float yawDegrees = solver.SmoothYawDegrees(0f, 0f, 0.1f, 0.02f);

            Assert.That(yawDegrees, Is.EqualTo(0f).Within(0.0001f));
        }
    }
}
