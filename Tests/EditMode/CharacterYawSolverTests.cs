using NUnit.Framework;
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
    }
}
