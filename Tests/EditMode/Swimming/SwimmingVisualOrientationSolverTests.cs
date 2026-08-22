using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools.Utils;

namespace PhysicsCharacterController.Tests
{
    public sealed class SwimmingVisualOrientationSolverTests
    {
        private SwimmingVisualOrientationSolver _solver;

        [SetUp]
        public void SetUp()
        {
            _solver = new SwimmingVisualOrientationSolver();
        }

        [TearDown]
        public void TearDown()
        {
            _solver = null;
        }

        [Test]
        public void CalculateTargetLocalRotation_TreadingPosture_PointsHeadAxisAlongSwimmingDirection()
        {
            Quaternion rotation = _solver.CalculateTargetLocalRotation(
                Quaternion.identity,
                Quaternion.identity,
                Quaternion.identity,
                swimmingAnimationBlend01: 0f);

            Assert.That(rotation * Vector3.up, Is.EqualTo(Vector3.forward).Using(Vector3ComparerWithEqualsOperator.Instance));
            Assert.That(rotation * Vector3.forward, Is.EqualTo(Vector3.down).Using(Vector3ComparerWithEqualsOperator.Instance));
        }

        [Test]
        public void CalculateTargetLocalRotation_TreadingPostureToRight_PointsHeadAxisRight()
        {
            Quaternion colliderRotation = Quaternion.FromToRotation(Vector3.forward, Vector3.right);

            Quaternion rotation = _solver.CalculateTargetLocalRotation(
                colliderRotation,
                Quaternion.identity,
                Quaternion.identity,
                swimmingAnimationBlend01: 0f);

            Assert.That(rotation * Vector3.up, Is.EqualTo(Vector3.right).Using(Vector3ComparerWithEqualsOperator.Instance));
            Assert.That(rotation * Vector3.forward, Is.EqualTo(Vector3.down).Using(Vector3ComparerWithEqualsOperator.Instance));
        }

        [Test]
        public void CalculateTargetLocalRotation_WithRotatedRoot_PreservesAcceptedWorldDirection()
        {
            Quaternion rootRotation = Quaternion.Euler(0f, 70f, 0f);
            Quaternion colliderRotation = Quaternion.FromToRotation(Vector3.forward, Vector3.left);

            Quaternion localRotation = _solver.CalculateTargetLocalRotation(
                colliderRotation,
                rootRotation,
                Quaternion.identity,
                swimmingAnimationBlend01: 0f);

            Vector3 worldHeadDirection = rootRotation * localRotation * Vector3.up;
            Assert.That(worldHeadDirection, Is.EqualTo(Vector3.left).Using(Vector3ComparerWithEqualsOperator.Instance));
        }

        [Test]
        public void CalculateTargetLocalRotation_SwimmingPosture_LeavesAnimatedForwardAxisOnAcceptedDirection()
        {
            Quaternion colliderRotation = Quaternion.FromToRotation(Vector3.forward, Vector3.right);

            Quaternion rotation = _solver.CalculateTargetLocalRotation(
                colliderRotation,
                Quaternion.identity,
                Quaternion.identity,
                swimmingAnimationBlend01: 1f);

            Assert.That(rotation * Vector3.forward, Is.EqualTo(Vector3.right).Using(Vector3ComparerWithEqualsOperator.Instance));
            Assert.That(rotation * Vector3.up, Is.EqualTo(Vector3.up).Using(Vector3ComparerWithEqualsOperator.Instance));
        }

        [Test]
        public void CalculateRebasedLocalRotation_WhenRootYawChanges_PreservesWorldRotation()
        {
            Quaternion previousRootRotation = Quaternion.Euler(0f, 10f, 0f);
            Quaternion currentRootRotation = Quaternion.Euler(0f, 100f, 0f);
            Quaternion currentLocalRotation = Quaternion.Euler(35f, 20f, 15f);
            Quaternion previousWorldRotation = previousRootRotation * currentLocalRotation;

            Quaternion rebasedLocalRotation = _solver.CalculateRebasedLocalRotation(
                previousRootRotation,
                currentRootRotation,
                currentLocalRotation);
            Quaternion currentWorldRotation = currentRootRotation * rebasedLocalRotation;

            Assert.That(Quaternion.Angle(currentWorldRotation, previousWorldRotation), Is.LessThan(0.001f));
        }
    }
}
