using System.Collections;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

namespace PhysicsCharacterController.Tests.PlayMode
{
    public sealed class CharacterRotationTests
    {
        private GameObject _characterGameObject;
        private Rigidbody _characterRigidbody;
        private CharacterRotation _characterRotation;

        #region Unity Lifecycle

        [SetUp]
        public void SetUp()
        {
            _characterGameObject = new GameObject("Character Rotation Test Character");
            _characterRigidbody = _characterGameObject.AddComponent<Rigidbody>();
            _characterRigidbody.isKinematic = true;
            _characterRotation = _characterGameObject.AddComponent<CharacterRotation>();
        }

        [TearDown]
        public void TearDown()
        {
            Object.Destroy(_characterGameObject);
        }

        #endregion

        #region Public Methods

        [Test]
        public void SetFacingDirectionImmediately_RotatesCharacterUpright()
        {
            _characterRotation.SetFacingDirectionImmediately(Vector3.right);

            Assert.That(Vector3.Dot(_characterRigidbody.rotation * Vector3.forward, Vector3.right), Is.GreaterThan(0.999f));
            Assert.That(Vector3.Dot(_characterRigidbody.rotation * Vector3.up, Vector3.up), Is.GreaterThan(0.999f));
        }

        [UnityTest]
        public IEnumerator RotateTowardsDirection_AppliesCentralTurnSpeed()
        {
            _characterRotation.RotateTowardsDirection(Vector3.right, deltaSeconds: 0.1f);

            yield return new WaitForFixedUpdate();

            Quaternion expectedRotation = Quaternion.Euler(0f, 54f, 0f);
            Assert.That(Quaternion.Angle(_characterRigidbody.rotation, expectedRotation), Is.LessThan(0.1f));
        }

        #endregion
    }
}
