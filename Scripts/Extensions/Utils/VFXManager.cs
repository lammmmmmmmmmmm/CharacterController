using UnityEngine;

namespace PhysicsCharacterController
{
    /// <summary>
    /// Spawns jump and landing particles at the character's feet.
    /// The feet position comes from CharacterColliderShape, so effects remain aligned when the
    /// character uses either a capsule or box collider.
    /// </summary>
    public class VFXManager : MonoBehaviour
    {
        [Header("Particle references")]
        public CharacterManager characterManager;

        [Space(10)]
        public GameObject particleJump;
        public GameObject particleLand;

        [Space(10)]
        public bool enableVFX;

        private CharacterColliderShape _characterColliderShape;

        #region Unity Lifecycle

        private void Awake()
        {
            _characterColliderShape = characterManager.GetComponent<CharacterColliderShape>();
        }

        #endregion

        #region Public Methods

        public void ParticleJump()
        {
            if (enableVFX)
            {
                SpawnAtCharacterFeet(particleJump);
            }
        }

        public void ParticleLand()
        {
            if (enableVFX)
            {
                SpawnAtCharacterFeet(particleLand);
            }
        }

        #endregion

        #region Private Methods

        private void SpawnAtCharacterFeet(GameObject particlePrefab)
        {
            Vector3 feetPosition = characterManager.transform.position
                - Vector3.up * _characterColliderShape.FeetOffsetMeters;
            GameObject particle = Instantiate(particlePrefab, feetPosition, Quaternion.identity);
            particle.transform.parent = transform;
        }

        #endregion
    }
}
