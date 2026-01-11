using UnityEngine;

namespace PhysicsCharacterController
{
    public class VFXManager : MonoBehaviour
    {
        [Header("Particle references")]
        public CharacterManager characterManager;

        [Space(10)]
        public GameObject particleJump;
        public GameObject particleLand;

        [Space(10)]
        public bool enableVFX;

        private CapsuleCollider _collider;

        private void Awake()
        {
            _collider = characterManager.GetComponent<CapsuleCollider>();
        }

        public void ParticleJump()
        {
            if (enableVFX)
            {
                GameObject tmpObj = Instantiate(particleJump,
                    characterManager.transform.position - new Vector3(0f, _collider.height / 2f, 0f),
                    Quaternion.identity);
                tmpObj.transform.parent = transform;
            }
        }

        public void ParticleLand()
        {
            if (enableVFX)
            {
                GameObject tmpObj = Instantiate(particleLand,
                    characterManager.transform.position - new Vector3(0f, _collider.height / 2f, 0f),
                    Quaternion.identity);
                tmpObj.transform.parent = transform;
            }
        }
    }
}