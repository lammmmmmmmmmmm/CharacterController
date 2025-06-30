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
        public GameObject particleFast;

        [Space(10)]
        public bool enableVFX;

        private CapsuleCollider _collider;
        private GameObject _characterModel;

        private void Awake()
        {
            _collider = characterManager.GetComponent<CapsuleCollider>();
            _characterModel = characterManager.characterModel;
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

        public void ParticleFast()
        {
            if (enableVFX)
            {
                GameObject tmpObj = Instantiate(particleFast, characterManager.transform.position,
                    _characterModel.transform.rotation);
                tmpObj.transform.parent = characterManager.transform;
            }
        }
    }
}