using UnityEngine;


namespace PhysicsCharacterController
{
    public class Shooter : MonoBehaviour
    {
        [Header("Shooter specs")]
        public GameObject projectile;
        public float speed = 20f;
        public int timer = 2000;

        private float _originalTimer;

        private void Awake()
        {
            _originalTimer = timer;
        }

        private void Update()
        {
            _originalTimer--;

            if (_originalTimer < 0)
            {
                GameObject instantiatedProjectile = Instantiate(projectile, transform.position, transform.rotation);
                instantiatedProjectile.GetComponent<Rigidbody>().linearVelocity = transform.TransformDirection(new Vector3(0, 0, speed));
                _originalTimer = timer;
            }
        }
    }
}