using UnityEngine;


namespace PhysicsCharacterController
{
    [RequireComponent(typeof(Collider))]
    public class PlatformSensor : MonoBehaviour
    {
        private MovingPlatform _movingPlatform;
        private BoxCollider _boxCollider;


        /**/


        private void Awake()
        {
            _movingPlatform = this.transform.parent.GetComponent<MovingPlatform>();
        }


        #region Collision detection

        private void OnTriggerEnter(Collider other)
        {
            Rigidbody rigidbody = other.GetComponent<Rigidbody>();
            if (rigidbody != null && rigidbody != _movingPlatform.GetComponent<Rigidbody>()) _movingPlatform.Add(rigidbody);
        }


        private void OnTriggerExit(Collider other)
        {
            Rigidbody rigidbody = other.GetComponent<Rigidbody>();
            if (rigidbody != null && rigidbody != _movingPlatform.GetComponent<Rigidbody>()) _movingPlatform.Remove(rigidbody);
        }

        #endregion
    }
}