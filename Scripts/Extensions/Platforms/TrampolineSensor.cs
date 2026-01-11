using UnityEngine;


namespace PhysicsCharacterController
{
    [RequireComponent(typeof(BoxCollider))]
    public class TrampolineSensor : MonoBehaviour
    {
        private TrampolinePlatform _trampolinePlatform;


        /**/


        private void Awake()
        {
            _trampolinePlatform = this.transform.parent.GetComponent<TrampolinePlatform>();
        }


        #region Collision detection

        private void OnTriggerEnter(Collider other)
        {
            Rigidbody rigidbody = other.GetComponent<Rigidbody>();
            if (rigidbody != null && rigidbody != _trampolinePlatform.GetComponent<Rigidbody>()) _trampolinePlatform.Add(rigidbody, rigidbody.linearVelocity.y);
        }


        private void OnTriggerExit(Collider other)
        {
            Rigidbody rigidbody = other.GetComponent<Rigidbody>();
            if (rigidbody != null && rigidbody != _trampolinePlatform.GetComponent<Rigidbody>()) _trampolinePlatform.Remove(rigidbody);
        }

        #endregion
    }
}