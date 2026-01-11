using System.Collections.Generic;
using UnityEngine;


namespace PhysicsCharacterController
{
    [RequireComponent(typeof(Collider))]
    public class SpeedArea : MonoBehaviour
    {
        [Header("Area properties")]
        [SerializeField] private float _velocityMultiplier = 1.1f;


        private List<Rigidbody> _rigidbodies = new List<Rigidbody>();


        /**/


        private void FixedUpdate()
        {
            if (_rigidbodies.Count > 0)
            {
                for (int i = 0; i < _rigidbodies.Count; i++)
                {
                    Rigidbody rb = _rigidbodies[i];
                    rb.linearVelocity *= _velocityMultiplier;
                }
            }
        }


        #region Collision detection

        private void OnTriggerEnter(Collider other)
        {
            Rigidbody rigidbody = other.GetComponent<Rigidbody>();
            if (rigidbody != null && !_rigidbodies.Contains(rigidbody)) _rigidbodies.Add(rigidbody);
        }


        private void OnTriggerExit(Collider other)
        {
            Rigidbody rigidbody = other.GetComponent<Rigidbody>();
            if (rigidbody != null && _rigidbodies.Contains(rigidbody)) _rigidbodies.Remove(rigidbody);
        }

        #endregion
    }
}