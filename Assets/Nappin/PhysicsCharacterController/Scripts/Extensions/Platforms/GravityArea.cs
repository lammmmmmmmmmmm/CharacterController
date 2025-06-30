using System.Collections.Generic;
using UnityEngine;


namespace PhysicsCharacterController
{
    [RequireComponent(typeof(Collider))]
    public class GravityArea : MonoBehaviour
    {
        [Header("Area properties")]
        public Vector3 gravityForce = new(0f, 1.37f, 0f);

        private readonly List<Rigidbody> _rigidbodies = new();

        private void FixedUpdate()
        {
            if (_rigidbodies.Count > 0)
            {
                foreach (var rb in _rigidbodies)
                {
                    rb.linearVelocity = new Vector3(rb.linearVelocity.x * gravityForce.x, gravityForce.y, rb.linearVelocity.z * gravityForce.z);
                }
            }
        }

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

    }
}