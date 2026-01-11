using System.Collections.Generic;
using UnityEngine;


namespace PhysicsCharacterController
{
    [RequireComponent(typeof(Collider))]
    [RequireComponent(typeof(Rigidbody))]
    public class TrampolinePlatform : MonoBehaviour
    {
        [Header("Trampoline properties")]
        [SerializeField] private float _bounceStrength = 20f;


        private List<Rigidbody> _rigidbodies = new List<Rigidbody>();
        private List<float> _velocities = new List<float>();


        /**/


        private void OnCollisionEnter(Collision collision)
        {
            Rigidbody rb = collision.transform.GetComponent<Rigidbody>();
            if (_rigidbodies.Contains(rb))
            {
                //Debug.Log(_velocities[_rigidbodies.IndexOf(rb)]);
                rb.AddForce(_bounceStrength * transform.up * -_velocities[_rigidbodies.IndexOf(rb)], ForceMode.Impulse);
            }
        }


        #region Handle list

        public void Add(Rigidbody rb, float velocityY)
        {
            if (!_rigidbodies.Contains(rb))
            {
                _rigidbodies.Add(rb);
                _velocities.Add(velocityY);
            }
        }


        public void Remove(Rigidbody rb)
        {
            if (_rigidbodies.Contains(rb))
            {
                _rigidbodies.Remove(rb);
                _velocities.Remove(_rigidbodies.IndexOf(rb));
            }
        }

        #endregion
    }
}