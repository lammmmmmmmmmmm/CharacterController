﻿using UnityEngine;


namespace PhysicsCharacterController
{
    public class ClimbArea : MonoBehaviour
    {
        [Header("Area properties")]
        public Vector3 climbSpeed = new Vector3(0f, 1.37f, 0f);
        [Space(10)]

        public InputReader inputReader;


        private Rigidbody player;


        /**/


        private void FixedUpdate()
        {
            if (player != null) player.linearVelocity = new Vector3(player.linearVelocity.x * climbSpeed.x, climbSpeed.y * inputReader.axisInput.y, player.linearVelocity.z * climbSpeed.z);
        }


        #region Collision detection

        private void OnTriggerEnter(Collider other)
        {
            Rigidbody rigidbody = other.GetComponent<Rigidbody>();

            try { if (other.GetComponent<CharacterManager>()) player = rigidbody; }
            catch { /* Debug.Log("Entered something else") */ }
        }


        private void OnTriggerExit(Collider other)
        {
            Rigidbody rigidbody = other.GetComponent<Rigidbody>();

            try { if (other.GetComponent<CharacterManager>()) player = null; }
            catch { /* Debug.Log("Entered something else") */ }
        }

        #endregion
    }
}