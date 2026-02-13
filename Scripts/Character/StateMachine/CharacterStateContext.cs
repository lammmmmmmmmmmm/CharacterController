namespace PhysicsCharacterController.CharacterStateMachine
{
    public sealed class CharacterStateContext
    {
        public CharacterMove CharacterMove { get; }
        public CharacterCrouch CharacterCrouch { get; }
        public CharacterJump CharacterJump { get; }
        public CharacterGravity CharacterGravity { get; }
        public GroundChecker GroundChecker { get; }

        public CharacterStateContext(
            CharacterMove characterMove,
            CharacterCrouch characterCrouch,
            CharacterJump characterJump,
            CharacterGravity characterGravity,
            GroundChecker groundChecker)
        {
            CharacterMove = characterMove;
            CharacterCrouch = characterCrouch;
            CharacterJump = characterJump;
            CharacterGravity = characterGravity;
            GroundChecker = groundChecker;
        }

        public bool IsGrounded => GroundChecker.IsGrounded;
    }
}