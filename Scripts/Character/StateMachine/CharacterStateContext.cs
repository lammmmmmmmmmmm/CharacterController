namespace PhysicsCharacterController.CharacterStateMachine
{
    public sealed class CharacterStateContext
    {
        public CharacterMove CharacterMove { get; }
        public CharacterCrouch CharacterCrouch { get; }
        public CharacterJump CharacterJump { get; }
        public CharacterGravity CharacterGravity { get; }
        public GroundChecker GroundChecker { get; }
        public CharacterAnimator Animator { get; }
        public LocomotionAnimationDataSO StandingAnimationData { get; }
        public LocomotionAnimationDataSO CrouchingAnimationData { get; }
        public AirborneAnimationDataSO AirborneAnimationData { get; }

        public CharacterStateContext(
            CharacterMove characterMove,
            CharacterCrouch characterCrouch,
            CharacterJump characterJump,
            CharacterGravity characterGravity,
            GroundChecker groundChecker,
            CharacterAnimator animator,
            LocomotionAnimationDataSO standingAnimationData,
            LocomotionAnimationDataSO crouchingAnimationData,
            AirborneAnimationDataSO airborneAnimationData)
        {
            CharacterMove = characterMove;
            CharacterCrouch = characterCrouch;
            CharacterJump = characterJump;
            CharacterGravity = characterGravity;
            GroundChecker = groundChecker;
            Animator = animator;
            StandingAnimationData = standingAnimationData;
            CrouchingAnimationData = crouchingAnimationData;
            AirborneAnimationData = airborneAnimationData;
        }

        public bool IsGrounded => GroundChecker.IsGrounded;
    }
}