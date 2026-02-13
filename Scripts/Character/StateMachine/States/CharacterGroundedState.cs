using HSM;

namespace PhysicsCharacterController.CharacterStateMachine.States
{
    public sealed class CharacterGroundedState : State
    {
        public readonly CharacterStandingState Standing;
        public readonly CharacterCrouchingState Crouching;

        private readonly CharacterStateContext _context;

        public CharacterGroundedState(StateMachine machine, State parent, CharacterStateContext context)
            : base(machine, parent)
        {
            _context = context;
            Standing = new CharacterStandingState(machine, this, context);
            Crouching = new CharacterCrouchingState(machine, this, context);
        }

        protected override State GetInitialState()
        {
            if (_context.CharacterCrouch.ShouldStayCrouched())
            {
                return Crouching;
            }

            return Standing;
        }

        protected override State GetTransition()
        {
            if (_context.CharacterJump.TryExecuteJump())
            {
                return ((CharacterRootState)Parent).Airborne;
            }

            if (!_context.IsGrounded)
            {
                return ((CharacterRootState)Parent).Airborne;
            }

            return null;
        }
    }
}
