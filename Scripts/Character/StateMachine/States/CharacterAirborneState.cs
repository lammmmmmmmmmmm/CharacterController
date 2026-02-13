using HSM;

namespace PhysicsCharacterController.CharacterStateMachine.States
{
    public sealed class CharacterAirborneState : State
    {
        private readonly CharacterStateContext _context;

        public CharacterAirborneState(StateMachine machine, State parent, CharacterStateContext context)
            : base(machine, parent)
        {
            _context = context;
        }

        protected override State GetTransition()
        {
            if (_context.CharacterJump.TryExecuteJump())
            {
                return null;
            }

            if (_context.IsGrounded)
            {
                return ((CharacterRootState)Parent).Grounded;
            }

            return null;
        }
    }
}