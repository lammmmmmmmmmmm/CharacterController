using HSM;
using PhysicsCharacterController.CharacterStateMachine;

namespace PhysicsCharacterController.CharacterStateMachine.States
{
    public sealed class CharacterRootState : State
    {
        public readonly CharacterGroundedState Grounded;
        public readonly CharacterAirborneState Airborne;

        private readonly CharacterStateContext _context;

        public CharacterRootState(StateMachine machine, CharacterStateContext context)
            : base(machine, null)
        {
            _context = context;
            Grounded = new CharacterGroundedState(machine, this, context);
            Airborne = new CharacterAirborneState(machine, this, context);
        }

        protected override State GetInitialState()
        {
            if (_context.IsGrounded)
            {
                return Grounded;
            }

            return Airborne;
        }

        protected override State GetTransition()
        {
            if (_context.IsGrounded)
            {
                return null;
            }

            if (ReferenceEquals(ActiveChild, Grounded))
            {
                return Airborne;
            }

            return null;
        }

        protected override void OnFixedUpdate(float fixedDeltaTime)
        {
            _context.CharacterGravity.ApplyGravity();
            ApplyMovementByInput();

            _context.CharacterJump.HandleCoyoteTime(fixedDeltaTime);
            _context.CharacterJump.HandleJumpBuffer(fixedDeltaTime);
        }

        public void ApplyMovementByInput()
        {
            if (_context.CharacterMove.HasMovementInput)
            {
                _context.CharacterMove.MoveWithInput();
                return;
            }

            _context.CharacterMove.Decelerate();
        }
    }
}