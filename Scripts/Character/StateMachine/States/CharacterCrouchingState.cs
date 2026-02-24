using HSM;

namespace PhysicsCharacterController.CharacterStateMachine.States
{
    public sealed class CharacterCrouchingState : State
    {
        private readonly CharacterStateContext _context;

        public CharacterCrouchingState(StateMachine machine, State parent, CharacterStateContext context)
            : base(machine, parent)
        {
            _context = context;
        }

        protected override State GetTransition()
        {
            if (_context.CharacterCrouch.ShouldStayCrouched())
            {
                return null;
            }

            return ((CharacterGroundedState)Parent).Standing;
        }

        protected override void OnEnter()
        {
            _context.CharacterCrouch.ApplyCrouchState();
            _context.Animator.PlayLinearMixer(_context.CrouchingAnimationData.LocomotionMixer);
        }

        protected override void OnFixedUpdate(float fixedDeltaTime)
        {
            _context.CrouchingAnimationData.LocomotionMixer.State.Parameter = _context.CharacterMove.CurrentSpeed;
        }
    }
}