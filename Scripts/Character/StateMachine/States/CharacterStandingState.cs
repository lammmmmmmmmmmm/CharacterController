using HSM;

namespace PhysicsCharacterController.CharacterStateMachine.States
{
    public sealed class CharacterStandingState : State
    {
        private readonly CharacterStateContext _context;

        public CharacterStandingState(StateMachine machine, State parent, CharacterStateContext context)
            : base(machine, parent)
        {
            _context = context;
        }

        protected override State GetTransition()
        {
            if (_context.CharacterCrouch.ShouldStayCrouched())
            {
                return ((CharacterGroundedState)Parent).Crouching;
            }

            return null;
        }

        protected override void OnEnter()
        {
            _context.CharacterCrouch.ApplyStandState();
            _context.Animator.PlayLinearMixer(_context.StandingAnimationData.LocomotionMixer);
        }

        protected override void OnFixedUpdate(float fixedDeltaTime)
        {
            _context.StandingAnimationData.LocomotionMixer.State.Parameter = _context.CharacterMove.CurrentSpeed;
        }
    }
}