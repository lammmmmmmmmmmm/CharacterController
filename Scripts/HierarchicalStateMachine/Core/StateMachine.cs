using System.Collections.Generic;

namespace HSM
{
    public class StateMachine
    {
        private bool _isStarted;

        public State Root { get; }
        public TransitionSequencer Sequencer { get; }

        public StateMachine(State root)
        {
            Root = root;
            Sequencer = new TransitionSequencer(this);
        }

        public void Start()
        {
            if (_isStarted) return;

            _isStarted = true;
            Root.Enter();
        }

        public void Tick(float deltaTime)
        {
            if (!_isStarted) Start();
            Sequencer.Tick(deltaTime);
        }

        public void FixedTick(float deltaTime)
        {
            if (!_isStarted) Start();
            Sequencer.FixedTick(deltaTime);
        }

        internal void InternalTick(float deltaTime) => Root.Update(deltaTime);
        internal void InternalFixedTick(float deltaTime) => Root.FixedUpdate(deltaTime);

        public void ChangeState(State from, State to)
        {
            if (from == to || from == null || to == null) return;

            State lca = TransitionSequencer.FindLowestCommonAncestor(from, to);
            ExitBranchUpTo(from, lca);
            EnterBranchDownTo(to, lca);
        }

        private static void ExitBranchUpTo(State from, State lca)
        {
            for (State current = from; current != lca; current = current.Parent)
            {
                current.Exit();
            }
        }

        private static void EnterBranchDownTo(State to, State lca)
        {
            var pathFromLca = new Stack<State>();
            for (State current = to; current != lca; current = current.Parent)
            {
                pathFromLca.Push(current);
            }

            while (pathFromLca.Count > 0)
            {
                pathFromLca.Pop().Enter();
            }
        }
    }
}
