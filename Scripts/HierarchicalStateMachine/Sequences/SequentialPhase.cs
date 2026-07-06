using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;

namespace HSM
{
    public class SequentialPhase : IPhase
    {
        readonly List<PhaseStep> _steps;
        readonly CancellationToken _cancellationToken;
        int _currentIndex = -1;
        Task _currentTask;

        public bool IsDone { get; private set; }

        public SequentialPhase(List<PhaseStep> steps, CancellationToken cancellationToken)
        {
            _steps = steps;
            _cancellationToken = cancellationToken;
        }

        public void Start() => AdvanceToNext();

        public bool Update()
        {
            if (IsDone) return true;
            if (_currentTask == null || _currentTask.IsCompleted) AdvanceToNext();
            return IsDone;
        }

        void AdvanceToNext()
        {
            _currentIndex++;
            if (_currentIndex >= _steps.Count)
            {
                IsDone = true;
                return;
            }

            _currentTask = _steps[_currentIndex](_cancellationToken);
        }
    }
}