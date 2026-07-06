using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;

namespace HSM
{
    public class ParallelPhase : IPhase
    {
        readonly List<PhaseStep> _steps;
        readonly CancellationToken _cancellationToken;
        List<Task> _runningTasks;

        public bool IsDone { get; private set; }

        public ParallelPhase(List<PhaseStep> steps, CancellationToken cancellationToken)
        {
            _steps = steps;
            _cancellationToken = cancellationToken;
        }

        public void Start()
        {
            if (_steps == null || _steps.Count == 0)
            {
                IsDone = true;
                return;
            }

            _runningTasks = new List<Task>(_steps.Count);
            for (int i = 0; i < _steps.Count; i++)
                _runningTasks.Add(_steps[i](_cancellationToken));
        }

        public bool Update()
        {
            if (IsDone) return true;

            IsDone = _runningTasks == null || _runningTasks.TrueForAll(t => t.IsCompleted);
            return IsDone;
        }
    }
}