using System.Threading;
using System.Threading.Tasks;

namespace HSM
{
    public interface IPhase
    {
        public bool IsDone { get; }
        public void Start();
        public bool Update();
    }

    public delegate Task PhaseStep(CancellationToken ct);

    public class NoopPhase : IPhase
    {
        public bool IsDone { get; private set; }
        public void Start() => IsDone = true;
        public bool Update() => IsDone;
    }
}