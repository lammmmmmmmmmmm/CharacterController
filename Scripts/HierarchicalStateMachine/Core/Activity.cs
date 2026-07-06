using System.Threading;
using System.Threading.Tasks;

namespace HSM
{
    public enum ActivityMode { Inactive, Activating, Active, Deactivating }

    public interface IActivity
    {
        public ActivityMode Mode { get; }
        public Task ActivateAsync(CancellationToken ct);
        public Task DeactivateAsync(CancellationToken ct);
    }

    public abstract class Activity : IActivity
    {
        public ActivityMode Mode { get; protected set; } = ActivityMode.Inactive;

        public virtual Task ActivateAsync(CancellationToken ct)
        {
            if (Mode != ActivityMode.Inactive) return Task.CompletedTask;

            Mode = ActivityMode.Activating;
            Mode = ActivityMode.Active;
            return Task.CompletedTask;
        }

        public virtual Task DeactivateAsync(CancellationToken ct)
        {
            if (Mode != ActivityMode.Active) return Task.CompletedTask;

            Mode = ActivityMode.Deactivating;
            Mode = ActivityMode.Inactive;
            return Task.CompletedTask;
        }
    }
}