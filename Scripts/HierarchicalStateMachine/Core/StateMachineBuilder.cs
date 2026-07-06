using System.Collections.Generic;
using System.Reflection;

namespace HSM
{
    public class StateMachineBuilder
    {
        private static readonly BindingFlags FIELD_BINDING_FLAGS =
            BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.FlattenHierarchy;

        private readonly State _root;

        public StateMachineBuilder(State root)
        {
            _root = root;
        }

        public StateMachine Build()
        {
            var machine = new StateMachine(_root);
            WireHierarchy(_root, machine, new HashSet<State>());
            return machine;
        }

        private void WireHierarchy(State state, StateMachine machine, HashSet<State> visited)
        {
            if (state == null || !visited.Add(state)) return;

            state.Machine = machine;

            foreach (var field in state.GetType().GetFields(FIELD_BINDING_FLAGS))
            {
                if (!typeof(State).IsAssignableFrom(field.FieldType)) continue;
                if (field.Name == nameof(State.Parent)) continue;

                var child = (State)field.GetValue(state);
                if (child == null) continue;
                if (!ReferenceEquals(child.Parent, state)) continue;

                WireHierarchy(child, machine, visited);
            }
        }
    }
}