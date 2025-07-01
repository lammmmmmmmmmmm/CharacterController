# AI Behavior Rules
- Never assume missing context. Ask questions if uncertain.

# Clean Code
## Coding Convention
- Avoid null comparisons against UnityEngine.Object subclasses
- Writing comments:
	- Only write comments for complex code that is not self-explanatory.
	- Avoid writing comments to explain "what" the code is doing.
	- Write comments answering the question "Why?" (why the code is written this way).
	- Write warning comments.
	- Write comments emphasizing the importance of something that may seem unimportant.
	- The code should be self-explanatory so that comments are not needed.
- Always use variables to store values instead of using magic numbers.
- A function should be longer than three lines; if it's not, don't create a function.
- Avoid the use of strings to access or reference data across systems.
- Classes within the same system can reference each other directly.
- Organize code into clearly separated modules, grouped by feature or responsibility.
- Functions that are only used by a single other function should be defined locally within that function.
- Follow the SOLID principles:
	- Each class or method should do one thing only
	- Make components extendable via inheritance or composition rather than modifying core logic.
	- Use base classes or interfaces that can be safely swapped with derived implementations
	- Break large interfaces into smaller, role-specific ones
	- Depend on abstractions (interfaces) rather than concrete classes. Apply this principle only when there are at least 2 implementations of the abstraction

## Unity
- Use MonoBehaviour for all the classes that need to interact with the game world. Others can be plain C# classes.
- Prefer using events for communication between different systems.
- Use [ScriptableObject event system](../Assets/_Project/Scripts/CommonSystems/SOEventSystem) for decoupling.
- Separate visual logic from game logic:
	- Game logic includes all the code that influences gameplay and are tied to this game's implementation. The game cannot function without game logic.
	- Visual logic includes effects that do not influence gameplay and are tied to this game's implementation such as animations, particle effects, UI transitions, and sound playback, etc. The game can still function without visual logic.
	- Game logic should not depend on visual logic. Instead, visual components should listen for events from the game logic and, in some cases, call it directly when needed.
- Class naming:
	- Class names should be in PascalCase, e.g.
	- Visual logic scripts (tied to this game's implementation):
		- MonoBehaviour visual logic scripts: <FeatureOrObjectName>Visual
		- C# visual logic scripts that are not MonoBehaviour: <FeatureOrObjectName>VisualPlain
		- UI visual logic (a subset of visual logic that specifically handles user interface elements):
			- MonoBehaviour UI logic scripts: <FeatureOrObjectName>UIUpdater
			- C# UI logic scripts that are not MonoBehaviour: <FeatureOrObjectName>UIUpdaterPlain
	- Game logic scripts (tied to this game's implementation):
		- Data scripts that are not ScriptableObjects: <FeatureOrObjectName>Data
		- ScriptableObjects: <FeatureOrObjectName>SO
		- Scene initialization scripts (act like a god class to hold miscellaneous setup functions): <SceneName>GodClass
		- Manager scripts (spawning and managing object lifecycles): <ManagedObjectName>Manager
		- Other game logic scripts:
			- MonoBehaviour logic scripts: <FeatureOrObjectName>GameLogic
			- C# logic scripts that are not MonoBehaviour: <FeatureOrObjectName>GameLogicPlain
	- General purpose scripts (can be used in multiple games): Follow default naming conventions.