# ValhallaLib
[![Java CI with Gradle](https://github.com/frc3268/ValhallaLib/actions/workflows/gradle.yml/badge.svg)](https://github.com/frc3268/ValhallaLib/actions/workflows/gradle.yml)

*repo for ValhallaLib, FRC team 3268's robot code library*

**Features** swerve-drive code, camera code, autonomous code, and dashboard code

## Development
### Naming
This library abides (or tries to abide) to the Kotlin conventions, as seen below:
- Subsystems: End with "Subsystem"
- Commands: End with "Command"
- Constants: UPPERCASE_WITH_UNDERSCORES
- Methods/Functions: startWithLowerCaseThenCamelCase
- Classes: StartWithUpperCaseThenPascalCase

### Structure
- `/src/main/kotlin/frc/lib` - All the library code (non wpilib code for example) go here
- `/src/main/kotlin/frc/robot` - Everything else goes here

### Swerve
The swerve module documentation can be found [here](docs/swervemodule.md).
You can also find the swerve database documentation [here](docs/swervedatabase.md).
### Contributions
Contributions are (very much) welcome! Generally:
1. Keep it simple
2. Don't appeal to boilerplate if you don't have to
3. Allow extensibility

### Usage
1. Clone/fork the repository
2. Change the constants to match your robot
3. Enjoy!

**Thanks** to FRC team 6814 for creating an excellent series on FRC programming (0 to Autonomous). Their code in episode 6 served as the basis for much of the swerve-drive code in this library
