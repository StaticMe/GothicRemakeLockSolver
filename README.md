# Gothic Remake Lock Solver

A small unofficial Python tool for solving the lockpicking puzzle in Gothic Remake.

The puzzle has 2 to 6 lips. Each lip has a pin that can sit in one of 7 holes. Moving one lip left or right can also move other lips. The solver searches for a stress-free sequence that moves every pin to the target hole.

The GUI uses normal hole numbers:

```text
holes: 1–7
target: hole 4
```

Internally, the solver uses zero-based indices, but you do not need to enter positions that way.

## Features

* Supports 2 to 6 lips
* Target is always hole 4
* Rejects stress moves automatically
* Finds a shortest solution using BFS
* Shows full, compact, and pattern-compressed solutions
* Includes a manual test mode
* Generates right moves automatically from the entered left moves

## Requirements

Python 3 with Tkinter.
Tkinter is included with most standard Python installations.

## How to use

1. Select the number of lips.
2. Enter the current pin position for each lip using hole numbers 1–7.
3. Enter the left-move vectors.
4. Click `Validate`.
5. Click `Solve`.

Only left-move vectors need to be entered.
The matching right-move vectors are calculated automatically by reversing the signs.

Example:

```text
left:  (1, 0, -1)
right: (-1, 0, 1)
```
The selected lip must always move by `+1` when moved left.

## Manual Mode

If you want to fiddle around yourself, but don't want to waste lockpicks or time ingame, you can do that with the Manual Test Mode.
Just click on the buttons to simulate the corresponding movement of the lockpicking mini game.
Stress inducing moves are denoted by red coloring of the button and are updated accordingly after each move.

## Stress

A move causes stress if any pin would move outside the valid range.

```text
valid holes: 1–7
invalid: below 1 or above 7
```

Stress moves are not used in solutions.

## Output modes

The solver can show the solution in three formats:

* full step-by-step output
* compact output
* pattern-compressed output

Pattern compression is useful when the solution repeats a move sequence, for example:

```text
4x [L5 right, L4 left]
```

## Disclaimer

This is an unofficial fan-made helper tool for Gothic Remake.

It is not affiliated with, endorsed by, or connected to THQ Nordic, Alkimia Interactive, Piranha Bytes, or any other rights holder.

Feel free to change the sourcecode to your likings after downloading. 
