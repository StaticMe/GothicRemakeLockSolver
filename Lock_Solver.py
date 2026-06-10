from __future__ import annotations

import argparse
import tkinter as tk
from tkinter import messagebox
from collections import deque
from typing import Any, Dict, List, Optional, Tuple

# Internal model:
# - The solver uses zero-based pin indices: 0..6.
# - The GUI displays human-readable hole numbers: 1..7.
# - The target is displayed as hole 4 and stored internally as index 3.

MIN_INDEX = 0
MAX_INDEX = 6
MIN_HOLE = 1
MAX_HOLE = 7
MIN_LIPS = 2
MAX_LIPS = 6
TARGET_INDEX = 3
TARGET_HOLE = TARGET_INDEX + 1

State = Tuple[int, ...]
Vector = Tuple[int, ...]
MoveTable = Dict[str, Vector]
Path = List[Tuple[str, State]]
CompressedPath = List[Tuple[int, str, State]]

# ============================================================
# DEFAULT LOCK CONFIGURATION
# ============================================================

# Stored internally as zero-based indices.
# Displayed in the GUI as: (3, 1, 7, 1, 2, 2)
DEFAULT_START: State = (2, 0, 6, 0, 1, 1)

# Enter only LEFT movement vectors.
# RIGHT movement vectors are generated automatically as the negative vectors.
DEFAULT_LEFT_MOVES: Dict[int, Vector] = {
    1: (1, 0, 0, -1, -1, -1),
    2: (-1, 1, -1, 0, 0, 0),
    3: (0, 0, 1, -1, 1, 0),
    4: (0, 0, 1, 1, 0, 0),
    5: (0, -1, 1, 0, 1, 0),
    6: (0, 0, 0, 0, 0, 1),
}


# ============================================================
# DISPLAY CONVERSION
# ============================================================

def index_to_hole(index_value: int) -> int:
    """Convert an internal zero-based index to a displayed hole number."""
    return index_value + 1


def hole_to_index(hole_value: int) -> int:
    """Convert a displayed hole number to an internal zero-based index."""
    return hole_value - 1


def state_to_holes(state: State) -> Tuple[int, ...]:
    """Convert an internal state tuple to displayed hole numbers."""
    return tuple(index_to_hole(value) for value in state)


def holes_to_state(holes: Tuple[int, ...]) -> State:
    """Convert displayed hole numbers to an internal state tuple."""
    return tuple(hole_to_index(value) for value in holes)


def format_state(state: State) -> str:
    """Format an internal state as displayed hole numbers."""
    return str(state_to_holes(state))


# ============================================================
# SOLVER CORE
# ============================================================

def make_goal(start: State) -> State:
    """The target is always internal index 3 for every lip."""
    return tuple(TARGET_INDEX for _ in start)


def negate_vector(vector: Vector) -> Vector:
    """Create the right movement vector from the left movement vector."""
    return tuple(-value for value in vector)


def apply_move(state: State, move_vector: Vector) -> State:
    """Apply one movement vector to one state."""
    return tuple(pin + delta for pin, delta in zip(state, move_vector))


def is_stress_free(state: State) -> bool:
    """A state is stress-free when all internal pin indices stay within 0..6."""
    return all(MIN_INDEX <= pin <= MAX_INDEX for pin in state)


def build_moves(left_moves: Dict[int, Vector]) -> MoveTable:
    """Build left and right moves from the left movement vectors."""
    moves: MoveTable = {}

    for lip in sorted(left_moves):
        left_vector = left_moves[lip]
        right_vector = negate_vector(left_vector)

        moves[f"L{lip} left"] = left_vector
        moves[f"L{lip} right"] = right_vector

    return moves


def validate_config(start: State, left_moves: Dict[int, Vector]) -> None:
    """Validate a lock configuration and fail early on common input errors."""
    lip_count = len(start)

    if not (MIN_LIPS <= lip_count <= MAX_LIPS):
        raise ValueError(
            f"Invalid number of lips: {lip_count}. "
            f"Allowed range is {MIN_LIPS} to {MAX_LIPS}."
        )

    expected_lips = set(range(1, lip_count + 1))
    actual_lips = set(left_moves.keys())

    if actual_lips != expected_lips:
        raise ValueError(
            f"LEFT_MOVES must contain exactly lips {sorted(expected_lips)}. "
            f"Currently present: {sorted(actual_lips)}."
        )

    for lip_index, value in enumerate(start, start=1):
        if not (MIN_INDEX <= value <= MAX_INDEX):
            raise ValueError(
                f"START: L{lip_index} has invalid internal index {value}. "
                f"Allowed internal range is {MIN_INDEX} to {MAX_INDEX}."
            )

    for lip, vector in left_moves.items():
        if len(vector) != lip_count:
            raise ValueError(
                f"L{lip} left has vector length {len(vector)}, "
                f"but expected length is {lip_count}."
            )

        selected_lip_delta = vector[lip - 1]
        if selected_lip_delta != 1:
            raise ValueError(
                f"L{lip} left is inconsistent: "
                f"the selected lip must have delta +1, "
                f"but it has {selected_lip_delta}."
            )


def find_stress_moves(state: State, moves: MoveTable) -> List[Tuple[str, State, List[str]]]:
    """Return all moves that would cause stress from the given state."""
    result: List[Tuple[str, State, List[str]]] = []

    for move_name, move_vector in moves.items():
        next_state = apply_move(state, move_vector)

        bad_positions = [
            f"L{index + 1}=hole {index_to_hole(value)}"
            for index, value in enumerate(next_state)
            if value < MIN_INDEX or value > MAX_INDEX
        ]

        if bad_positions:
            result.append((move_name, next_state, bad_positions))

    return result


def reconstruct_path(
    parent: Dict[State, Tuple[Optional[State], Optional[str]]],
    goal: State,
) -> Path:
    """Reconstruct the move path after BFS reaches the goal."""
    path: Path = []
    current = goal

    while parent[current][0] is not None:
        previous_state, move_name = parent[current]
        if move_name is None:
            raise RuntimeError("Invalid parent path: missing move name.")
        path.append((move_name, current))
        current = previous_state  # type: ignore[assignment]

    path.reverse()
    return path


def solve_bfs(start: State, goal: State, moves: MoveTable) -> Tuple[Optional[Path], int, int]:
    """
    Find a shortest stress-free solution.

    Returns:
    - path or None
    - checked_states: states actually popped from the queue
    - discovered_states: states discovered before termination
    """
    queue = deque([start])

    parent: Dict[State, Tuple[Optional[State], Optional[str]]] = {
        start: (None, None)
    }

    checked_states = 0

    while queue:
        current = queue.popleft()
        checked_states += 1

        if current == goal:
            path = reconstruct_path(parent, goal)
            return path, checked_states, len(parent)

        for move_name, move_vector in moves.items():
            next_state = apply_move(current, move_vector)

            # Reject stress moves immediately.
            if not is_stress_free(next_state):
                continue

            # Do not revisit states that have already been discovered.
            if next_state in parent:
                continue

            parent[next_state] = (current, move_name)
            queue.append(next_state)

    return None, checked_states, len(parent)


# ============================================================
# OUTPUT FORMATTING
# ============================================================

def compress_path(path: Path) -> CompressedPath:
    """Compress directly repeated identical moves."""
    if not path:
        return []

    compressed: CompressedPath = []
    current_move = path[0][0]
    count = 1
    last_state = path[0][1]

    for move_name, state in path[1:]:
        if move_name == current_move:
            count += 1
        else:
            compressed.append((count, current_move, last_state))
            current_move = move_name
            count = 1

        last_state = state

    compressed.append((count, current_move, last_state))
    return compressed


def group_signature(group: Tuple[int, str, State]) -> Tuple[int, str]:
    """Return a group signature while intentionally ignoring the state."""
    count, move_name, _state = group
    return count, move_name


def compress_repeated_blocks(groups: CompressedPath) -> List[Dict[str, Any]]:
    """Compress repeated adjacent blocks, for example ABABAB -> 3x [A, B]."""
    result: List[Dict[str, Any]] = []
    i = 0
    n = len(groups)

    while i < n:
        best_block_len = 0
        best_repeat_count = 0
        max_block_len = (n - i) // 2

        # Try short blocks first so ABABAB becomes 3x [A, B].
        for block_len in range(2, max_block_len + 1):
            base_block = [
                group_signature(group)
                for group in groups[i:i + block_len]
            ]

            repeat_count = 1
            j = i + block_len

            while j + block_len <= n:
                next_block = [
                    group_signature(group)
                    for group in groups[j:j + block_len]
                ]

                if next_block == base_block:
                    repeat_count += 1
                    j += block_len
                else:
                    break

            if repeat_count >= 2:
                best_block_len = block_len
                best_repeat_count = repeat_count
                break

        if best_repeat_count >= 2:
            block_groups = groups[i:i + best_block_len]
            end_index = i + best_block_len * best_repeat_count - 1
            end_state = groups[end_index][2]

            result.append({
                "type": "block",
                "repeat_count": best_repeat_count,
                "block": [
                    (count, move_name)
                    for count, move_name, _state in block_groups
                ],
                "state": end_state,
            })

            i += best_block_len * best_repeat_count
        else:
            count, move_name, state = groups[i]

            result.append({
                "type": "single",
                "count": count,
                "move_name": move_name,
                "state": state,
            })

            i += 1

    return result


def format_move_phrase(count: int, move_name: str) -> str:
    """Format one move phrase, hiding the count when it is one."""
    if count == 1:
        return move_name
    return f"{count}x {move_name}"


def format_block(block: List[Tuple[int, str]]) -> str:
    """Format one repeated move block."""
    return ", ".join(
        format_move_phrase(count, move_name)
        for count, move_name in block
    )


def format_stress_moves(state: State, moves: MoveTable) -> str:
    """Format all stress-causing moves from a given state."""
    stress_moves = find_stress_moves(state, moves)
    lines = [f"Stress check for state: {format_state(state)}"]

    if not stress_moves:
        lines.append("No stress-causing moves.")
        return "\n".join(lines)

    for move_name, next_state, bad_positions in stress_moves:
        lines.append(
            f"{move_name:<10} -> {format_state(next_state)} "
            f"Stress at {', '.join(bad_positions)}"
        )

    return "\n".join(lines)


def format_solution(
    start: State,
    goal: State,
    path: Optional[Path],
    checked_states: int,
    discovered_states: int,
    show_full: bool = True,
    show_compact: bool = True,
    show_pattern: bool = True,
) -> str:
    """Format the solution using displayed hole numbers."""
    lines = [
        f"Start: {format_state(start)}",
        f"Target: {format_state(goal)}",
        f"Checked states:    {checked_states}",
        f"Discovered states: {discovered_states}",
        "",
    ]

    if path is None:
        lines.append("No stress-free solution found.")
        return "\n".join(lines)

    lines.append(f"Solution found in {len(path)} individual moves.")
    lines.append("")

    if show_full:
        lines.append("Individual steps:")
        for step, (move_name, state) in enumerate(path, start=1):
            lines.append(f"{step:02d}. {move_name:<10} -> {format_state(state)}")
        lines.append("")

    if show_compact:
        compressed = compress_path(path)
        lines.append(f"Compact output in {len(compressed)} groups:")
        for step, (count, move_name, state) in enumerate(compressed, start=1):
            phrase = format_move_phrase(count, move_name)
            lines.append(f"{step:02d}. {phrase:<15} -> {format_state(state)}")
        lines.append("")

    if show_pattern:
        groups = compress_path(path)
        pattern_groups = compress_repeated_blocks(groups)
        lines.append(f"Pattern-compact output in {len(pattern_groups)} groups:")

        for step, entry in enumerate(pattern_groups, start=1):
            if entry["type"] == "single":
                phrase = format_move_phrase(entry["count"], entry["move_name"])
                lines.append(f"{step:02d}. {phrase:<30} -> {format_state(entry['state'])}")
            elif entry["type"] == "block":
                block_text = format_block(entry["block"])
                lines.append(f"{step:02d}. {entry['repeat_count']}x [{block_text}] -> {format_state(entry['state'])}")

        lines.append("")

    return "\n".join(lines)


# ============================================================
# GUI
# ============================================================

class LockSolverApp(tk.Tk):
    """Tkinter GUI for entering and solving lock puzzles."""

    def __init__(self) -> None:
        super().__init__()

        self.title("Lock Solver")
        self.geometry("1180x820")
        self.minsize(980, 680)

        self.lip_count_var = tk.IntVar(value=len(DEFAULT_START))
        self.show_full_var = tk.BooleanVar(value=True)
        self.show_compact_var = tk.BooleanVar(value=True)
        self.show_pattern_var = tk.BooleanVar(value=True)

        # GUI start variables store displayed hole numbers, not internal indices.
        self.start_vars: List[tk.IntVar] = []
        self.move_vars: List[List[tk.IntVar]] = []
        self.manual_state: State = DEFAULT_START
        self.manual_state_vars: List[tk.StringVar] = []
        self.manual_buttons: Dict[str, tk.Button] = {}
        self.last_solution_text = ""

        self._build_static_layout()
        self.load_example()

    # ---------------------------
    # Layout
    # ---------------------------

    def _build_static_layout(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(3, weight=1)

        top_frame = tk.LabelFrame(self, text="Global Settings", padx=8, pady=6)
        top_frame.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 5))
        top_frame.columnconfigure(12, weight=1)

        tk.Label(top_frame, text="Number of movable lips:").grid(row=0, column=0, sticky="w")
        self.lip_count_spin = tk.Spinbox(
            top_frame,
            from_=MIN_LIPS,
            to=MAX_LIPS,
            width=5,
            textvariable=self.lip_count_var,
            command=self.rebuild_dynamic_inputs,
        )
        self.lip_count_spin.grid(row=0, column=1, padx=(6, 14), sticky="w")

        tk.Button(top_frame, text="Apply lips count", command=self.rebuild_dynamic_inputs).grid(
            row=0, column=2, padx=(0, 18), sticky="w"
        )

        tk.Label(top_frame, text=f"Target: always pin hole {TARGET_HOLE}").grid(
            row=0, column=3, padx=(0, 18), sticky="w"
        )
        tk.Label(top_frame, text=f"Boundary: pin holes {MIN_HOLE}..{MAX_HOLE}").grid(
            row=0, column=4, padx=(0, 18), sticky="w"
        )
        tk.Label(top_frame, text="Right vectors are generated automatically.").grid(
            row=0, column=5, sticky="w"
        )

        input_frame = tk.Frame(self)
        input_frame.grid(row=1, column=0, sticky="ew", padx=10, pady=5)
        input_frame.columnconfigure(0, weight=1)
        input_frame.columnconfigure(1, weight=1)

        self.start_frame = tk.LabelFrame(input_frame, text="Start Position", padx=8, pady=8)
        self.start_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 5))

        self.moves_frame = tk.LabelFrame(input_frame, text="Left Movement Vectors", padx=8, pady=8)
        self.moves_frame.grid(row=0, column=1, sticky="nsew", padx=(5, 0))

        controls_frame = tk.LabelFrame(self, text="Actions", padx=8, pady=8)
        controls_frame.grid(row=2, column=0, sticky="ew", padx=10, pady=5)
        controls_frame.columnconfigure(12, weight=1)

        tk.Button(controls_frame, text="Validate", command=self.validate_clicked).grid(row=0, column=0, padx=4)
        tk.Button(controls_frame, text="Solve", command=self.solve_clicked).grid(row=0, column=1, padx=4)
        tk.Button(controls_frame, text="Stress from Start", command=self.stress_from_start_clicked).grid(row=0, column=2, padx=4)
        tk.Button(controls_frame, text="Copy Output", command=self.copy_output).grid(row=0, column=3, padx=4)
        tk.Button(controls_frame, text="Load Example", command=self.load_example).grid(row=0, column=4, padx=4)
        tk.Button(controls_frame, text="Clear Output", command=self.clear_output).grid(row=0, column=5, padx=4)

        tk.Checkbutton(controls_frame, text="Individual steps", variable=self.show_full_var).grid(row=0, column=6, padx=(18, 4))
        tk.Checkbutton(controls_frame, text="Compact", variable=self.show_compact_var).grid(row=0, column=7, padx=4)
        tk.Checkbutton(controls_frame, text="Pattern", variable=self.show_pattern_var).grid(row=0, column=8, padx=4)

        main_frame = tk.Frame(self)
        main_frame.grid(row=3, column=0, sticky="nsew", padx=10, pady=(5, 10))
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=2)
        main_frame.rowconfigure(0, weight=1)

        self.manual_frame = tk.LabelFrame(main_frame, text="Manual Test Mode", padx=8, pady=8)
        self.manual_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 5))

        output_frame = tk.LabelFrame(main_frame, text="Output", padx=8, pady=8)
        output_frame.grid(row=0, column=1, sticky="nsew", padx=(5, 0))
        output_frame.rowconfigure(0, weight=1)
        output_frame.columnconfigure(0, weight=1)

        self.output_text = tk.Text(output_frame, wrap="none", font=("Consolas", 10))
        self.output_text.grid(row=0, column=0, sticky="nsew")

        y_scroll = tk.Scrollbar(output_frame, orient="vertical", command=self.output_text.yview)
        y_scroll.grid(row=0, column=1, sticky="ns")
        self.output_text.configure(yscrollcommand=y_scroll.set)

        x_scroll = tk.Scrollbar(output_frame, orient="horizontal", command=self.output_text.xview)
        x_scroll.grid(row=1, column=0, sticky="ew")
        self.output_text.configure(xscrollcommand=x_scroll.set)

    def rebuild_dynamic_inputs(self) -> None:
        """Rebuild all inputs after the number of lips changes."""
        try:
            lip_count = int(self.lip_count_var.get())
        except (tk.TclError, ValueError):
            messagebox.showerror("Error", "Invalid lip count.")
            return

        if not (MIN_LIPS <= lip_count <= MAX_LIPS):
            messagebox.showerror("Error", f"Lip count must be between {MIN_LIPS} and {MAX_LIPS}.")
            return

        old_start_holes = [var.get() for var in self.start_vars] if self.start_vars else list(state_to_holes(DEFAULT_START))
        old_moves = [[var.get() for var in row] for row in self.move_vars] if self.move_vars else []

        for child in self.start_frame.winfo_children():
            child.destroy()
        for child in self.moves_frame.winfo_children():
            child.destroy()
        for child in self.manual_frame.winfo_children():
            child.destroy()

        # Start position input, displayed as human-readable hole numbers.
        self.start_vars = []
        for col in range(lip_count):
            tk.Label(self.start_frame, text=f"L{col + 1}").grid(row=0, column=col, padx=3, pady=(0, 3))
            value = old_start_holes[col] if col < len(old_start_holes) else TARGET_HOLE
            value = max(MIN_HOLE, min(MAX_HOLE, int(value)))
            var = tk.IntVar(value=value)
            self.start_vars.append(var)
            tk.Spinbox(
                self.start_frame,
                from_=MIN_HOLE,
                to=MAX_HOLE,
                width=4,
                textvariable=var,
            ).grid(row=1, column=col, padx=3)

        tk.Button(self.start_frame, text="Manual = Start", command=self.reset_manual_state).grid(
            row=2, column=0, columnspan=lip_count, pady=(8, 0), sticky="ew"
        )

        # Movement vector input. Vectors are relative deltas and stay in -1..1.
        tk.Label(self.moves_frame, text="Move").grid(row=0, column=0, padx=4, pady=(0, 4))
        for col in range(lip_count):
            tk.Label(self.moves_frame, text=f"L{col + 1}").grid(row=0, column=col + 1, padx=3, pady=(0, 4))

        self.move_vars = []
        for row in range(lip_count):
            tk.Label(self.moves_frame, text=f"L{row + 1} left").grid(row=row + 1, column=0, sticky="w", padx=4)
            row_vars: List[tk.IntVar] = []

            for col in range(lip_count):
                if row < len(old_moves) and col < len(old_moves[row]):
                    value = int(old_moves[row][col])
                elif row == col:
                    value = 1
                else:
                    value = 0

                if row == col:
                    value = 1

                var = tk.IntVar(value=value)
                row_vars.append(var)

                spin = tk.Spinbox(
                    self.moves_frame,
                    from_=-1,
                    to=1,
                    width=4,
                    textvariable=var,
                )
                spin.grid(row=row + 1, column=col + 1, padx=3, pady=2)

                if row == col:
                    spin.configure(state="disabled", disabledforeground="black")

            self.move_vars.append(row_vars)

        # Manual test mode.
        self.manual_state_vars = []
        tk.Label(self.manual_frame, text="Current state:").grid(row=0, column=0, columnspan=2, sticky="w")

        for col in range(lip_count):
            tk.Label(self.manual_frame, text=f"L{col + 1}").grid(row=1, column=col, padx=3, pady=(6, 0))
            var = tk.StringVar(value="-")
            self.manual_state_vars.append(var)
            tk.Label(
                self.manual_frame,
                textvariable=var,
                width=4,
                relief="sunken",
                bg="white",
            ).grid(row=2, column=col, padx=3, pady=(0, 6))

        self.manual_status_var = tk.StringVar(value="")
        tk.Label(self.manual_frame, textvariable=self.manual_status_var, anchor="w", justify="left").grid(
            row=3, column=0, columnspan=max(lip_count, 2), sticky="ew", pady=(0, 8)
        )

        self.manual_buttons = {}
        move_button_row = 4
        for lip in range(1, lip_count + 1):
            left_name = f"L{lip} left"
            right_name = f"L{lip} right"

            b_left = tk.Button(
                self.manual_frame,
                text=left_name,
                width=12,
                command=lambda name=left_name: self.manual_move_clicked(name),
            )
            b_left.grid(row=move_button_row + lip - 1, column=0, columnspan=2, sticky="ew", pady=2)
            self.manual_buttons[left_name] = b_left

            b_right = tk.Button(
                self.manual_frame,
                text=right_name,
                width=12,
                command=lambda name=right_name: self.manual_move_clicked(name),
            )
            b_right.grid(row=move_button_row + lip - 1, column=2, columnspan=2, sticky="ew", pady=2, padx=(6, 0))
            self.manual_buttons[right_name] = b_right

        tk.Button(self.manual_frame, text="Reset to Start", command=self.reset_manual_state).grid(
            row=move_button_row + lip_count + 1,
            column=0,
            columnspan=max(lip_count, 4),
            sticky="ew",
            pady=(10, 0),
        )

        self.reset_manual_state()

    # ---------------------------
    # Read and write GUI data
    # ---------------------------

    def load_example(self) -> None:
        """Load the bundled example configuration."""
        self.lip_count_var.set(len(DEFAULT_START))
        self.start_vars = [tk.IntVar(value=value) for value in state_to_holes(DEFAULT_START)]
        self.move_vars = [
            [tk.IntVar(value=value) for value in DEFAULT_LEFT_MOVES[lip]]
            for lip in sorted(DEFAULT_LEFT_MOVES)
        ]
        self.rebuild_dynamic_inputs()
        self.append_output("Current example loaded.\n")

    def get_start(self) -> State:
        """Read displayed hole numbers and convert them to internal indices."""
        holes = tuple(int(var.get()) for var in self.start_vars)
        return holes_to_state(holes)

    def get_left_moves(self) -> Dict[int, Vector]:
        """Read left movement vectors from the GUI."""
        left_moves: Dict[int, Vector] = {}
        for row_index, row_vars in enumerate(self.move_vars, start=1):
            left_moves[row_index] = tuple(int(var.get()) for var in row_vars)
        return left_moves

    def validate_current_config(self) -> Tuple[State, State, Dict[int, Vector], MoveTable]:
        """Read, validate, and build the current lock configuration."""
        start = self.get_start()
        left_moves = self.get_left_moves()
        validate_config(start, left_moves)
        goal = make_goal(start)
        moves = build_moves(left_moves)
        return start, goal, left_moves, moves

    # ---------------------------
    # Buttons
    # ---------------------------

    def validate_clicked(self) -> None:
        try:
            start, goal, _left_moves, _moves = self.validate_current_config()
        except ValueError as exc:
            messagebox.showerror("Validation Failed", str(exc))
            return

        messagebox.showinfo(
            "Validation OK",
            f"Configuration is valid.\nStart: {format_state(start)}\nTarget: {format_state(goal)}",
        )
        self.update_manual_panel()

    def solve_clicked(self) -> None:
        try:
            start, goal, _left_moves, moves = self.validate_current_config()
        except ValueError as exc:
            messagebox.showerror("Validation Failed", str(exc))
            return

        path, checked_states, discovered_states = solve_bfs(start, goal, moves)
        text = format_solution(
            start=start,
            goal=goal,
            path=path,
            checked_states=checked_states,
            discovered_states=discovered_states,
            show_full=self.show_full_var.get(),
            show_compact=self.show_compact_var.get(),
            show_pattern=self.show_pattern_var.get(),
        )
        self.set_output(text)
        self.last_solution_text = text

    def stress_from_start_clicked(self) -> None:
        try:
            start, _goal, _left_moves, moves = self.validate_current_config()
        except ValueError as exc:
            messagebox.showerror("Validation Failed", str(exc))
            return

        text = format_stress_moves(start, moves)
        self.set_output(text)
        self.last_solution_text = text

    def copy_output(self) -> None:
        text = self.output_text.get("1.0", "end-1c")
        self.clipboard_clear()
        self.clipboard_append(text)
        self.update()
        messagebox.showinfo("Copied", "Output copied to clipboard.")

    def clear_output(self) -> None:
        self.set_output("")

    # ---------------------------
    # Manual mode
    # ---------------------------

    def reset_manual_state(self) -> None:
        try:
            self.manual_state = self.get_start()
        except Exception:
            self.manual_state = tuple(TARGET_INDEX for _ in range(int(self.lip_count_var.get())))
        self.update_manual_panel()

    def manual_move_clicked(self, move_name: str) -> None:
        try:
            _start, goal, _left_moves, moves = self.validate_current_config()
        except ValueError as exc:
            messagebox.showerror("Validation Failed", str(exc))
            return

        move_vector = moves[move_name]
        next_state = apply_move(self.manual_state, move_vector)

        if not is_stress_free(next_state):
            bad_positions = [
                f"L{index + 1}=hole {index_to_hole(value)}"
                for index, value in enumerate(next_state)
                if value < MIN_INDEX or value > MAX_INDEX
            ]
            messagebox.showwarning(
                "Stress",
                f"{move_name} would cause stress.\n"
                f"Next state: {format_state(next_state)}\n"
                f"Problem: {', '.join(bad_positions)}"
            )
            return

        self.manual_state = next_state
        self.update_manual_panel()

        if self.manual_state == goal:
            messagebox.showinfo("Solved", "Target state reached.")

    def update_manual_panel(self) -> None:
        lip_count = int(self.lip_count_var.get())

        if len(self.manual_state) != lip_count:
            self.manual_state = self.get_start()

        displayed = state_to_holes(self.manual_state)
        for index, var in enumerate(self.manual_state_vars):
            value = displayed[index] if index < len(displayed) else "-"
            var.set(str(value))

        try:
            _start, goal, _left_moves, moves = self.validate_current_config()
        except ValueError:
            self.manual_status_var.set("Invalid configuration.")
            for button in self.manual_buttons.values():
                button.configure(bg="SystemButtonFace", activebackground="SystemButtonFace")
            return

        if self.manual_state == goal:
            self.manual_status_var.set("Target reached.")
        else:
            self.manual_status_var.set(f"Target: {format_state(goal)}")

        for move_name, button in self.manual_buttons.items():
            next_state = apply_move(self.manual_state, moves[move_name])
            if is_stress_free(next_state):
                button.configure(bg="#dff0d8", activebackground="#dff0d8")
            else:
                button.configure(bg="#f2dede", activebackground="#f2dede")

    # ---------------------------
    # Output
    # ---------------------------

    def set_output(self, text: str) -> None:
        self.output_text.delete("1.0", "end")
        self.output_text.insert("1.0", text)
        self.output_text.see("1.0")

    def append_output(self, text: str) -> None:
        self.output_text.insert("end", text)
        self.output_text.see("end")


# ============================================================
# CLI TEST
# ============================================================

def run_self_test() -> None:
    """Run a small non-GUI test for CI or quick local verification."""
    start = DEFAULT_START
    left_moves = DEFAULT_LEFT_MOVES
    validate_config(start, left_moves)
    goal = make_goal(start)
    moves = build_moves(left_moves)
    path, checked, discovered = solve_bfs(start, goal, moves)

    print("SELF-TEST")
    print(f"Start: {format_state(start)}")
    print(f"Target: {format_state(goal)}")
    print(f"Checked states:    {checked}")
    print(f"Discovered states: {discovered}")

    if path is None:
        print("No solution found for the default example.")
    else:
        print(f"Solution found: {len(path)} individual moves")
        print("First move:", path[0] if path else "-")
        print("Last move:", path[-1] if path else "-")

    # Pattern compression sanity check: ABABAB -> 3x [A, B]
    fake_groups: CompressedPath = [
        (1, "A", (0,)),
        (1, "B", (1,)),
        (1, "A", (2,)),
        (1, "B", (3,)),
        (1, "A", (4,)),
        (1, "B", (5,)),
    ]
    pattern = compress_repeated_blocks(fake_groups)
    assert len(pattern) == 1
    assert pattern[0]["type"] == "block"
    assert pattern[0]["repeat_count"] == 3
    print("Pattern compression: OK")

    assert state_to_holes((0, 3, 6)) == (1, 4, 7)
    assert holes_to_state((1, 4, 7)) == (0, 3, 6)
    print("Display conversion: OK")

    assert "left" in next(iter(moves.keys()))
    assert all("left" in key or "right" in key for key in moves.keys())
    print("Move naming: OK")


def main() -> None:
    parser = argparse.ArgumentParser(description="Lock Solver GUI")
    parser.add_argument("--test", action="store_true", help="Run solver/formatter tests without starting the GUI.")
    args = parser.parse_args()

    if args.test:
        run_self_test()
        return

    app = LockSolverApp()
    app.mainloop()


if __name__ == "__main__":
    main()
