# Refactoring Plan: Remove CLI, Return to Python Library

**Goal**: Convert pyldcn from a CLI-first tool back to a pure Python library meant to be called from Python scripts.

**Date**: 2025-11-07

---

## Phase 1: Preserve Current State ✓

### Step 1.1: Tag current release as 'CLI-friendly'
- Create git tag: `CLI-friendly`
- Push tag to track this milestone
- **Commit**: "Tag current state as CLI-friendly before refactoring"

---

## Phase 2: Create Refactoring Branch ✓

### Step 2.1: Create and checkout new branch
- Branch name: `remove-cli`
- Based on: current `main` branch
- **Commit**: N/A (branch creation)

---

## Phase 3: Remove CLI Components

### Step 3.1: Identify CLI files to remove
Files to DELETE:
- `pyldcn/cli/device.py` - Device CLI commands
- `pyldcn/cli/motion.py` - Motion CLI commands (drive home, moveabs, etc.)
- `pyldcn/cli/network.py` - Network CLI commands (ldcn)
- `pyldcn/cli/__init__.py` - CLI module init

### Step 3.2: Remove CLI entry points
- Remove from `setup.py` or `pyproject.toml`:
  - `ldcn` console script
  - `drive` console script
  - `device` console script (if exists)

### Step 3.3: Convert CLI functions to library functions
Location: `pyldcn/command/axis.py`

Current CLI-oriented methods that are FINE AS-IS (already library functions):
- `AxisController.home_axis()` - Keep as-is
- `AxisController.move_absolute()` - Keep as-is
- `AxisController.move_relative()` - Keep as-is
- `AxisController.get_position()` - Keep as-is
- `AxisController.wait_for_motion_complete()` - Keep as-is

Changes needed:
- Remove excessive `print()` statements (or make them optional via logging)
- Add proper logging instead of print statements
- Return status/results instead of printing

### Step 3.4: Delete CLI directory and files
**Commit**: "Remove CLI interface files (device.py, motion.py, network.py)"

---

## Phase 4: Update Entry Points

### Step 4.1: Update setup.py/pyproject.toml
- Remove `console_scripts` entry points:
  - `ldcn`
  - `drive`
  - `device`
- Keep the package installable as a library

**Commit**: "Remove CLI console script entry points"

---

## Phase 5: Update Core Library for Library Usage

### Step 5.1: Update pyldcn/command/axis.py
- Add Python logging instead of print statements
- Make output optional/configurable
- Keep all functionality intact
- Add docstrings emphasizing library usage

Changes:
```python
import logging

logger = logging.getLogger(__name__)

# Replace print() with logger.info() / logger.debug()
# Make verbose output optional via logging level
```

**Commit**: "Refactor axis.py: Replace print statements with logging"

### Step 5.2: Update pyldcn/network.py (if needed)
- Review for CLI-specific code
- Ensure clean library interface
- Add logging if needed

**Commit**: "Update network.py for library usage (if changes made)"

---

## Phase 6: Update Documentation

### Step 6.1: Update README.md
Current focus: CLI usage (`ldcn`, `drive` commands)

New focus: Library usage from Python scripts

Changes:
- Remove CLI installation instructions
- Remove CLI command examples
- Add library usage examples:
  - Initializing network
  - Controlling axes
  - Homing
  - Moving to positions
- Add code examples showing imports and usage

**Commit**: "Update README.md: Document library usage instead of CLI"

### Step 6.2: Update/Create example scripts
Create/update:
- `examples/home_axis.py` - Example: Home a single axis
- `examples/move_axis.py` - Example: Move axis to position
- `examples/read_status.py` - Example: Read device status
- Update existing examples to reflect library usage

**Commit**: "Add example scripts demonstrating library usage"

### Step 6.3: Update technical documentation
Files to update:
- `docs/servo_commands.md` - Keep as-is (protocol reference)
- `docs/homing_procedure.md` - Update examples to show library usage
- `docs/io_commands.md` - Keep as-is (protocol reference)
- Any other docs referencing CLI commands

**Commit**: "Update documentation: Remove CLI references, add library examples"

---

## Phase 7: Testing and Validation

### Step 7.1: Test library imports
```python
# Test that library can be imported and used
from pyldcn import LDCNNetwork
from pyldcn.command import AxisController
```

### Step 7.2: Test core functionality
- Test that existing test scripts still work
- Verify `test_power_relay.py` works
- Check example scripts run correctly

**Commit**: "Verify library functionality after CLI removal" (if fixes needed)

---

## Phase 8: Cleanup

### Step 8.1: Remove any remaining CLI artifacts
- Search for CLI-specific code patterns
- Remove argparse imports if unused
- Clean up unused imports

**Commit**: "Clean up remaining CLI artifacts"

### Step 8.2: Update package metadata
- Update `setup.py` or `pyproject.toml` description
- Change from "CLI tool" to "Python library"
- Update classifiers if present

**Commit**: "Update package metadata: CLI tool → Python library"

---

## Phase 9: Finalization

### Step 9.1: Review all changes
- Review git diff
- Verify no broken imports
- Check that library interface is clean
- Ensure documentation is complete

### Step 9.2: Merge to main (after user approval)
- User reviews the branch
- If approved: merge `remove-cli` → `main`
- Tag as new version (e.g., `v2.0.0-library`)

### Step 9.3: Remove this planning document
- Delete `REFACTOR_PLAN.md`
- **Commit**: "Remove refactoring plan document"

---

## Summary of Expected Changes

**Files to DELETE**:
- `pyldcn/cli/device.py`
- `pyldcn/cli/motion.py`
- `pyldcn/cli/network.py`
- `pyldcn/cli/__init__.py`
- Possibly entire `pyldcn/cli/` directory

**Files to MODIFY**:
- `setup.py` or `pyproject.toml` (remove console_scripts)
- `pyldcn/command/axis.py` (add logging, reduce print statements)
- `README.md` (complete rewrite focusing on library usage)
- `docs/homing_procedure.md` (update examples)
- Any other docs with CLI references

**Files to CREATE**:
- `examples/home_axis.py` (library usage example)
- `examples/move_axis.py` (library usage example)
- Other example scripts as needed

**Files to KEEP AS-IS**:
- `pyldcn/network.py` (core library)
- `pyldcn/command/__init__.py`
- `pyldcn/command/axis.py` (after modifications)
- `docs/servo_commands.md` (protocol reference)
- `docs/io_commands.md` (protocol reference)
- Test scripts using the library

---

## Rollback Plan

If issues arise:
1. Checkout main: `git checkout main`
2. The `CLI-friendly` tag preserves the working CLI version
3. Can restart refactoring or keep CLI version

---

## Notes

- The core library functionality remains intact
- Only the CLI interface is removed
- Users will import and call functions directly instead of using CLI commands
- This is a major version change (2.0.0 suggested)
- The `CLI-friendly` tag allows returning to CLI version if needed

---

**Status**: ⏸️ Awaiting execution

**Next Step**: Phase 1 - Tag current state as 'CLI-friendly'
