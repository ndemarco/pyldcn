# Documentation Review and Recommendations

**Date:** 2025-11-20
**Scope:** All markdown files in `/docs` (excluding `design/` subfolder)
**Files Reviewed:** 15 total

---

## Executive Summary

The documentation is comprehensive and well-organized overall, but there are opportunities to improve consistency, eliminate duplication, and enhance cross-referencing. The main issues are:

1. **Substantial duplication** between status reporting documents
2. **Formatting inconsistencies** across files
3. **Incomplete cross-references** between related topics
4. **Information placement issues** (some content appears in unexpected locations)
5. **Minor conflicting information** about specific details

---

## 1. Substantially Duplicated Sections

### 1.1 Status Reporting Duplication (HIGH PRIORITY)

**Files Affected:**
- `servo_status_reporting.md`
- `LS-231SE/LS-231SE_status.md`
- `LS-231SE/servo_api_reference.md`

**Duplicated Content:**
- Status byte bit definitions (appears in all 3 files)
- Auxiliary status byte definitions (appears in all 3 files)
- Status item configuration bits (appears in 2 files)
- Diagnostic state tables (appears in 2 files with slightly different presentations)

**Recommendation:**
- **Keep:** `LS-231SE/LS-231SE_status.md` as the comprehensive reference for LS-231SE status/diagnostics
- **Simplify:** `servo_status_reporting.md` to be a brief overview with links to the detailed LS-231SE doc
- **Refactor:** `servo_api_reference.md` to reference status definitions instead of duplicating them

---

### 1.2 I/O Commands Duplication (MEDIUM PRIORITY)

**Files Affected:**
- `io_status_reporting.md`
- `sk-2310g2_io_mapping.md`

**Duplicated Content:**
- Digital input/output byte layouts
- Status reporting mechanism (Define Status, Read Status commands)
- Command reference tables

**Recommendation:**
- **Keep:** `sk-2310g2_io_mapping.md` as the complete I/O reference (it's device-specific and more detailed)
- **Refactor:** `io_status_reporting.md` to be a generic LS-773 protocol guide that references sk-2310g2 for specific implementation
- Consider renaming `io_status_reporting.md` to `LS-773_protocol.md` for clarity

---

### 1.3 LDCN Protocol Basics (LOW PRIORITY)

**Files Affected:**
- `ldcn_protocol.md`
- Multiple device-specific files reference LDCN basics

**Current State:** Well-handled with good cross-referencing

**Minor Issue:** Some files duplicate the packet structure diagram

**Recommendation:**
- Add a standard reference section at the top of device-specific docs pointing to `ldcn_protocol.md`
- Remove packet structure diagrams from device-specific docs

---

## 2. Out of Place Information

### 2.1 Power Control Documentation

**File:** `power_control.md`
**Issue:** Extremely brief (15 lines) with minimal structure, references `test_power_control.py` script for logic

**Recommendation:**
Move this content into `SK-2310g2_supervisor.md` under the "Power Control and Monitoring" section (which already exists but could be expanded with this information). Delete `power_control.md` as a standalone file.

**Rationale:** Power control is SK-2310g2-specific functionality, not generic LDCN

---

### 2.2 Persistent Mode Information

**File:** `LS-231SE/LS-231SE_persistent_mode.md`
**Issue:** Very brief (30 lines), discusses NVRAM storage but notes it's "undocumented" and "of no direct use" for LDCN modes

**Recommendation:**
Move this content into `LS-231SE/LS-231SE_initialization.md` as a subsection titled "NVRAM Storage and Device Mode", since it's initialization-related. Delete the standalone file.

---

### 2.3 Initialization Documentation

**File:** `LS-231SE/LS-231SE_initialization.md`
**Issue:** Only 7 steps listed, no elaboration, very minimal (40 lines total)

**Recommendation:**
Merge with `LS-231SE/servo_api_reference.md` "Quick Start" section, which already has initialization examples. The standalone init doc doesn't provide enough value.

---

## 3. Conflicting Information

### 3.1 Safety Bus ServoFAULT Direction (MINOR)

**Location:** `safety_bus.md` line 54-61

**Conflict:**
```
Pin 4: ServoFAULT | Input/Output | Device-specific
```

With a "CLAUDE: Confirm..." note indicating uncertainty about whether it's truly bidirectional.

**Files with related info:**
- `safety_bus.md` (generic spec)
- `SK-2310g2_supervisor.md` (SK-2310g2 treats it as input)
- LS-231SE docs imply it's an output

**Recommendation:**
Clarify that:
- SK-2310g2: ServoFAULT is an **INPUT** (monitors servo drives)
- LS-231SE: ServoFAULT is an **OUTPUT** (reports drive faults)
- Update `safety_bus.md` to reflect this device-role distinction

---

### 3.2 Spindle Control Safety Constraints

**Location:** `sk-2310g2_io_mapping.md` line 173-178

**Note 3** states: "Spindle ON (Bit 2) and Safety Link Bridge (Bit 12) must not be set simultaneously."

**Issue:** This critical safety constraint appears only in the I/O mapping doc, not in the supervisor safety documentation

**Recommendation:**
Add this safety constraint prominently to:
- `SK-2310g2_supervisor.md` in the "Spindle Control" section
- Cross-reference from `sk-2310g2_io_mapping.md` to the supervisor doc

---

### 3.3 Guard Lock Control Configuration

**Locations:**
- `SK-2310g2_supervisor.md` (multiple mentions of J19 jumper)
- `sk-2310g2_io_mapping.md` (Output 9 / Byte1 Bit1)

**Potential Conflict:** J14/J15 interaction with guard locking is mentioned in supervisor doc but not explained clearly in I/O mapping

**Recommendation:**
Add a note in `sk-2310g2_io_mapping.md` at the Guard Lock output definition referencing the jumper configuration section in `SK-2310g2_supervisor.md`

---

## 4. Missing or Incomplete Cross-References

### 4.1 High-Value Links to Add

| From File | Section | Should Link To | Reason |
|-----------|---------|----------------|---------|
| `ldcn_protocol.md` | "Define Status" command | `servo_status_reporting.md` | Device-specific status details |
| `ldcn_protocol.md` | "Group Addressing" section | `LS-231SE/LS-231SE_commands.md` "Start Motion" | Synchronized motion example |
| `servo_status_reporting.md` | "Status Items" table | `LS-231SE/LS-231SE_status.md` | Complete diagnostic reference |
| `io_status_reporting.md` | All command references | `sk-2310g2_io_mapping.md` | Specific I/O implementation |
| `LS-231SE/LS-231SE_commands.md` | "Set Homing Mode" | `LS-231SE/LS-231SE_homing.md` | Detailed homing procedures |
| `LS-231SE/LS-231SE_commands.md` | "I/O Control" | `LS-231SE/LS-231SE_path_point.md` | Path point timing details |
| `LS-231SE/LS-231SE_homing.md` | Limit switch references | `LS-231SE/LS-231SE_IO.md` | Digital I/O mapping |
| `LS-231SE/LS-231SE_IO.md` | HomeSEL table | `servo_status_reporting.md` | Status bit resolution |
| `LS-231SE/LS-231SE_status.md` | All command references | `LS-231SE/LS-231SE_commands.md` | Command details |
| `SK-2310g2_supervisor.md` | Safety Bus section | `safety_bus.md` | Complete safety bus spec |
| `SK-2310g2_supervisor.md` | I/O references | `sk-2310g2_io_mapping.md` | Complete I/O mapping |
| `sk-2310g2_io_mapping.md` | Status reporting | `io_status_reporting.md` | LS-773 protocol details |
| `safety_bus.md` | Device examples | `SK-2310g2_supervisor.md`, LS-231SE docs | Device-specific implementations |

---

### 4.2 Missing Section Anchors

Many files reference sections without providing anchor links. Add anchors to:

**In `LS-231SE/LS-231SE_commands.md`:**
- Each command heading (e.g., `## Load Trajectory {#load-trajectory}`)
- Extended command subsections

**In `SK-2310g2_supervisor.md`:**
- Each jumper section (e.g., `### J10 - Safe State Detection {#j10-safe-state}`)
- Each diagnostic code
- Each connector section

**In `io_status_reporting.md`:**
- Each command section
- Status item definitions

---

## 5. Formatting Variations

### 5.1 Inconsistent Command Reference Formatting

**Variations Found:**

**Style A** (Used in `LS-231SE/LS-231SE_commands.md`):
```markdown
### Load Trajectory

**Command:** `0x04` (CMD_LOAD_TRAJ)<br>
**Data bytes:** 1 to 15 bytes<br>
**Returns:** Yes - Standard status packet
```

**Style B** (Used in `io_status_reporting.md`):
```markdown
### Set PWM

**Command:** `0x04` (CMD_SET_PWM)
**Data bytes:** 2 bytes
**Returns:** Yes - Standard status packet
```

**Recommendation:**
- **Preferred Style:** Style B (no `<br>` tags, cleaner markdown)
- Apply consistently across all command reference sections

---

### 5.2 Inconsistent Table Formatting

**Issue:** Some tables use centered alignment, others don't; some use `---` separators, others use `|---|`

**Examples:**
- `ldcn_protocol.md`: Uses `|-----|` with left alignment
- `LS-231SE/LS-231SE_status.md`: Uses `|:-:|:-:|` for centered columns
- `safety_bus.md`: Mix of both styles

**Recommendation:**
- **For text-heavy tables:** Left alignment (easier to read)
- **For status/flag tables:** Centered alignment for bit values
- **Always use:** `|---|` style separator (more compatible with markdown parsers)

---

### 5.3 Code Block Language Tags

**Issue:** Inconsistent language tags in code blocks

**Found:**
- `` ```python `` (most common)
- `` ``` `` (no language tag)
- `` ```text `` (for LDCN packet examples)
- No language tag for ASCII diagrams

**Recommendation:**
- Python code: `` ```python ``
- LDCN packets/hex: `` ```text `` or `` ```hexdump ``
- ASCII diagrams: `` ```text ``
- Ensure ALL code blocks have language tags

---

### 5.4 Heading Hierarchy Issues

**Files with issues:**
- `LS-231SE/LS-231SE_path_point.md` - Uses "##" for title but then jumps to "**" for subsections
- `LS-231SE/LS-231SE_status.md` - Inconsistent heading levels (some sections are ##, others ###)

**Recommendation:**
```
# Title (document name)
## Major Section
### Subsection
#### Detail level
```

Audit all files for proper heading hierarchy.

---

### 5.5 Metadata Headers

**Inconsistent metadata across files:**

**Some files have:**
```markdown
**Author:** NickyDoes
**Source:** [document reference]
**Date:** 2025-11-13
```

**Others have:**
```markdown
This document describes...
[No metadata]
```

**Recommendation:**
Add consistent metadata to ALL documentation files:
```markdown
# Document Title

**Author:** NickyDoes
**Last Updated:** YYYY-MM-DD
**Source:** [Primary reference document]
**Status:** [Complete | In Progress | Needs Review]
```

---

## 6. Specific Formatting Improvements Needed

### 6.1 `LS-231SE/LS-231SE_path_point.md`

**Issues:**
- Line 49: "TODO: Make this a bulleted list"
- Line 51: "TODO: Add super short summary..."
- Line 56: "TODO: Also add a summary and link"
- Incomplete sections: "Related Commands", "Related Status"

**Action Required:**
- Complete all TODO items
- Add proper cross-references to relevant commands
- Add links to status reporting documentation

---

### 6.2 `LS-231SE/LS-231SE_commands.md`

**Issues:**
- Line 445: "TODO: Add reference and link to LS-231SE_path_point.md file"
- Extended commands section could use more examples

**Action Required:**
- Add the missing link
- Consider adding a "Common Command Sequences" section with workflows

---

### 6.3 `SK-2310g2_supervisor.md`

**Issues:**
- Line 54: "CLAUDE: Confirm 4 - ServoFAULT..." (unresolved question)
- Line 249: "CLAUDE: Confirm if all faults must be reset..." (unresolved)
- Very long file (830 lines) - could be split into logical sections

**Action Required:**
- Resolve CLAUDE confirmation questions
- Consider splitting into separate files:
  - `SK-2310g2_supervisor.md` (overview + safety system)
  - `SK-2310g2_jumpers.md` (jumper configuration)
  - `SK-2310g2_diagnostics.md` (diagnostic codes)

---

### 6.4 `sk-2310g2_io_mapping.md`

**Issue:**
- Line 157: Stray characters `$H_2O$` (appears to be a LaTeX artifact)

**Action Required:**
- Remove the stray characters
- Review entire file for other LaTeX/formatting artifacts

---

## 7. Link Format Preferences

### Current Practices

The docs use several link formats:

**Format A - Relative with file extension:**
```markdown
See [servo_commands.md](servo_commands.md)
```

**Format B - Relative without extension:**
```markdown
See [servo_commands](servo_commands)
```

**Format C - With section anchor:**
```markdown
See [Set Homing Mode](servo_commands.md#set-homing-mode)
```

**Format D - External reference:**
```markdown
See `servo_commands.md` (no link)
```

---

### Recommendation: Which format do you prefer?

**Option 1: Explicit file extension (more robust)**
```markdown
[link text](filename.md#section)
```
✅ Works in all markdown viewers
✅ GitHub renders correctly
✅ Clear what file you're linking to

**Option 2: No extension (cleaner)**
```markdown
[link text](filename#section)
```
✅ Cleaner appearance
⚠️ May not work in all viewers
❌ Less clear for maintenance

**Option 3: Relative path format (most explicit)**
```markdown
[link text](../docs/filename.md#section)
```
✅ Unambiguous
✅ Works across directory structures
❌ More verbose

---

**My recommendation:** **Option 1** (with file extension)
- Most compatible
- Balances clarity and cleanliness
- Standard practice in most documentation projects

---

## 8. Summary of Action Items

### High Priority (Eliminate Major Duplication)

1. ✅ **Refactor status reporting docs** to eliminate duplication between:
   - `servo_status_reporting.md` (make brief overview)
   - `LS-231SE/LS-231SE_status.md` (keep as comprehensive reference)
   - `LS-231SE/servo_api_reference.md` (reference instead of duplicate)

2. ✅ **Merge power control** content from `power_control.md` into `SK-2310g2_supervisor.md`

3. ✅ **Complete TODO items** in:
   - `LS-231SE/LS-231SE_path_point.md`
   - `LS-231SE/LS-231SE_commands.md`

### Medium Priority (Improve Organization)

4. ✅ **Move NVRAM content** from `LS-231SE/LS-231SE_persistent_mode.md` into initialization doc

5. ✅ **Add missing cross-references** per table in section 4.1

6. ✅ **Resolve conflicts:**
   - ServoFAULT direction clarification
   - Spindle safety constraints placement
   - Guard lock configuration clarity

7. ✅ **Fix formatting issues:**
   - Remove stray `$H_2O$` characters
   - Resolve CLAUDE confirmation questions
   - Standardize command reference formatting

### Low Priority (Polish)

8. ✅ **Standardize formatting:**
   - Code block language tags
   - Table alignment styles
   - Heading hierarchy
   - Metadata headers

9. ✅ **Add section anchors** for deep linking throughout documentation

10. ✅ **Consider splitting** `SK-2310g2_supervisor.md` into smaller files

---

## 9. Files Recommended for Consolidation

| Action | Files | New Organization |
|--------|-------|------------------|
| **Merge** | `power_control.md` → `SK-2310g2_supervisor.md` | Delete `power_control.md` |
| **Merge** | `LS-231SE/LS-231SE_persistent_mode.md` → `LS-231SE/LS-231SE_initialization.md` | Delete `persistent_mode.md` |
| **Merge** | `LS-231SE/LS-231SE_initialization.md` → `LS-231SE/servo_api_reference.md` | Delete `initialization.md` |
| **Refactor** | `io_status_reporting.md` | Rename to `LS-773_protocol.md`, remove SK-2310g2-specific content |
| **Refactor** | `servo_status_reporting.md` | Simplify to brief overview, link to `LS-231SE/LS-231SE_status.md` |

**Net result:** 15 files → 12 files (cleaner, less duplication)

---

## 10. Positive Observations

The documentation has many strengths:

✅ **Comprehensive coverage** of all device functionality
✅ **Detailed command references** with examples
✅ **Good use of tables** for specifications
✅ **Safety information** is prominent and thorough
✅ **Real-world examples** in many sections
✅ **Clear hardware/software distinction**
✅ **Good ASCII diagrams** for wiring/topology
✅ **Consistent terminology** within individual files

---

## 11. Next Steps

Please review these recommendations and let me know:

1. **Which link format you prefer** (Option 1, 2, or 3 from section 7)
2. **Priority order** - which issues should I tackle first?
3. **File consolidation approval** - are you okay with merging/deleting the files listed in section 9?
4. **Formatting preferences** - any specific style choices I should follow?

Once I have your feedback, I can begin implementing the changes systematically.

---

**End of Review**
