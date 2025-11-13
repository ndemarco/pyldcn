# Reorganization Suggestions for io_status_reporting.md

## Critical Issues Found

### 1. Duplicate Headers (Lines 72-74)
**Problem:** Two identical section headers
```markdown
### Status reporting
### Status Reporting
```
**Fix:** Delete line 72, keep only line 74

### 2. Missing Sync Input Command Section
**Problem:** Command table (line 486) references "Sync Input Command" but only narrative section exists at line 348 "Synchronized Input Capture"
**Fix:** Add formal command section matching other commands:
```markdown
### Sync Input Command

**Command:** `0x0C` (CMD_SYNC_INPUT)
**Data bytes:** 0 bytes
**Returns:** Yes - Standard status packet

Captures current input states and counter/timer value atomically.

**Use Case**:
- Atomic snapshot of all digital inputs and counter value
- Useful for synchronized multi-axis position capture
- Captured values are read via Define Status or Read Status commands (bits 6 and 7)

**Workflow**:
[existing code example from line 348 section]

**Note:** There is no trigger or interrupt to capture all values on an input. Capture can only be initiated via a LDCN command and is subject to transmission and command processing delays.
```

### 3. Excessive Blank Lines
- Lines 127-131: 6 blank lines
- Lines 164-165: 2 blank lines (could be 1)
- Lines 344-347: 4 blank lines
**Fix:** Reduce to single blank line between sections

### 4. Capitalization Error (Line 16)
**Problem:** "Specific models" should be "Specific Models"
**Fix:** Change to title case

### 5. Missing Sections
The following sections from earlier versions are missing:
- **Digital Input Configuration** (with pull-up/pull-down details, Active LOW/HIGH)
- **Analog Input Configuration** (voltage ranges, resolution, conversion examples)
- **Output Protection** (short circuit, overcurrent details)

These should be added under new section "I/O Configuration Details"

## Recommended Document Structure

```
# Logosol Network I/O Node Commands

## 1. General Introduction
   [Current content - good as is]

## 2. Device Hardware
   ### 2.1 LS-773 Hardware Features
       - Feature list
       - LS-773 Input Bit Layout
       - OUT_SH Flag
   ### 2.2 SK-2310g2 Hardware Features
       - Description
       - I/O Hardware Differences (comparison table)
       - Analog I/O Summary

## 3. Protocol Concepts
   ### 3.1 Status Reporting Overview
       [Current "Status Reporting" content]
   ### 3.2 Status Response Structure
       - Status Byte
       - Status Items (bitmap table)
       - Checksum Byte
       - Device ID and Version Details
       - Example Configurations (with actual response bytes)
   ### 3.3 Counter/Timer
       - Overview
       - Modes
       - Timer Mode
       - Counter Mode
       - Overflow Behavior
       - Reset

## 4. Command Reference
   ### Command Summary Table
       [Move table from line 477 to here]

   ### 4.1 Define Status Command
   ### 4.2 Read Status Command
   ### 4.3 Set PWM
   ### 4.4 Sync Output
   ### 4.5 Set Outputs
   ### 4.6 Set Sync Output
   ### 4.7 Set Timer Mode
   ### 4.8 Sync Input Command [NEW - needs to be created]

## 5. I/O Configuration Details
   ### 5.1 Digital Input Configuration
       - Active LOW (default) vs Active HIGH
       - Pull resistor configuration
       - Input bit layout (can reference LS-773 section)
   ### 5.2 Analog Input Configuration
       - Voltage range selection
       - Resolution per range
       - Conversion examples
   ### 5.3 Output Protection
       - Short circuit protection
       - Overcurrent protection
       - Recovery procedures

## 6. Implementation Guide
   ### 6.1 Efficient Status Configuration
   ### 6.2 Reading Analog Inputs
   ### 6.3 Performance Considerations
   ### 6.4 Initialization Sequence
   ### 6.5 PWM Output Configuration
   ### 6.6 Synchronized Input Capture Best Practices

## 7. References
   [Move from line 462 to end]
```

## Heading Level Corrections

Current commands are level 3 (###) under "Common Features". They should be:
- If keeping "Command Reference" as level 2 (##), make commands level 3 (###)
- Or make "Command Reference" level 1 and commands level 2 (##)

**Recommendation:** Make commands level 3 under a new level 2 "Command Reference" section

## Grammar and Spelling Issues

1. **Line 99:** Missing space after pipe character
   ```
   | **1** | Checksum error flag |Set if...
   ```
   Should be:
   ```
   | **1** | Checksum error flag | Set if...
   ```

2. **Line 135:** Extra space after period
   ```
   A single 32-bit counter/timer.
   ```

3. **Line 340:** Broken anchor link
   ```
   See [Counter/Timer Overview](#countertimer-overview)
   ```
   Should be:
   ```
   See [Counter/Timer](#countertimer)
   ```

## Structural Improvements

### 1. Add Status Response Format Section
After "Status Response Structure", add subsection with example configurations showing actual byte-by-byte responses (this content exists scattered but should be consolidated)

### 2. Consolidate Related Content
- Move "Synchronized Input Capture" (line 348) narrative into the new "Sync Input Command" section
- Move "Device ID and Version" details (lines 121-125) into a subsection under "Status Response Structure"

### 3. Consistent Section Separators
Some sections use `---`, others don't. Recommend:
- Use `---` only between major sections (level 2 ##)
- Don't use between subsections (level 3 ###)

### 4. Cross-Reference Links
Add links between related sections:
- From command sections to "Implementation Guide" examples
- From "Counter/Timer" overview to "Set Timer Mode" command
- From "Synchronized Input Capture" to "Sync Input Command"

## Command Table Issues

Line 486 references "#sync-input-command" anchor but no such section exists with level 2/3 heading. Either:
1. Create the command section (recommended), OR
2. Update anchor to "#synchronized-input-capture"

## Priority Fixes (Do First)

1. ✅ Delete duplicate "Status reporting" header (line 72)
2. ✅ Fix capitalization "Specific models" → "Specific Models" (line 16)
3. ✅ Add formal "Sync Input Command" section
4. ✅ Remove excessive blank lines (127-131, 344-347)
5. ✅ Fix broken anchor link in line 340
6. ✅ Add missing space in table (line 99)
7. ✅ Move command table from end (line 477) to before command sections
8. ✅ Move References section to end (after Implementation Guide)

## Lower Priority (Nice to Have)

1. Add missing I/O Configuration sections (Digital Input, Analog Input, Output Protection)
2. Consolidate scattered "Status Response Format" content
3. Standardize section separators (---) usage
4. Add cross-reference links between sections
5. Group commands under "Command Reference" section
