# Future Features - Parked Development

This directory contains features that have been developed but are **parked for future integration** after core functionality is verified.

## Status: PARKED 🅿️

These features are complete but **not yet tested** on hardware. They will be integrated after:
1. Core LDCN protocol is verified
2. Axis initialization and movement are working
3. Basic motion control is stable

## Parked Features

### config_management/

Configuration management system for LDCN devices and axes.

**Components:**
- `config/` - Configuration validation and management
  - `axis_config.py` - Axis configuration management
  - `schema.py` - Configuration schemas
  - `exceptions.py` - Config error handling

- `cli/` - Command-line interface
  - `device.py` - `pyldcn-device` CLI tool
  - Commands: discover, validate, diff, merge

- `util.py` - Device list save/load utilities

- `save_load_device_list.py` - Example script

**Purpose:**
- Discover devices and save to JSON
- Define axis configurations (pitch, encoder resolution, PID gains)
- Merge device discovery with axis config
- Validate and compare configurations

**Why Parked:**
Configuration management is useful for complex multi-axis systems, but the priority is to get **basic axis movement working first**. Once we can reliably initialize and move one axis, this config system will make it easy to manage multiple axes.

**Integration Plan:**
1. Verify core motion control works
2. Test with 1-2 axes manually configured
3. Re-integrate config management for production systems
4. Add LinuxCNC HAL integration

## Notes

All code is marked "UNVERIFIED - Not yet tested on hardware" and was developed based on:
- Logosol LDCN protocol documentation
- LS-231SE and SK-2310g2 datasheets
- Best practices for industrial motion control

The code is well-structured and should work, but hardware verification is required before relying on it in production.
