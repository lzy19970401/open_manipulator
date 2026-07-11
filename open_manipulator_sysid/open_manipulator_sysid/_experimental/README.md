# Experimental / non-production code

Not imported by the main sysid pipeline (`excitation_*`, `sysid_identify_*`, `sysid_bip_*`, etc.).

- `test_figaroh_optimal.py` — optional FIGAROH trajectory experiment (requires external `figaroh` pip package).
- `figaroh_config/` — OMX yaml for FIGAROH runs (if present).

Production packages live alongside this directory:

- `excitation_trajectory/`
- `excitation_recording/`
- `five_link_dynamics/`
- `system_identification/`
- `model_validation/`
- `pinocchio_support/`
- `reference/`
