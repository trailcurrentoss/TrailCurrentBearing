# TrailCurrent Bearing

![TrailCurrent Bearing](DOCS/images/bearing_assembled.png)

GNSS (Global Navigation Satellite System) module for precise positioning, timing, and navigation. Part of the [TrailCurrent](https://trailcurrent.com) open-source vehicle platform.

## Project Overview

TrailCurrent Bearing is a standalone GNSS module that utilizes the DFRobot Gravity GNSS receiver. The data that comes in is formatted and sent over the CAN bus for other modules to use.

**Example Use Cases:**
- Digital clocks with precise time synchronization
- Navigation and geolocation devices
- Geospatial macros that control devices based on pinned locations
- Vehicle tracking and positioning systems

## Hardware Overview

- **Function:** Precision GNSS receiver with CAN bus interface
- **Key Features:**
  - DFRobot Gravity GNSS receiver supporting multiple satellite systems
  - CAN bus output for integration with TrailCurrent ecosystem
  - Flexible GNSS mode selection (GPS, BeiDou, GLONASS, combinations)
  - Hierarchical schematic design (5 sheets) for system modularity
  - Power management and signal conditioning

## Hardware Requirements

### Components

- **GNSS Receiver:**
  - DFRobot Gravity GNSS module with multi-constellation support

- **Microcontroller:**
  - ESP32 or equivalent for CAN interface and data processing

- **Power Management:**
  - Buck converter for regulated power supply

- **Interface:**
  - CAN transceiver for CAN bus communication
  - JST XH connectors for power and signal distribution

### KiCAD Library Dependencies

This project uses the consolidated [TrailCurrentKiCADLibraries](https://github.com/trailcurrentoss/TrailCurrentKiCADLibraries).

**Setup:**

```bash
# Clone the library
git clone git@github.com:trailcurrentoss/TrailCurrentKiCADLibraries.git

# Set environment variables (add to ~/.bashrc or ~/.zshrc)
export TRAILCURRENT_SYMBOL_DIR="/path/to/TrailCurrentKiCADLibraries/symbols"
export TRAILCURRENT_FOOTPRINT_DIR="/path/to/TrailCurrentKiCADLibraries/footprints"
export TRAILCURRENT_3DMODEL_DIR="/path/to/TrailCurrentKiCADLibraries/3d_models"
```

See [KICAD_ENVIRONMENT_SETUP.md](https://github.com/trailcurrentoss/TrailCurrentKiCADLibraries/blob/main/KICAD_ENVIRONMENT_SETUP.md) in the library repository for detailed setup instructions.

## Opening the Project

1. **Set up environment variables** (see Library Dependencies above)
2. **Open KiCAD:**
   ```bash
   kicad /path/to/EDA/trailcurrent-gps-module/trailcurrent-gps-module.kicad_pro
   ```
3. **Verify libraries load** - All symbol and footprint libraries should resolve without errors
4. **Navigate hierarchical sheets:**
   - Main sheet links to sub-modules for power management, GNSS interface, CAN interface, etc.
   - Each sheet is self-contained but references consolidated library components
5. **View 3D models** - Open PCB and press `Alt+3` to view the 3D visualization
   - All 3D models should load from the consolidated library path
   - Models use relative `${TRAILCURRENT_3DMODEL_DIR}` references

## Firmware

See `src/` directory for PlatformIO-based firmware implementing CAN bus communication and GNSS data processing.

**Setup:**
```bash
# Install PlatformIO (if not already installed)
pip install platformio

# Build firmware
pio run -d /path/to/project

# Upload to board
pio run -t upload -d /path/to/project
```

## GNSS Modes

The DFRobot Gravity receiver supports multiple satellite constellation configurations:

- **Mode 1:** GPS only
- **Mode 2:** BeiDou only
- **Mode 3:** GPS + BeiDou
- **Mode 4:** GLONASS only
- **Mode 5:** GPS + GLONASS
- **Mode 6:** BeiDou + GLONASS
- **Mode 7:** GPS + BeiDou + GLONASS (Maximum constellation)

Configure the desired mode in firmware initialization. Multi-constellation modes provide better coverage, especially in urban canyons and challenging environments.

## Manufacturing

- **PCB Files:** Ready for fabrication via standard PCB services (JLCPCB, OSH Park, etc.)
- **BOM Generation:** Export BOM from KiCAD schematic (Tools → Generate BOM)
- **JLCPCB Assembly:** This project supports automated assembly using JLCPCB
  - See [BOM_ASSEMBLY_WORKFLOW.md](https://github.com/trailcurrentoss/TrailCurrentKiCADLibraries/blob/main/BOM_ASSEMBLY_WORKFLOW.md) for detailed assembly workflow
  - Component library includes LCSC, Mouser, and DigiKey part numbers for flexible sourcing

## Project Structure

- **EDA/trailcurrent-gps-module/** - KiCAD schematic (5 sheets) and PCB design files
  - Main module sheet with sub-sheet references
  - Power management circuit
  - GNSS interface circuit
  - CAN interface circuit
  - Support circuits (regulators, filtering, etc.)
- **src/** - Firmware source code for GNSS data processing and CAN interface
- **lib/** - Custom libraries and dependencies
- **include/** - Header files

## Documentation

- **Main Schematic:** EDA/trailcurrent-gps-module/trailcurrent-gps-module.kicad_sch (with 5 sheet hierarchy)
- **PCB Layout:** EDA/trailcurrent-gps-module/trailcurrent-gps-module.kicad_pcb

## License

MIT License - See LICENSE file for details

This is open source hardware. You are free to use, modify, and distribute these designs for personal or commercial purposes.

## Contributing

Improvements and contributions are welcome! Please submit issues or pull requests to the main repository.

## Support

For questions about:
- **KiCAD setup:** See [KICAD_ENVIRONMENT_SETUP.md](https://github.com/trailcurrentoss/TrailCurrentKiCADLibraries/blob/main/KICAD_ENVIRONMENT_SETUP.md)
- **Library consolidation:** See [CONNECTOR_CONSOLIDATION_SUMMARY.md](https://github.com/trailcurrentoss/TrailCurrentKiCADLibraries/blob/main/CONNECTOR_CONSOLIDATION_SUMMARY.md)
- **Assembly workflow:** See [BOM_ASSEMBLY_WORKFLOW.md](https://github.com/trailcurrentoss/TrailCurrentKiCADLibraries/blob/main/BOM_ASSEMBLY_WORKFLOW.md)
