# Bean-Sorter
Project to build a green coffee bean sorter for small farms

Folder directory structure
my_robot_project/
│
├── 📂 docs/                     # DOCUMENTATION (For You & AI)
│   ├── system_context.md        # Your "Source of Truth" (Pinouts, Specs)
│   ├── diagrams.md              # Mermaid diagrams
│   └── architecture.md          # General notes on how things work
│
├── 📂 src/                      # SOURCE CODE (The Logic)
│   ├── __init__.py              # Tells Python this folder contains code
│   ├── config.py                # Global settings (Pin numbers, Constants)
│   │
│   ├── 📂 interfaces/           # TEMPLATES (Abstract Base Classes)
│   │   ├── __init__.py
│   │   ├── motor_interface.py   # Defines "What is a motor?"
│   │   └── sensor_interface.py  # Defines "What is a sensor?"
│   │
│   ├── 📂 drivers/              # HARDWARE DRIVERS (Talking to chips)
│   │   ├── __init__.py
│   │   ├── tmc2208.py           # TMC2208 specific code
│   │   ├── as5600.py            # AS5600 specific code
│   │   └── imx296_camera.py     # Global Shutter Camera code
│   │
│   └── 📂 controllers/          # BRAINS (The logic)
│       ├── __init__.py
│       └── motion_controller.py # Logic linking Encoder to Motor
│
├── 📂 tests/                    # TESTS (Safety Checks)
│   └── test_motors.py           # Code to test hardware safely
│
├── .gitignore                   # Files for Git to ignore (e.g., passwords)
├── requirements.txt             # List of libraries (gpiozero, picamera2)
└── main.py                      # THE STARTER (Run this file!)