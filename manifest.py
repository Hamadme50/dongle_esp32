include('$(PORT_DIR)/boards/manifest.py')
module("main.py", base_path=".", opt=2)
module("serial.py", base_path=".", opt=2)
module("protocol_config.py", base_path=".", opt=2)
