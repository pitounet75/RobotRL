import json
import odrive
from odrive.configuration import get_dict
odrv0 = odrive.find_any()
axis_cfg = get_dict(odrv0.axis0, False)
print(json.dumps(axis_cfg, indent=2))