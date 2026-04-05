import math
from collections import defaultdict

import odrive
import fibre.remote_object as ro


def format_value(v):
    if isinstance(v, float):
        if math.isinf(v):
            return "float('inf')" if v > 0 else "float('-inf')"
        if math.isnan(v):
            return "float('nan')"
        return repr(v)
    if isinstance(v, bool):
        return repr(v)
    return repr(v)


def walk_collect(obj, prefix, assignments_by_ns):
    for name, attr in sorted(obj._remote_attributes.items(), key=lambda x: x[0]):
        path = f"{prefix}.{name}" if prefix else name
        if isinstance(attr, ro.RemoteProperty):
            if not attr._can_read:
                continue
            try:
                val = attr.get_value()
            except Exception as e:
                full = f"odrv0.{path}"
                ns, var = full.rsplit(".", 1)
                assignments_by_ns[ns].append(
                    (var, f"# odrv0.{path}  # read error: {e}")
                )
                continue
            full = f"odrv0.{path}"
            ns, var = full.rsplit(".", 1)
            line = f"{full} = {format_value(val)}"
            assignments_by_ns[ns].append((var, line))
        elif isinstance(attr, ro.RemoteObject):
            walk_collect(attr, path, assignments_by_ns)


def main():
    odrv0 = odrive.find_any()
    by_ns = defaultdict(list)
    walk_collect(odrv0, "", by_ns)

    first_block = True
    for ns in sorted(by_ns.keys()):
        if not first_block:
            print()
        first_block = False
        for _var, line in sorted(by_ns[ns], key=lambda x: x[0]):
            print(line)


if __name__ == "__main__":
    main()