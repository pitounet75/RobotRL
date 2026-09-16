# Runs after the espressif32 platform sets UPLOADER.
Import("env")

import os

if env.subst("$UPLOAD_PROTOCOL") == "esptool":
    wrapper = os.path.join(env.subst("$PROJECT_DIR"), "scripts", "esptool_nodtr.py")
    real = env.subst("$UPLOADER")
    env.Replace(UPLOADER=wrapper)
    env.Append(ENV={"PIO_REAL_ESPTOOL": real})
