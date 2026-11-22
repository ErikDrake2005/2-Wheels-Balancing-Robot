"""Copy canonical wire headers into the standalone Arduino sketch package."""
from pathlib import Path
import shutil

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
for name in ("GiaoThucChanDoanEspNow.h", "GiaoThucKhaoSatRung.h"):
    shutil.copyfile(ROOT / "include" / "model" / name, HERE / "espnow_rung_receiver" / name)
    print(f"Synced {name}")
