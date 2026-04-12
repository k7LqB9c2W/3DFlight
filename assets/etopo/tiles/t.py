from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed
from urllib.request import Request, urlopen
from urllib.error import HTTPError, URLError
import shutil
import time

BASE_URL = "https://www.ngdc.noaa.gov/mgg/global/relief/ETOPO2022/data/15s/15s_surface_elev_gtif/"

IRAN_TILES = [
    "ETOPO_2022_v1_15s_N15E030_surface.tif",
    "ETOPO_2022_v1_15s_N15E045_surface.tif",
    "ETOPO_2022_v1_15s_N15E060_surface.tif",
    "ETOPO_2022_v1_15s_N30E030_surface.tif",
    "ETOPO_2022_v1_15s_N30E045_surface.tif",
    "ETOPO_2022_v1_15s_N30E060_surface.tif",
]

MAX_WORKERS = 4
RETRIES = 3
TIMEOUT_SECONDS = 60


def download_one(filename: str, out_dir: Path) -> str:
    dest = out_dir / filename
    temp = out_dir / f"{filename}.part"

    if dest.exists() and dest.stat().st_size > 0:
        return f"SKIP  {filename}"

    url = BASE_URL + filename
    headers = {"User-Agent": "Mozilla/5.0"}

    last_error = None
    for attempt in range(1, RETRIES + 1):
        try:
            req = Request(url, headers=headers)
            with urlopen(req, timeout=TIMEOUT_SECONDS) as resp, open(temp, "wb") as f:
                shutil.copyfileobj(resp, f, length=1024 * 1024)

            temp.replace(dest)
            size_mb = dest.stat().st_size / (1024 * 1024)
            return f"DONE  {filename}  ({size_mb:.1f} MB)"

        except (HTTPError, URLError, TimeoutError, OSError) as e:
            last_error = e
            if temp.exists():
                try:
                    temp.unlink()
                except OSError:
                    pass
            if attempt < RETRIES:
                time.sleep(1.5 * attempt)

    return f"FAIL  {filename}  ({last_error})"


def main():
    out_dir = Path(__file__).resolve().parent
    out_dir.mkdir(parents=True, exist_ok=True)

    print("Downloading Iran ETOPO tiles:")
    for name in IRAN_TILES:
        print(" ", name)
    print()

    with ThreadPoolExecutor(max_workers=MAX_WORKERS) as executor:
        futures = {executor.submit(download_one, name, out_dir): name for name in IRAN_TILES}

        for future in as_completed(futures):
            print(future.result())

    print("\nAll done.")


if __name__ == "__main__":
    main()