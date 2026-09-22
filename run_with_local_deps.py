"""用系统内置 Python 运行本工程，并加载随工程保存的 kRPC 依赖。"""

from pathlib import Path
import runpy
import sys


ROOT = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT), str(ROOT / ".test-python-deps")]

if len(sys.argv) < 2:
    raise SystemExit("用法: run_with_local_deps.py <脚本.py> [脚本参数...]")

script = ROOT / sys.argv[1]
sys.argv = [str(script), *sys.argv[2:]]
runpy.run_path(str(script), run_name="__main__")
