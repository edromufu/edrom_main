"""Native Webots Python entry point, also usable without executable file mode."""
import os
from pathlib import Path
import sys

launcher = Path(__file__).resolve().with_name('bhv_sim')
os.execv('/bin/bash', ['/bin/bash', str(launcher), *sys.argv[1:]])
