"""Adapt CMake's diagnostic/preprocessor probes to Zig 0.13's C++ driver."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile

zig=sys.argv[1]
args=[arg for arg in sys.argv[2:] if arg!='-Wl,-v']
temporary=None
try:
    # Zig 0.13 treats '-o -' as a filename, whereas CMake expects stdout.
    if '-E' in args and '-o' in args and args[args.index('-o')+1]=='-':
        fd,temporary=tempfile.mkstemp(suffix='.i',dir=os.environ.get('ZIG_GLOBAL_CACHE_DIR'))
        os.close(fd)
        args[args.index('-o')+1]=temporary
    result=subprocess.run([zig,'c++','-target','x86_64-linux-gnu.2.35',*args])
    if result.returncode==0 and temporary:
        sys.stdout.buffer.write(Path(temporary).read_bytes())
    sys.exit(result.returncode)
finally:
    if temporary:Path(temporary).unlink(missing_ok=True)
