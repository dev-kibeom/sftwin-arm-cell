"""Make the inherited PTY controlling, then replace this process with the runtime."""
import fcntl
import os
import sys
import termios

# Popen(start_new_session=True) established this process as the session leader.
# Acquiring the terminal preserves normal Ctrl+C/SIGHUP terminal behavior while
# the GUI retains the actual runtime PID and process group through exec.
fcntl.ioctl(0, termios.TIOCSCTTY, 0)
os.execvpe(sys.argv[1], sys.argv[1:], os.environ)
