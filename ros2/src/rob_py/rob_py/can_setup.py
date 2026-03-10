"""
can_setup.py
------------
Helper that brings up a SocketCAN interface before any node tries to open it.

Two commands are run with sudo (passwordless sudo must be configured for
`ip` on the target machine, or the node must be launched with appropriate
privileges):

    sudo ip link set <channel> type can bitrate <bitrate>
    sudo ip link set <channel> up

If the interface is already UP the commands are skipped gracefully.
"""

import subprocess
import logging

logger = logging.getLogger(__name__)


def setup_can_interface(channel: str = 'can0', bitrate: int = 1_000_000) -> bool:
    """Bring up a SocketCAN interface.

    Parameters
    ----------
    channel:
        SocketCAN interface name, e.g. ``'can0'``.
    bitrate:
        CAN bitrate in bps (default 1 000 000).

    Returns
    -------
    bool
        ``True`` if the interface is up after the call, ``False`` on error.
    """
    # Check current state to avoid redundant calls
    state = _get_operstate(channel)
    if state == 'UP':
        logger.info("CAN interface '%s' is already UP — skipping setup.", channel)
        return True

    logger.info(
        "Setting up CAN interface '%s' at %d bps ...", channel, bitrate)

    cmds = [
        ['sudo', 'ip', 'link', 'set', channel, 'type', 'can',
         'bitrate', str(bitrate)],
        ['sudo', 'ip', 'link', 'set', channel, 'up'],
    ]

    for cmd in cmds:
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=5,
            )
            if result.returncode != 0:
                logger.error(
                    "Command %s failed (rc=%d): %s",
                    ' '.join(cmd), result.returncode, result.stderr.strip())
                return False
        except FileNotFoundError:
            logger.error("'ip' command not found. Install iproute2.")
            return False
        except subprocess.TimeoutExpired:
            logger.error("Command %s timed out.", ' '.join(cmd))
            return False

    state = _get_operstate(channel)
    if state == 'UP':
        logger.info("CAN interface '%s' is UP.", channel)
        return True

    logger.error(
        "CAN interface '%s' did not come UP after setup (state=%s).",
        channel, state)
    return False


def teardown_can_interface(channel: str = 'can0') -> None:
    """Bring down the CAN interface (optional, called on node shutdown)."""
    subprocess.run(
        ['sudo', 'ip', 'link', 'set', channel, 'down'],
        capture_output=True, timeout=5)
    logger.info("CAN interface '%s' brought down.", channel)


# ---------------------------------------------------------------------------
def _get_operstate(channel: str) -> str:
    """Return the operstate string (UP / DOWN / UNKNOWN / …) or empty string."""
    try:
        result = subprocess.run(
            ['cat', f'/sys/class/net/{channel}/operstate'],
            capture_output=True, text=True, timeout=3)
        return result.stdout.strip().upper()
    except Exception:
        return ''
