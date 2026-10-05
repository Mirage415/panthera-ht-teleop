"""Generate narrowly scoped udev rules from verified local USB paths."""
import argparse
import json
from pathlib import Path
import re


def render_rules(config, user, sys_tty=Path('/sys/class/tty')):
    if not re.fullmatch(r'[a-zA-Z_][a-zA-Z0-9_-]*', user) or user == 'root':
        raise ValueError('Specify a non-root local login name')
    rules, seen = [], set()
    for role in ('leader', 'follower'):
        dev = Path(config[role]).resolve(strict=True)
        device = (sys_tty / dev.name / 'device').resolve(strict=True)
        board = next((p for p in [device, *device.parents] if (p / 'idVendor').exists()), None)
        if board is None or (board / 'idVendor').read_text().strip() != 'caf1' or (board / 'idProduct').read_text().strip() != 'ffff':
            raise ValueError(f'{role}: not a supported Livelybot board')
        if not re.fullmatch(r'[0-9]+-[0-9]+(?:\.[0-9]+)*', board.name):
            raise ValueError('Unexpected USB topology')
        if board.name in seen:
            raise ValueError('Leader and follower must use different boards')
        seen.add(board.name)
        rules.append(f'SUBSYSTEM=="tty", ATTRS{{idVendor}}=="caf1", ATTRS{{idProduct}}=="ffff", KERNELS=="{board.name}", OWNER="{user}", GROUP="dialout", MODE="0660"')
    return '\n'.join(rules) + '\n'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--user', required=True)
    args = parser.parse_args()
    config = json.loads(Path(__file__).with_name('teleop-config.json').read_text())
    print(render_rules(config, args.user), end='')
