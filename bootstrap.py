#!/usr/bin/env python3
"""
Bootstrap the configuration of the source code.  Check out any git
submodules ready doing a build.
"""

import argparse
import logging
import os
import subprocess
import sys

VERSION = '0.0.1'


def parse_args(args):
    parser = argparse.ArgumentParser(description="Application source code bootstrap")
    parser.add_argument('--verbose', action='store_true', default=False,
                        help='Set logging level to verbose')
    parser.add_argument('-V',
                        '--version',
                        action='version',
                        version=VERSION,
                        help='Show version and exit')

    ns = parser.parse_args(args)

    return ns


def main(args):
    git_args = 'git submodule update --init --no-recommend-shallow'.split()
    logging.info("Checking out git submodules: %s", ' '.join(git_args))

    environment = os.environ.copy()
    return_code = subprocess.call(git_args, env=environment)
    if return_code != 0:
        raise RuntimeError("Failed to update git submodules")

    return return_code


if __name__ == '__main__':

    parsed_args = parse_args(sys.argv[1:])

    if parsed_args.verbose:
        log_level = logging.DEBUG
    else:
        log_level = logging.INFO

    logging.basicConfig(level=log_level)

    exitval = main(parsed_args)

    sys.exit(exitval)
