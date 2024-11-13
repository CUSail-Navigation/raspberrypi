# raspberrypi

This repository stores the most recent stable version of the CUSail codebase.
It is designed to run on a Raspberry Pi 4B (2GB RAM) and is written in
Python 3.7.3.

Documentation:
https://cusail-navigation.github.io/raspberrypi/

To generate the documentation, follow this guide: 
https://cusail-navigation.github.io/intrasite/website/documentation.html

Run from the __raspberrypi__ directory:
- To run the navigation algorithm: python3 -m nav_algo
- To run all unit test cases: python3 -m unittest

Run from the __raspberrypi__/__nav_algo__ directory:
- To run the event algorithm test cases: python3 -m event_tests (requires matplotlib)
