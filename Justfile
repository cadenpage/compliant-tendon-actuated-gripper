set shell := ["zsh", "-cu"]

default:
    @just --list

setup:
    python3 -m venv .venv
    .venv/bin/python -m pip install -r requirements.txt

prb:
    python3 scripts/prb.py

animate:
    python3 scripts/animation.py

all: prb animate

clean:
    rm -f imgs/prb-analysis.png imgs/finger-curl.gif
