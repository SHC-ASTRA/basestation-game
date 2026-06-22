# ASTRA basestation-game

Welcome! This is the repository for UAH's ASTRA Base Station for the University
Rover Challenge 2025-26.

ASTRA is a project under the AutoSat branch of Space Hardware Club at the
University of Alabama in Huntsville.

This program failed to function whatsoever at competition the cycle it was introduced. The reason is still unknown, as no testing has been able to reproduce this phenomenon.

## If you're on basestation and no one is there to help you

1. Make sure everything is plugged in correctly:

    A. If no tracking antenna, ethernet should be connected from the back of basestation to the NON-RED side of the POE injector.
        Red side should be connected to the antenna dish.
   
    B. Both the POE injector and basestation box should be plugged into the UGREEN.
   
    C. A keyboard, mouse, and controller plugged into the left hand side of basestation is optimal.
   
    D. The deck should be plugged in to the USB-C cable which comes through the mid-right hole

3. Turn on the UGREEN and make sure the output is enabled as well.

4. Wait for the deck to boot (1m 30s)

5. Press enter and type in the password

6. Press the windows key on the keyboard and open Console

7. Enter the following command:
   ```bash
   cd Documents/basestation-game && nix-develop --command "godot-mono --editor"
   ```

8. Press F5 on a keyboard, or click the Arrow icon on the top right
   of the screen on the steamdeck

If something doesn't work for whatever reason, you can try a few things:

- Close the window and the Godot game engine, restart from #1

- Run the following command in the basestation-game repository:
   ```bash
   ./RecompileC#.sh
   ```
  Relaunch the godot editor and press F5 once again.

- Ping Roald Schaum in the ASTRA discord

## Installation

This is a quickstart guide to get basestation-game up and running on any machine,
for someone that doesn't know anything about basestation-game.

1. Clone the repository:

   ```bash
   git clone https://github.com/SHC-ASTRA/basestation-game --recursive
   ```

2. If you forgot to clone the repository with submodules, use the following command to clone them:

   ```bash
   git submodule update --init --recursive
   ```

3. In the repository's folder, open a nix development shell (this will take a while to download everything):

   ```bash
   nix develop
   ```

4. In the dev shell, re-compile astra_msgs for ROS#:

   ```bash
   ./RecompileC#.sh
   ```

5. Still in the dev shell, open the Godot editor:

   ```bash
   godot-mono --editor
   ```

6. In the Godot editor, press the `F5` key to launch Basestation!

## Contributors

For anyone adding to this repository, please add your name to the README before
making a pull request.

- Roald Schaum
- David Sharpe

### External credit

A big think you to user "AngryMeenky" for their MapTileProvider plugin.
