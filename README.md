# ASTRA basestation-game

Welcome! This is the repository for UAH's ASTRA Base Station for the University
Rover Challenge 2025-26.

ASTRA is a project under the AutoSat branch of Space Hardware Club at the
University of Alabama in Huntsville.

## If you're on basestation and no one is there to help you

1. Make sure everything is plugged in correctly:

    A. If no tracking antenna, ethernet should be connected from the back of basestation to the NON-RED side of the POE injector.
        Red side should be connected to the antenna dish.
   
    B. Both the POE injector and basestation box should be plugged into the UGREEN.
   
    C. A keyboard, mouse, and controller plugged into the left hand side of basestation is optimal.
   
    D. The deck should be plugged in to the USB-C cable which comes through the mid-left hole

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

- Open the Godot editor, open the ROS folder on the bottom right and run
  CompileRosMsgs.cs by right clicking on it, then re-run the program.

- Ping Roald Schaum in the ASTRA discord

## Contributors

For anyone adding to this repository, please add your name to the README before
making a pull request.

- Roald Schaum
- David Sharpe

### External credit

A big think you to user "AngryMeenky" for their MapTileProvider plugin.
