# flake.nix
{
  description = "Godot-Mono dev environment with recursive .so linking";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          config.allowUnfree = true;
        };
      in
      {
        devShells.default = pkgs.mkShell {
          name = "godot-mono-dev";

          buildInputs = with pkgs; [
            patchelf
            godot-mono
          ];

          shellHook = ''
            echo "Activating Godot-Mono dev shell..."

            LD_LIBRARY_PATH=""

            # Recursively find all directories containing .so files
            export LD_LIBRARY_PATH=$(find "$HOME/ASTRA/basestation-game/addons" -type f -name "*.so" -exec dirname {} \; | sort -u | tr '\n' ':' | sed 's/:$//'):$LD_LIBRARY_PATH

            echo "LD_LIBRARY_PATH set to:"
            echo "$LD_LIBRARY_PATH"
          '';
        };
      }
    );
}
