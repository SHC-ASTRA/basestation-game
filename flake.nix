{
  inputs.modernpkgs.url = "github:NixOS/nixpkgs/nixos-25.11";
  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.05";

  outputs =
    inputs:
    let
      supportedSystems = [
        "x86_64-linux"
        "aarch64-linux"
      ];

      forEachSupportedSystem =
        f:
        inputs.nixpkgs.lib.genAttrs supportedSystems (
          system:
          f {
            pkgs = import inputs.nixpkgs { inherit system; };
            mpkgs = import inputs.modernpkgs { inherit system; };
            inherit system;
          }
        );
    in
    {
      devShells = forEachSupportedSystem (
        { system, mpkgs, pkgs }:
        {
          default = pkgs.mkShell {
            packages =
              with mpkgs;
              [
                godotPackages_4_6.godot-mono
              ];

            env = { };
            shellHook = '''';
          };
        }
      );

      packages = forEachSupportedSystem (
        { pkgs, mpkgs, system }:
        {
          default = mpkgs.stdenv.mkDerivation {
            pname = "basestation-game";
            version = "0.1.0";
            src = ./.;

            nativeBuildInputs = with mpkgs; [
              mono
              unzip
              makeWrapper
              godotPackages_4_6.godot-mono
            ];

            buildInputs = with mpkgs; [
              mono
              unzip
              makeWrapper
              dotnet-sdk_10
              godotPackages_4_6.godot-mono
            ];

            buildPhase = ''
                	      runHook preBuild

                	      export HOME=$TMPDIR

                	      mkdir -p $HOME/.local/share/godot
                	      ln -s ${mpkgs.godotPackages_4_6.export-template-mono}/share/godot/export_templates $HOME/.local/share/godot

                	      mkdir -p $out/bin/
                	      godot-mono --path . --headless --export-release "nix ${system}" $out/bin/basestation-game
               		      ls > $out/log

                	      runHook postBuild
              	    '';

            postInstall = ''

            '';
          };
        }
      );
    };
}
