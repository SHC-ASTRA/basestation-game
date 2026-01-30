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
            mpkgs = import inputs.modernpkgs { inherit system; };
            pkgs = import inputs.nixpkgs { inherit system; };
            inherit system;
          }
        );
    in
    {
      devShells = forEachSupportedSystem (
        { mpkgs, pkgs, system }:
        {
          default = pkgs.mkShell {
            packages =
              with mpkgs;
              [
                dotnet-sdk_10
                godotPackages_4_6.godot-mono
              ];

            env = { };
            shellHook = '''';
          };
        }
      );

      packages = forEachSupportedSystem (
        { mpkgs, pkgs, system }:
        {
          default = mpkgs.stdenv.mkDerivation {
            pname = "basestation-game";
            version = "0.1.0";
            src = ./.;

            nativeBuildInputs = with mpkgs; [
              mono
              unzip
              makeWrapper
              dotnet-sdk_10
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
                	      ln -s ${pkgs.godotPackages_4_6.export-template-mono}/share/godot/export_templates $HOME/.local/share/godot

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
