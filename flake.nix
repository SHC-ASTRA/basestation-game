{
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
	    inherit system;
          }
        );
    in
    {
      devShells = forEachSupportedSystem (
        { pkgs, system }:
        {
          default = pkgs.mkShell {
            packages = with pkgs.godotPackages_4_5; [
              godot-mono
            ];

            env = { };
            shellHook = '''';
          };
        }
      );

      packages = forEachSupportedSystem (
        { pkgs, system }:
        {
          default = pkgs.stdenv.mkDerivation {
            pname = "basestation-game";
            version = "0.1.0";
            src = ./.;

            nativeBuildInputs = with pkgs; [
              godotPackages_4_5.godot-mono
              unzip
            ];

            buildInputs = [

            ];

            buildPhase = ''
              	      runHook preBuild

              	      export HOME=$TMPDIR

              	      mkdir -p $HOME/.local/share/godot
              	      ln -s ${pkgs.godotPackages_4_5.export-template-mono}/share/godot/export_templates $HOME/.local/share/godot

              	      mkdir -p $out/bin/
              	      godot-mono --path project --headless --export-release "nix ${system}" $out/bin/basestation-game
		      ls > $out/log

              	      runHook postBuild
              	    '';
          };
        }
      );
    };
}
