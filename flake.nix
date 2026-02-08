{
  inputs = {
    modernpkgs.url = "github:NixOS/nixpkgs/nixos-25.11";
    nixpkgs = {
      url = "github:NixOS/nixpkgs/nixos-25.05";
      follows = "nix-ros-overlay/nixpkgs";
    };
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/develop";
    # astra-msgs.url = "github:SHC-ASTRA/astra_msgs";
  };
  outputs = { self, nix-ros-overlay, modernpkgs, nixpkgs }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
        mpkgs = import modernpkgs { inherit system; };

        # astra_msgs_pkgs = astra-msgs.packages.${system};
      in {
        devShells.default = pkgs.mkShell {
          name = "basestation-game";
          packages = with pkgs; [
            colcon
            mpkgs.godotPackages_4_6.godot-mono
            # astra_msgs_pkgs.astra-msgs
            (with pkgs.rosPackages.humble;
              buildEnv {
                paths = [
                  ros-core
                  ros2cli
                  ros2run
                  ament-cmake-core
                  python-cmake-module
                  rosbridge-suite
                ];
              })
          ];

          # # wrap the installed script so PATH includes the required tools at runtime
          # # set PATH to include the binaries from buildInputs
          # postInstall = ''
          #   wrapProgram $out/start_rosbridge.sh \
          #     --set PATH ${pkgs.lib.makeBinPath (with pkgs; [ ])}
          # '';
          env = { };
        };

        default = pkgs.mkDerivation {
          pname = "basestation-game";
          version = "0.1.0";
          src = ./.;

          nativeBuildInputs = with modernpkgs; [
            mono
            unzip
            makeWrapper
            godotPackages_4_6.godot-mono
          ];

          buildInputs = with modernpkgs; [
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
              	      ln -s ${modernpkgs.godotPackages_4_6.export-template-mono}/share/godot/export_templates $HOME/.local/share/godot

              	      mkdir -p $out/bin/
              	      godot-mono --path . --headless --export-release "nix ${system}" $out/bin/basestation-game
             		      ls > $out/log

              	      runHook postBuild
            	    '';

          postInstall = "\n";
        };
      });
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys =
      [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
