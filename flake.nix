{
  inputs = {
    modernpkgs.url = "github:NixOS/nixpkgs/nixos-25.11";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/develop";
    astra-msgs.url =
      "github:SHC-ASTRA/astra_msgs/2a90e7712424d8f44093334f7a359696d9398216";
  };
  outputs = { self, nix-ros-overlay, modernpkgs, nixpkgs, astra-msgs }@inputs:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
        astra_msgs_pkgs = astra-msgs.packages.${system};

        rosDistro = "humble";

        mpkgs = import modernpkgs { inherit system; };
      in {
        devShells.default = pkgs.mkShell {
          name = "basestation-game";
          packages = with pkgs; [
            colcon
            mpkgs.godotPackages_4_6.godot-mono
            (with pkgs.rosPackages.${rosDistro};
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
            astra_msgs_pkgs.astra-msgs
          ];

          extraPaths = [ ];

          env = {
            ASTRAMSGS = "${inputs.astra-msgs.outPath}";
            # ROSBRIDGE =
            #   "${pkgs.rosPackages.${rosDistro}.rosbridge-suite.outPath}";
            ROSBRIDGESERVER =
              "${pkgs.rosPackages.${rosDistro}.rosbridge-server.outPath}";
            # ROSBRIDGELIBRARY =
            #   "${pkgs.rosPackages.${rosDistro}.rosbridge-library.outPath}";
            ROSCOMPILER = "./ROS/Compiler";
          };

          shellHook = ''
            dotnet run --no-build --configuration Release --debug False --project $ROSCOMPILER
          '';
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
        };
      });
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys =
      [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
