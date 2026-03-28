{
  inputs = {
    modernpkgs.url = "github:NixOS/nixpkgs/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/develop";
    astra-msgs.url = "github:SHC-ASTRA/astra_msgs/92e0442b59ee624f6979ffec5c88c2f9023b54c3";
  };
  outputs =
    {
      self,
      nixpkgs,
      astra-msgs,
      modernpkgs,
      nix-ros-overlay,
    }@inputs:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
        astra_msgs_pkgs = astra-msgs.packages.${system};

        rosDistro = "humble";

        mpkgs = import modernpkgs { inherit system; };
      in
      {
        devShells.default = pkgs.mkShell {
          name = "basestation-game";
          packages =
            (
              with mpkgs;
              with mpkgs.godotPackages_4_6;
              [
                colcon
                (
                  with pkgs.rosPackages.${rosDistro};
                  buildEnv {
                    paths = [
                      ros2cli
                      ros2run
                      ros-core
                      control-msgs
                      rosbridge-suite
                      ament-cmake-core
                      python-cmake-module
                    ];
                  }
                )
                godot-mono
                openssl_3_5
                astra_msgs_pkgs.astra-msgs
              ]
            )
            ++ (with pkgs; [
              roslyn
              dotnet-sdk_8
              omnisharp-roslyn
            ]);

          extraPaths = [ ];

          env = {
            ASTRAMSGS = "${inputs.astra-msgs.outPath}";

            CONTROLMSGS = "${pkgs.rosPackages.${rosDistro}.control-msgs.outPath}/share/control_msgs/";

            ROSBRIDGESERVER = "${pkgs.rosPackages.${rosDistro}.rosbridge-server.outPath}";

            ROSCOMPILER = "./ROS/Compiler";
          };

          shellHook = ''
            if [ ! -d $ROSCOMPILER/bin/Release/net8.0 ]; then
                dotnet run --configuration Release --debug False --project $ROSCOMPILER
                rm $ROSCOMPILER/obj -r
            else
                dotnet run --no-build --configuration Release --debug False --project $ROSCOMPILER
            fi
          '';
        };

        packages = {
          default = mpkgs.stdenv.mkDerivation {
            pname = "basestation-game";
            version = "0.5.0";
            src = ./.;

            nativeBuildInputs =
              with mpkgs;
              with mpkgs.godotPackages_4_6;
              [
                mono
                unzip
                godot-mono
                makeWrapper
                dotnet-sdk_8
                export-template
              ];

            buildPhase = ''
                	      runHook preBuild

                	      export HOME=$TMPDIR

                	      mkdir -p $HOME/.local/share/godot
                	      ln -s ${mpkgs.godotPackages_4_6.export-template-mono}/share/godot/export_templates $HOME/.local/share/godot

                	      mkdir -p $out/bin/
                	      godot-mono --headless --verbose --export-release "nix ${system}" $out/bin/basestation-game
               		      ls > $out/log

                	      runHook postBuild
              	    '';

            installPhase = ''
              runHook preInstall
              chmod 775 -R $out
              ls -lsa $out > $out/log2
              runHook postInstall
            '';
          };
        };
      }
    );
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
