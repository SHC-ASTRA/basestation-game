{
  inputs = {
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    modernpkgs.url = "github:NixOS/nixpkgs/master";
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/develop";
    astra-msgs.url = "github:SHC-ASTRA/astra_msgs/2303314514094c1bdefe7a60aad32cbba419b3ba";
    basestation-cameras.url = "github:SHC-ASTRA/basestation-cameras";
  };
  outputs =
    {
      self,
      nixpkgs,
      astra-msgs,
      modernpkgs,
      nix-ros-overlay,
      basestation-cameras,
    }@inputs:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (
      system:
      let
        rospkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
        astra_msgs_pkgs = astra-msgs.packages.${system};

        rosDistro = "humble";

        pkgs = import modernpkgs { inherit system; };
      in
      {
        devShells.default = rospkgs.mkShell {
          name = "basestation-game";
          packages = (
            with pkgs;
            with pkgs.godotPackages_4_6;
            [
              colcon
              (
                with rospkgs.rosPackages.${rosDistro};
                buildEnv {
                  paths = [
                    tf2-msgs
                    ros2cli
                    ros2run
                    ros-core
                    control-msgs
                    rosbridge-suite
                    diagnostic-msgs
                    ament-cmake-core
                    python-cmake-module
                  ];
                }
              )
              roslyn-ls
              godot-mono
              dotnet-sdk_9
              astra_msgs_pkgs.astra-msgs
              basestation-cameras.packages.${pkgs.stdenv.hostPlatform.system}.default
              basestation-cameras.packages.${pkgs.stdenv.hostPlatform.system}.cameracli
            ]
          );

          env = {
            ASTRAMSGS = "${inputs.astra-msgs.outPath}";

            CONTROLMSGS = "${rospkgs.rosPackages.${rosDistro}.control-msgs.outPath}/share/control_msgs/";
            DIAGNOSTICMSGS = "${
              rospkgs.rosPackages.${rosDistro}.diagnostic-msgs.outPath
            }/share/diagnostic_msgs/";

            ROSBRIDGE = "${rospkgs.rosPackages.${rosDistro}.rosbridge-suite.outPath}";

            ROSCOMPILER = "./ROS/Compiler";

            DOTNET_ROOT = "${pkgs.dotnet-sdk_9}";
          };
          shellHook = ''
            mkdir -p .dev-bin
            ln -sf "${pkgs.roslyn-ls.outPath}/bin/Microsoft.CodeAnalysis.LanguageServer" .dev-bin/roslyn-ls
          '';
        };
      }
    );
  nixConfig = {
    # Cache to pull ros packages from
    extra-substituters = [
      "https://ros.cachix.org"
      "https://attic.iid.ciirc.cvut.cz/ros"
    ];
    extra-trusted-public-keys = [
      "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="
      "ros:JR95vUYsShSqfA1VTYoFt1Nz6uXasm5QrcOsGry9f6Q="
    ];
  };
}
