{
  inputs = {
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    modernpkgs.url = "github:NixOS/nixpkgs/master";
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/develop";
    astra-msgs.url = "github:SHC-ASTRA/astra_msgs/5a20f3f56964dddf962f651d871d6f3537b1e9f3";
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
              roslyn
              godot-mono
              openssl_3_6
              dotnet-sdk_8
              omnisharp-roslyn
              astra_msgs_pkgs.astra-msgs
            ]
          );

          env = {
            ASTRAMSGS = "${inputs.astra-msgs.outPath}";

            CONTROLMSGS = "${rospkgs.rosPackages.${rosDistro}.control-msgs.outPath}/share/control_msgs/";

            ROSCOMPILER = "./ROS/Compiler";
          };
        };
      }
    );
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
