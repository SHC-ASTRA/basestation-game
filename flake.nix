{
  inputs = {
    modernpkgs.url = "github:NixOS/nixpkgs/nixos-25.11";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/develop";
    nuget-packageslock2nix.url = "github:mdarocha/nuget-packageslock2nix"; 
    astra-msgs.url = "github:SHC-ASTRA/astra_msgs/92e0442b59ee624f6979ffec5c88c2f9023b54c3";
  };
  outputs = 
    { self, nix-ros-overlay, modernpkgs, nuget-packageslock2nix, nixpkgs, astra-msgs }@inputs:
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
                export-template
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

        packages.default = pkgs.buildDotnetModule {
            pname = "basestation-game";
            version = "0.5.0";
            src = ./.;

            projectFile = ./basestation-game.csproj; # Use $ dotnet restore to generate the pachage-lock.json file
            nugetDeps = nuget-packageslock2nix.lib { 
              inherit system;
              name = "deps"; 
              lockfiles = [ ./packages.lock.json ];
              excludePackages = [ "Godot.SourceGenerators-4.6.0" "GodotSharp-4.6.0" "GodotSharpEditor-4.6.0" "Godot.NET.Sdk-4.6.0"];
 
            };

            dotnet-sdk = pkgs.dotnetCorePackages.sdk_8_0; 
            dotnet-runtime = pkgs.dotnetCorePackages.runtime_8_0;

            nativeBuildInputs = with mpkgs; [
              mono
              unzip
              makeWrapper
              #dotnetCorePackages.sdk_8_0_1xx-bin
              which
              tree
          #      godotPackages_4_6.export-templates-mono-bin


            ];

            buildInputs = with mpkgs; [
              mono
              unzip
              makeWrapper

              tree
  
  #godotPackages_4_6.export-templates-mono-bin

              which
              #dotnetCorePackages.sdk_8_0_1xx-bin
              godotPackages_4_6.godot-mono
              godotPackages_4_6.export-template

            ];

            buildPhase = ''
              runHook preBuild
              export HOME=$TMPDIR

              
              echo tree $HOME
              echo "WEEEEEEEE"

              echo "LLLL"
              echo "$(ls ${mpkgs.godotPackages_4_6.export-template-mono}/share/godot/export_templates/4.6.stable.mono/)"
              echo "PPPP"
              mkdir -p $HOME/.local/share/godot


              ln -s ${mpkgs.godotPackages_4_6.export-template-mono}/share/godot/export_templates/ $HOME/.local/share/godot

              mkdir -p $out/bin/



       	      godot4.6-mono --path . --headless --verbose --export-release "nix ${system}" $out/bin/basestation-game

              cd $out/bin/
              echo "$(tree -a .)"  


              
       	      runHook postBuild'';

              #postInstall = ''
               #   wrapProgram $out/bin/basestation-game --set DOTNET_ROOT ${mpkgs.dotnetCorePackages.sdk_8_0_1xx-bin}
                #'';
       
            preInstall = ''
              echo "$(tree -a .)"
            '';

            installPhase = ''
              echo "$(tree -a .)"
            '';
          };

        

      });

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys =
      [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
