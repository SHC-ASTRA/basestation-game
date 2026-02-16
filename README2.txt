For if the csproj gets damaged.
Shouldn't need to use the first package, as of 2/14/2026,
    as we include custom astra-ros-sharp and astra-ros-bridge
    so that we have QOS support.

dotnet add package Siemens.RosSharp.RosBridgeClient.ROS2 --version 2.2.0
dotnet add package WebSocketSharp --version 1.0.3-rc11

NoWarn should be a sub-property of WebSocketSharp, eg:
    <PackageReference Include="WebSocketSharp" Version="1.0.3-rc11">
        <NoWarn>NU1701</NoWarn>
    </PackageReference>
