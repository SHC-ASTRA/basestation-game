# Main changes

In accordance with the Apache 2.0 License, this file is
required to track and maintain a list of changes made to
the files herein and in relation to the parent repository.

The parent repository is located at
[siemens/ros-sharp](https://github.com/siemens/ros-sharp).

## Primary changes are as follows:

- Removed RosSharpMessageCompiler from source to employ
  a custom implementation

- Made ActionAutoGen, MessageAutoGen, ServiceAutoGen,
  and MsgAutoGenUtilities static classes as all their
  members were static as well.

- Added custom package name support to ActionAutoGen, MessageAutoGen, and ServiceAutoGen

- Removed all Ros1 support (unused in our use case)

- Changed base namespace to RosSharp.RosBridgeClient.CMessageGeneration, for compat with the original package.

- Minor optimizations
