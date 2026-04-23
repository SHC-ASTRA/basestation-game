rm -r $ROSCOMPILER/bin
dotnet run --configuration Release --debug False --no-self-contained --project $ROSCOMPILER
rm -r $ROSCOMPILER/obj
