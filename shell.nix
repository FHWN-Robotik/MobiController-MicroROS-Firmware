{ pkgs ? import <nixpkgs> { }, ... }:

pkgs.mkShell {
  nativeBuildInputs = with pkgs; [
    cmake
    gdb
    gcc-arm-embedded-12
    clang-tools
    openocd
    dfu-util
  ];

  shellHook = ''
    unset SOURCE_DATE_EPOCH
  '';
}
