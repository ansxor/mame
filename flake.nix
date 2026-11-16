{
  description = "MAME development shell (with the Rust toolchain for 3rdparty/f3rt-audio)";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { self, nixpkgs }:
    let
      systems = [ "x86_64-linux" "aarch64-linux" ];
      forAll = f: nixpkgs.lib.genAttrs systems (system: f nixpkgs.legacyPackages.${system});
    in {
      devShells = forAll (pkgs: {
        default = pkgs.mkShell {
          packages = with pkgs; [
            # build tools
            gnumake pkg-config python3 which
            # f3rt-audio (cxx.rs bridge, built by `make f3rt_audio`)
            cargo rustc
            # osd=sdl
            SDL2 SDL2_ttf fontconfig freetype alsa-lib pipewire libpulseaudio
            libGL libxcb libx11 libxext libxi libxinerama libxrandr
          ];
        };
      });
    };
}
